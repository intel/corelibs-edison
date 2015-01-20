/*
 * interrupt.c
 *
 * Provides a pseudo interrupt inteface which is broadly an analoge of the Arduino pin based interrupt
 * callback mechanism
 * We don't support 'real' interrupts from kernel to user-space right now since that's way out of scope
 *
 * Author: Bryan O'Donoghue <bryan.odonoghue@intel.com>
 */
#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <fcntl.h>              /* Obtain O_* constant definitions */
#include <netinet/in.h>
#include <mqueue.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/timerfd.h>


/* ia32 port */
#include <Arduino.h>
#include <interrupt.h>
#include <trace.h>
#include <wiring_digital.h>

#define __TTYUART_IDX_TX 1
#define __TTYUART_IDX_RX 0
#define NSECS_PER_SEC 1000000000UL

#define MY_TRACE_PREFIX "interrupt"

#define timeradd(a, b, result)				\
do {							\
	result.tv_sec = a.tv_sec + b.tv_sec;		\
	result.tv_nsec = a.tv_nsec + b.tv_nsec;		\
	if (result.tv_nsec >= NSECS_PER_SEC) {		\
		++result.tv_sec;			\
		result.tv_nsec -= NSECS_PER_SEC;	\
	}						\
}while(0)


/*************************** Static ****************************/
struct interrupt {
	int ihandle;
	uint8_t pin;			// for reference
	int pin_index;			// pinGetIndex
	void (*callback)(void);		// function pointer
	uint32_t mode;
	struct interrupt * pnext;
	struct interrupt * pprev;
};

struct interrupt_desc {
	uint32_t irq_count;
	int initialised;
	int pipe_tx_rx[2];		// Transmit message queue
	int hpet_handle;		// Handle to /dev/rtc - the source of our timers
	pthread_t thread;
	pthread_barrier_t barrier;
	pthread_mutex_t mutex;
	struct interrupt *phead;
};

/*
 * From Linux GPIO lib documentation:
 *
 * Sysfs attribute files are pollable.  The idea is that you read
 * the content and then you use 'poll' or 'select' to wait for
 * the content to change.  When the content changes (assuming the
 * manager for the kobject supports notification), poll will
 * return POLLERR|POLLPRI, and select will return the fd whether
 * it is waiting for read, write, or exceptions.
 * Once poll/select indicates that the value has changed, you
 * need to close and re-open the file, as simply seeking and reading
 * again will not get new data, or reset the state of 'poll'.
 * Reminder: this only works for attributes which actively support
 * it, and it is not possible to test an attribute from userspace
 * to see if it supports poll (Nether 'poll' or 'select' return
 * an appropriate error code).  When in doubt, set a suitable timeout value.
*/

static struct interrupt_desc idesc;

/**
 * interrupt_main
 *
 */
static void interrupt_main(void * pargs)
{
	int loop = 1, ret = 0, max = 0;
	fd_set fdset, fdset_except;
	char dummy;
	uint64_t elapsed;
	extern int errno;
	struct interrupt * pinterrupt = NULL;

	pthread_barrier_wait(&idesc.barrier);

	/* Loop waiting for interrupt or shutdown command */
	while (loop) {
		/* zero */
		FD_ZERO(&fdset);
		FD_ZERO(&fdset_except);

		/* Add kick listener */
		FD_SET(idesc.pipe_tx_rx[__TTYUART_IDX_RX], &fdset);
		max = idesc.pipe_tx_rx[__TTYUART_IDX_RX];

		/* Add elements - iterate through linked list of active elements */
		pthread_mutex_lock(&idesc.mutex);

		pinterrupt = idesc.phead;
		while (pinterrupt != NULL) {
			/* Timer interrupts will be attached to no GPIO pin */
			if (pinterrupt->pin_index == PIN_EINVAL) {
				FD_SET(pinterrupt->ihandle, &fdset);
			}
			else {
				FD_SET(pinterrupt->ihandle, &fdset_except);
				/* Required for GPIOLib handles before select() - very nasty !*/
				read(pinterrupt->ihandle, &dummy, 1);
			}
			max = max < pinterrupt->ihandle ? pinterrupt->ihandle : max;
			pinterrupt = pinterrupt->pnext;
		}

		pthread_mutex_unlock(&idesc.mutex);

		/* Select on the FD set - infinite timeout */
		ret = select(max + 1, &fdset, 0, &fdset_except, NULL);

		/* select returned an error */
		if (unlikely(ret < 0)) {
			trace_error("%s Critical fault during select: ", __func__,
					strerror(errno));
			loop = 0;
			continue;
		}

		/* timeout */
		if (unlikely(!ret)) {
			trace_debug("%s Select timeout", __func__);
			continue;
		}

		/* pipe */
		if (FD_ISSET(idesc.pipe_tx_rx[__TTYUART_IDX_RX], &fdset)) {
			/* Update loop */
			ret = read(idesc.pipe_tx_rx[__TTYUART_IDX_RX], (char*) &loop,
					sizeof(loop));
			if (ret < 0) {
				if ((ret = !EAGAIN) && (ret != EWOULDBLOCK)) {
					/* Critical - exit out */
					trace_error("%s Unable to read serial", __func__);
					loop = 0;
				}
			}
		}

		pthread_mutex_lock(&idesc.mutex);
		pinterrupt = idesc.phead;

		/* Interrupt */
		while (pinterrupt != NULL) {
			if (FD_ISSET(pinterrupt->ihandle, &fdset_except)) {
				/* Pin based interrupt */
				pinterrupt->callback();

				/* Need to reopen see documentation in Linux GPIO Lib */
				pinterrupt->ihandle = pinHandleReopen(pinterrupt->pin_index);
				if (pinterrupt->ihandle < 0){
					trace_error("%s Critical error unable to reopen pin %d "
							"handle - as required by gpiolib for select()!",
							__func__, pinterrupt->pin);
					pthread_exit(0);
				}
			}
			else if (FD_ISSET(pinterrupt->ihandle, &fdset)) {
				/* Timer based interrupt */
				pinterrupt->callback();
				if (read(pinterrupt->ihandle, &elapsed, sizeof(elapsed)) != sizeof(elapsed)) {
					trace_error("%s Unable to read timerfd", __func__);
					loop = 0;
				}
			}
			pinterrupt = pinterrupt->pnext;
		}
		pthread_mutex_unlock(&idesc.mutex);
	}

	/* Exit out */
	pthread_exit(NULL);
}

/**
 * addIRQEntry
 *
 * Adds either a pin based or timer based "interrupt" to the linked list of IRQs
 * Calling context must hold pthread_mutex_lock(&idesc.mutex); and subsequently release
 */
static int addIRQEntry(int ihandle, uint32_t pin, void (*callback)(void), uint32_t mode)
{
	int ret = 0;
	struct interrupt * pinterrupt = NULL;

	/* add new entry to end of linked list */
	pinterrupt = malloc(sizeof(struct interrupt));
	if (pinterrupt == NULL){
		trace_error("%s oom malloc %d bytes fail!", __func__, sizeof(struct interrupt));
		ret = -ENOMEM;
		goto done;
	}
	pinterrupt->callback = callback;
	pinterrupt->pin = pin;
	pinterrupt->ihandle = ihandle;
	pinterrupt->pin_index = pinGetIndex(pin);
	pinterrupt->mode = mode;
	pinterrupt->pnext = NULL;
	pinterrupt->pprev = NULL;

	if (idesc.phead == NULL){
		idesc.phead = pinterrupt;
	}else{
		pinterrupt->pprev = idesc.phead;

		if(idesc.phead->pnext != NULL){
			pinterrupt->pnext = idesc.phead->pnext;
			pinterrupt->pnext->pprev = pinterrupt;
		}
		idesc.phead->pnext = pinterrupt;
	}
	idesc.irq_count++;
done:
	return ret;
}
/*************************** Global ****************************/

/**
 * Attach a callback to an gpio 'interrupt' asynchronous to sketch loop();
 *
 * pin: the pin number
 * function : the function to call when the interrupt occurs
 * mode: defines when the interrupt should be triggered - LOW , CHANGE, RISING, FALLING, HIGH
 */
void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode)
{
	int ihandle = -1;
	int loop = 1;
	struct interrupt * pinterrupt = idesc.phead;
	int ret = 0;

	/* error check */
	if (callback == NULL){
		trace_error("%s -EINVAL callback is null", __func__);
		return;
	}

	/* lock */
	pthread_mutex_lock(&idesc.mutex);

	/* configure pin mode */
	if (pinModeIRQ(pin, mode)){
		trace_error("%s cannot configure pin %d to mode %d!", __func__, pin, mode);
		goto done;
	}

	/* ensure we have a persistent handle */
	ihandle = pin2gpiohandle(pin);
	if (ihandle < 0){
		trace_error("%s Arduino pin %d has no persistent gpiolib handle", __func__, pin);
		goto done;
	}

	/* check to see if this pin is already configured.. */
	while(pinterrupt != NULL){
		if(pinterrupt->pin == pin){
			trace_error("%s detachInterrupt before trying to configure pin %d again", __func__, pin);
			goto done;
		}
		pinterrupt = pinterrupt->pnext;
	}

	ret = addIRQEntry(ihandle, pin, callback, mode);
done:
	/* unlock */
	pthread_mutex_unlock(&idesc.mutex);

	/* async notify */
	if(write(idesc.pipe_tx_rx[__TTYUART_IDX_TX], (const char*)&loop, sizeof(loop))<0){
		fprintf(stderr, "interrupt error %d\n", errno);
	}

	/* Adding an interrupt failed !*/
	if( ret != 0){
		/* Give time for trace to complete */
		sleep(10);
		exit(ret);
	}
}

/**
 * Detach a callback to an gpio 'interrupt' asynchronous to sketch loop();
 */
void detachInterrupt(uint32_t pin)
{
	struct interrupt * pinterrupt = idesc.phead;
	int kick = 0, loop = 1;

	/* lock */
	pthread_mutex_lock(&idesc.mutex);

	/* check to see if this pin is already configured.. */
	while(pinterrupt != NULL && kick == 0){
		if(pinterrupt->pin == pin){

			/* remove entry */
			if (pinterrupt->pprev != NULL) {
				pinterrupt->pprev->pnext = pinterrupt->pnext;
				if (idesc.phead == pinterrupt)
					idesc.phead = pinterrupt->pprev;
			}
			if(pinterrupt->pnext != NULL) {
				pinterrupt->pnext->pprev = pinterrupt->pprev;
				if (idesc.phead == pinterrupt)
					idesc.phead = pinterrupt->pnext;
			}
			if (idesc.phead == pinterrupt)
				idesc.phead = NULL;

			(void)pinModeIRQ(pinterrupt->pin, (uint8_t)NONE);
			free(pinterrupt);

			pinterrupt = NULL;

			/* kick interrupt thread */
			kick = 1;

		} else {
			pinterrupt = pinterrupt->pnext;
		}
	}

	idesc.irq_count--;

	/* unlock */
	pthread_mutex_unlock(&idesc.mutex);

	/* async notify */
	if (kick == 1)
		if(write(idesc.pipe_tx_rx[__TTYUART_IDX_TX], (const char*)&loop, sizeof(loop))<0){
			fprintf(stderr, "interrupt error %d\n", errno);
		}
}

/**
 * Attach a callback to a timer 'interrupt' asynchronous to sketch loop();
 *
 * function : the function to call when the interrupt occurs
 * microseconds: defines when the interrupt should be triggered in the timing loop.
 */
void attachTimerInterrupt(void (*callback)(void), uint32_t microseconds)
{
	int ret = 0;
	int loop = 1;
	int fd;

	if (createTimerfd(&fd, microseconds) < 0) {
		trace_error("%s Cannot create timer for callback %p", __func__, callback);
		return;
	}

	pthread_mutex_lock(&idesc.mutex);

	ret = addIRQEntry(fd, -1, callback, 0);

	pthread_mutex_unlock(&idesc.mutex);

	/* async notify - not strictly necessary - remove ?*/
	if (write(idesc.pipe_tx_rx[__TTYUART_IDX_TX], (const char*) &loop,
	        sizeof(loop)) < 0) {
		trace_error("%s interrupt error %d\n", __func__, errno);
	}

	/* Adding an interrupt failed */
	if (ret != 0) {
		trace_error("%s addIRQEntry failed %d\n", __func__);
		/* Give time for trace to complete */
		sleep(10);
		exit(ret);
	}
}

int createTimerfd(int *fd, uint32_t period_us)
{
	struct itimerspec timer_interval;
	struct timespec timer_res;
	struct timespec now;
	struct timespec interval;
	long period_ns;

	if (period_us == 0) {
		trace_error("%s Cannot create zero microseconds timer", __func__);
		return -1;
	}

	*fd = timerfd_create(CLOCK_MONOTONIC, 0);
	if (*fd < 0) {
		trace_error("%s Cannot create timer file descriptor: %s", __func__,
				strerror(errno));
		return -1;
	}

	if (clock_getres(CLOCK_MONOTONIC, &timer_res) < 0) {
		trace_error("%s Cannot get timer resolution: %s", __func__,
				strerror(errno));
		goto do_error;
	}

	if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
		trace_error("%s Cannot get current time: %s", __func__,
				strerror(errno));
		goto do_error;
	}

	period_ns = period_us * 1000;
	if (period_ns % timer_res.tv_nsec != 0) {
		trace_error("%s Cannot add timer with period %d us - timer @ %d Hz",
		        __func__, period_us, timer_res.tv_nsec * NSECS_PER_SEC);
		goto do_error;
	}

	/* Set repeated expirations at the given interval */
	timer_interval.it_interval.tv_sec = period_ns / NSECS_PER_SEC;
	timer_interval.it_interval.tv_nsec = period_ns % NSECS_PER_SEC;

	/* Set the initial expiration to now + initial interval */
	timeradd(now, timer_interval.it_interval, timer_interval.it_value);

	if (timerfd_settime(*fd, TFD_TIMER_ABSTIME, &timer_interval, NULL) < 0) {
		trace_error("%s Can't set the timer: %s", __func__, strerror(errno));
		goto do_error;
	}

	trace_debug("%s Creating timer: freq: @%.2f Hz, timerfd: %d",
	        __func__, NSECS_PER_SEC / (double)period_ns, *fd);

	return 0;

do_error:
	close(*fd);
	return -1;
}

/**
 * interrupt_init
 *
 * Initialise the interrupt queue
 */
int interrupt_init( void )
{
	extern int errno;
	int ret = 0, fd;

	/* Setup pipe */
	ret = pipe2(idesc.pipe_tx_rx, O_NONBLOCK);
	if (ret < 0){
		fprintf(stderr, "error making pipe!\n");
		return errno;
	}

	idesc.irq_count = 0;
	idesc.phead = NULL;

	pthread_mutex_init(&idesc.mutex, 0);

	pthread_barrier_init(&idesc.barrier, 0, 2);

	/* initiate IRQ handler */
	pthread_create(&idesc.thread, NULL, interrupt_main, 0);

	/* sync startup */
	pthread_barrier_wait(&idesc.barrier);

	/* Good to go - start the client thread */
	return 0;
}

/**
 * interrupt_fini
 *
 * Shutdown interrupt services
 */
void interrupt_fini(void)
{
	int loop = 0;

	/* Send interrupt message */
	if(write(idesc.pipe_tx_rx[__TTYUART_IDX_TX], (const char*)&loop, sizeof(loop))<0){
		fprintf(stderr, "interrupt error %d\n", errno);
	}

	/* Join thread */
	pthread_join(idesc.thread, 0);

	if (idesc.pipe_tx_rx[__TTYUART_IDX_TX] != -1){
		close(idesc.pipe_tx_rx[__TTYUART_IDX_TX]);
		idesc.pipe_tx_rx[__TTYUART_IDX_TX] = -1;
	}

	if (idesc.pipe_tx_rx[__TTYUART_IDX_RX] != -1){
		close(idesc.pipe_tx_rx[__TTYUART_IDX_RX]);
		idesc.pipe_tx_rx[__TTYUART_IDX_RX] = -1;
	}
}
