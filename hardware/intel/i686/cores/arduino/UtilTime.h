#ifndef __TIME_H__
#define __TIME_H__


#ifdef __cplusplus
extern "C" {
#endif

#include <unistd.h>
#include <time.h>
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int us);
int timeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIME_H__ */
