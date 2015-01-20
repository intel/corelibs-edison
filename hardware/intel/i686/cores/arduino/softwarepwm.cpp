/*
 * Copyright (c) 2014 Intel Corporation
 * Dino Tinitigan <dino.tinitigan@intel.com>
 *
 * SoftwarePWM libaray for Intel Edison
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 *Bit-bangs pwm functionality
*/

#define MAX_DUTY 255

#include <pthread.h>
#include <Arduino.h>

struct pwm_thread_param
{
  unsigned int duty;
  unsigned int pin;
  unsigned int frequency;
  int threadID;
};

int threadControl[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
pthread_mutex_t pinMutex[14] = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, 
	PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, 
	PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, 
	PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, 
	PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER};

void *pwmHandler(void *ptr);
void softAnalogWrite(uint32_t pin, uint32_t duty, uint32_t freq);
void softAnalogWrite(uint32_t pin, uint32_t duty);
void pulse(int usec, int pin);

void softAnalogWrite(uint32_t pin, uint32_t duty, uint32_t freq)
{
  pthread_mutex_lock(&pinMutex[pin]);
  int tempThreadControl = ++threadControl[pin];
  pthread_mutex_unlock(&pinMutex[pin]);
  
  pthread_t pwmThread;
  
  struct pwm_thread_param *params;
  params=(pwm_thread_param *)malloc(sizeof(pwm_thread_param));
  params->pin = pin;
  params->duty = duty;
  params->frequency = freq;
  params->threadID = tempThreadControl;
  
  int iret1 = pthread_create( &pwmThread, NULL, pwmHandler, (void*) params);
  pthread_detach(pwmThread);
}

void softAnalogWrite(uint32_t pin, uint32_t duty)
{
  pthread_mutex_lock(&pinMutex[pin]);
  int tempThreadControl = ++threadControl[pin];
  pthread_mutex_unlock(&pinMutex[pin]);
  
  pthread_t pwmThread;
  
  struct pwm_thread_param *params;
  params=(pwm_thread_param *)malloc(sizeof(pwm_thread_param));
  params->pin = pin;
  params->duty = duty;
  params->frequency = 500;
  params->threadID = tempThreadControl;
  
  int iret1 = pthread_create( &pwmThread, NULL, pwmHandler, (void*) params);
  pthread_detach(pwmThread);
}

void *pwmHandler(void *arg)
{
  pwm_thread_param *param;
  param = (pwm_thread_param*)arg;
  
  int duty = param->duty;
  int pin = param->pin;
  int freq = param->frequency;
  int currentThreadControlID = param->threadID;
  
  if(freq > 0)
  {
    int period = (1.0/((double)(freq)))*1000000;
    while(true)
    { 
      if(currentThreadControlID != threadControl[pin])
      {
        break;
      }

      unsigned long a = micros();
      unsigned long b = a; 
      if((duty > 0) && (duty <MAX_DUTY))
      {
        double div = (period/((double)MAX_DUTY));
        int pulseL = ((double)duty * div) + .5;
        pulse(pulseL, pin);
      }
      else if(duty ==0)
      {
        fastDigitalWrite(pin, LOW); 
      }
      else if(duty ==MAX_DUTY)
      {
        fastDigitalWrite(pin, HIGH);
      }
      else
      {
        //invalid duty cycle value
      }
      while((b-a) < (period))
      {
        b = micros();
      }
    }
  }
}

void pulse(int usec, int pin)
{
  noInterrupts();
  unsigned long a = micros();
  unsigned long b = a;
  fastDigitalWrite(pin, HIGH);
  while((b-a) < (usec))
  {
    b = micros();
  }
  fastDigitalWrite(pin, LOW); 
  interrupts();
}