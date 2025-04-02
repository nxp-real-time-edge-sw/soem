/*
 * Copyright 2025 NXP
 *
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

#include <time.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <osal.h>

#define USECS_PER_SEC		1000000

#ifndef timercmp
#define timercmp(a, b, CMP)                                 \
  (((a)->tv_sec == (b)->tv_sec) ?                           \
   ((a)->tv_usec CMP (b)->tv_usec) :                        \
   ((a)->tv_sec CMP (b)->tv_sec))
#endif

#ifndef timeradd
#define timeradd(a, b, result)                              \
  do {                                                      \
    (result)->tv_sec = (a)->tv_sec + (b)->tv_sec;           \
    (result)->tv_usec = (a)->tv_usec + (b)->tv_usec;        \
    if ((result)->tv_usec >= 1000000)                       \
    {                                                       \
       ++(result)->tv_sec;                                  \
       (result)->tv_usec -= 1000000;                        \
    }                                                       \
  } while (0)
#endif

#ifndef timersub
#define timersub(a, b, result)                              \
  do {                                                      \
    (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;           \
    (result)->tv_usec = (a)->tv_usec - (b)->tv_usec;        \
    if ((result)->tv_usec < 0) {                            \
      --(result)->tv_sec;                                   \
      (result)->tv_usec += 1000000;                         \
    }                                                       \
  } while (0)
#endif

#define MAX_RT_THREADS_NUM	2
#define MAX_THREADS_NUM		2
#define STACK_SIZE		(8 * 1024)

static K_THREAD_STACK_ARRAY_DEFINE(rt_stack, MAX_RT_THREADS_NUM, STACK_SIZE);
static K_THREAD_STACK_ARRAY_DEFINE(stack, MAX_THREADS_NUM, STACK_SIZE);

void *osal_malloc(size_t size)
{
   return malloc(size);
}

void osal_free(void *ptr)
{
   free(ptr);
}

int osal_usleep(uint32 usec)
{
   struct timespec ts;

   ts.tv_sec = usec / USECS_PER_SEC;
   ts.tv_nsec = (usec % USECS_PER_SEC) * 1000;

   return nanosleep(&ts, NULL);
}

ec_timet osal_current_time(void)
{
   struct timespec current_time;
   ec_timet return_value;

   clock_gettime(CLOCK_REALTIME, &current_time);
   return_value.sec = current_time.tv_sec;
   return_value.usec = current_time.tv_nsec / 1000;

   return return_value;
}

void osal_time_diff(ec_timet *start, ec_timet *end, ec_timet *diff)
{
   if (end->usec < start->usec) {
      diff->sec = end->sec - start->sec - 1;
      diff->usec = end->usec + 1000000 - start->usec;
   } else {
      diff->sec = end->sec - start->sec;
      diff->usec = end->usec - start->usec;
   }
}

/* Returns time from some unspecified moment in past,
 * strictly increasing, used for time intervals measurement. */
static void osal_getrelativetime(struct timeval *tv)
{
   struct timespec ts;

   clock_gettime(CLOCK_MONOTONIC, &ts);
   tv->tv_sec = ts.tv_sec;
   tv->tv_usec = ts.tv_nsec / 1000;
}

void osal_timer_start(osal_timert *self, uint32 timeout_usec)
{
   struct timeval start_time;
   struct timeval timeout;
   struct timeval stop_time;

   osal_getrelativetime(&start_time);
   timeout.tv_sec = timeout_usec / USECS_PER_SEC;
   timeout.tv_usec = timeout_usec % USECS_PER_SEC;
   timeradd(&start_time, &timeout, &stop_time);

   self->stop_time.sec = stop_time.tv_sec;
   self->stop_time.usec = stop_time.tv_usec;
}

boolean osal_timer_is_expired(osal_timert *self)
{
   struct timeval current_time;
   struct timeval stop_time;
   int is_not_yet_expired;

   osal_getrelativetime(&current_time);
   stop_time.tv_sec = self->stop_time.sec;
   stop_time.tv_usec = self->stop_time.usec;
   is_not_yet_expired = timercmp(&current_time, &stop_time, <);

   return is_not_yet_expired == FALSE;
}

int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
   static unsigned int cnt = 0;
   pthread_attr_t attr;
   pthread_t *threadp;
   int ret;

   if (cnt > MAX_THREADS_NUM || stacksize > STACK_SIZE) {
      printf("%s: Invalid parameter!\n", __func__);
      return 0;
   }

   threadp = thandle;
   ret = pthread_attr_init(&attr);
   if (ret) {
      printf("%s: pthread_attr_init() failed (%d)!\n", __func__, ret);
      return 0;
   }

   ret = pthread_attr_setstack(&attr, &stack[cnt], STACK_SIZE);
   if (ret) {
      printf("%s: pthread_attr_setstack() failed (%d)!\n", __func__, ret);
      return 0;
   }

   ret = pthread_create(threadp, &attr, func, param);
   pthread_attr_destroy(&attr);
   if (ret) {
      printf("%s: pthread_create() failed (%d)!\n", __func__, ret);
      return 0;
   }

   cnt++;

   return 1;
}

int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param)
{
   static unsigned int cnt = 0;
   struct sched_param schparam;
   pthread_attr_t attr;
   pthread_t *threadp;
   int ret;

   if (cnt > MAX_RT_THREADS_NUM || stacksize > STACK_SIZE) {
      printf("%s: Invalid parameter!\n", __func__);
      return 0;
   }

   threadp = thandle;
   ret = pthread_attr_init(&attr);
   if (ret) {
      printf("%s: pthread_attr_init() failed (%d)!\n", __func__, ret);
      return 0;
   }

   ret = pthread_attr_setstack(&attr, &rt_stack[cnt], STACK_SIZE);
   if (ret) {
      printf("%s: pthread_attr_setstack() failed (%d)!\n", __func__, ret);
      return 0;
   }

   ret = pthread_create(threadp, &attr, func, param);
   pthread_attr_destroy(&attr);
   if (ret) {
      printf("%s: pthread_create() failed (%d)!\n", __func__, ret);
      return 0;
   }

   cnt++;

   memset(&schparam, 0, sizeof(schparam));

   schparam.sched_priority = sched_get_priority_max(SCHED_FIFO);
   ret = pthread_setschedparam(*threadp, SCHED_FIFO, &schparam);
   if (ret) {
      printf("%s: pthread_setschedparam() failed (%d)!\n", __func__, ret);
      return 0;
   }

   return 1;
}
