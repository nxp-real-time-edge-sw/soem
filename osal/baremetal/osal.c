/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/* 
 * Copyright 2022 NXP
*/

#include <osal.h>
#include <time.h>
#include <stdlib.h>
#include <sys/time.h>

#ifndef timercmp
#define  timercmp(a, b, CMP)                                \
  (((a)->tv_sec == (b)->tv_sec) ?                           \
   ((a)->tv_usec CMP (b)->tv_usec) :                        \
   ((a)->tv_sec CMP (b)->tv_sec))
#endif

#ifndef timeradd
#define  timeradd(a, b, result)                             \
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
#define  timersub(a, b, result)                             \
  do {                                                      \
    (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;           \
    (result)->tv_usec = (a)->tv_usec - (b)->tv_usec;        \
    if ((result)->tv_usec < 0) {                            \
      --(result)->tv_sec;                                   \
      (result)->tv_usec += 1000000;                         \
    }                                                       \
  } while (0)
#endif

#define USECS_PER_SEC 1000000
void osal_gettime(struct timeval *);

ec_timet osal_current_time (void)
{
   struct timeval current_time;
   ec_timet return_value;

   osal_gettime(&current_time);
   return_value.sec = current_time.tv_sec;
   return_value.usec = current_time.tv_usec;
   return return_value;
}

int osal_usleep (uint32 usec)
{
   struct timeval current_time;
   struct timeval target_time;
   struct timeval sleep_time;
   osal_gettime(&current_time);
   sleep_time.tv_sec = 0;
   sleep_time.tv_usec = usec;
   timeradd(&current_time, &sleep_time, &target_time);
   while(1) {
       osal_gettime(&current_time);
       if (timercmp(&current_time, &target_time, > ))
           break;
   }
   return 0;
}


void osal_timer_start (osal_timert * self, uint32 timeout_usec)
{
   struct timeval start_time;
   struct timeval timeout;
   struct timeval stop_time;

   osal_gettime(&start_time);
   timeout.tv_sec = timeout_usec / USECS_PER_SEC;
   timeout.tv_usec = timeout_usec % USECS_PER_SEC;
   timeradd (&start_time, &timeout, &stop_time);

   self->stop_time.sec = stop_time.tv_sec;
   self->stop_time.usec = stop_time.tv_usec;
}

boolean osal_timer_is_expired (osal_timert * self)
{
   struct timeval current_time;
   struct timeval stop_time;
   int is_not_yet_expired;

   osal_gettime(&current_time);
   stop_time.tv_sec = self->stop_time.sec;
   stop_time.tv_usec = self->stop_time.usec;
   is_not_yet_expired = timercmp (&current_time, &stop_time, <);

   return is_not_yet_expired == 0;
}

void *osal_malloc(uint32_t size)
{
   return malloc(size);
}

void osal_free(void *ptr)
{
   free(ptr);
}

void osal_thread_exit(void)
{
	return;
}

int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
   return 1;
}

int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param)
{
   return 1;
}
