/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/* 
 * Copyright 2022 NXP
*/


#include <osal.h>
#include <time.h>
#include <sys/time.h>
#include <assert.h>
#include "task.h"
#include "FreeRTOS.h"

#define RT_TASK_PRIORITY (configMAX_PRIORITIES - 1U)
#define NORMAL_TASK_PRIORITY (configMAX_PRIORITIES - 2U)
#define LOW_TASK_PRIORITY (1)

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

#define USECS_PER_SEC   1000000
#define USECS_PER_TICK  (USECS_PER_SEC / configTICK_RATE_HZ)

void udelay (uint32_t us)
{
   TickType_t  ticks = (us / USECS_PER_TICK) + 1;
   vTaskDelay(ticks);
}

int gettimeofday(struct timeval *tp, void *tzp)
{
   TickType_t tick = xTaskGetTickCount();
   TickType_t ticks_left;

   assert (tp != NULL);

   tp->tv_sec = tick / configTICK_RATE_HZ;

   ticks_left = tick % configTICK_RATE_HZ;
   tp->tv_usec = ticks_left * USECS_PER_TICK;
   assert (tp->tv_usec < USECS_PER_SEC);

   return 0;
}

int osal_usleep (uint32 usec)
{
   udelay(usec);
   return 0;
}

int osal_gettimeofday(struct timeval *tv, struct timezone *tz)
{
   return gettimeofday(tv, tz);
}

ec_timet osal_current_time (void)
{
   struct timeval current_time;
   ec_timet return_value;

   gettimeofday (&current_time, 0);
   return_value.sec = current_time.tv_sec;
   return_value.usec = current_time.tv_usec;
   return return_value;
}

void osal_timer_start (osal_timert * self, uint32 timeout_usec)
{
   struct timeval start_time;
   struct timeval timeout;
   struct timeval stop_time;

   gettimeofday (&start_time, 0);
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

   gettimeofday (&current_time, 0);
   stop_time.tv_sec = self->stop_time.sec;
   stop_time.tv_usec = self->stop_time.usec;
   is_not_yet_expired = timercmp (&current_time, &stop_time, <);

   return is_not_yet_expired == FALSE;
}

void osal_thread_exit()
{
	vTaskSuspend(NULL);
}

#if(configSUPPORT_DYNAMIC_ALLOCATION)
int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
   BaseType_t xReturned;
   xReturned = xTaskCreate(func, "worker", (configSTACK_DEPTH_TYPE)stacksize, param, NORMAL_TASK_PRIORITY, thandle);
   if(xReturned ==  pdPASS)
   {
      return 0;
   }
   return 1;
}
#else
StackType_t osalStackBuffer[MAX_SOEM_TASK_STACK * MAX_SOEM_TASK];
StaticTask_t osalTaskBuffer[MAX_SOEM_TASK];
int osalTaskIndex = 0;
int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
	configASSERT( stacksize <= MAX_SOEM_TASK_STACK);
	configASSERT( osalTaskIndex <= MAX_SOEM_TASK);
	thandle = (void*)xTaskCreateStatic(func, "worker", MAX_SOEM_TASK_STACK, param, NORMAL_TASK_PRIORITY,
				&osalStackBuffer[MAX_SOEM_TASK_STACK * osalTaskIndex], &osalTaskBuffer[osalTaskIndex]);
    osalTaskIndex++;
	return 1;
}

StaticSemaphore_t osalMutexBuffer[MAX_SOEM_MUTE];
int osalMuteIndex = 0;
SemaphoreHandle_t osalSemaphoreCreateMutexStatic()
{
	SemaphoreHandle_t xSemaphore = NULL;
	configASSERT( osalMuteIndex <= MAX_SOEM_MUTE);
	xSemaphore = xSemaphoreCreateMutexStatic(&osalMutexBuffer[osalMuteIndex]);
	configASSERT( xSemaphore );
	return xSemaphore;
}

#endif
