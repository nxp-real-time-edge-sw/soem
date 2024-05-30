/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/* 
 * Copyright 2022 NXP
*/

#ifndef _osal_defs_
#define _osal_defs_
#include "FreeRTOS.h"
#include "semphr.h"
#include "soem_config.h"

#ifdef EC_DEBUG
#define EC_PRINT PRINTF
#else
#define EC_PRINT(...) do {} while (0)
#endif

#ifndef PACKED
#define PACKED_BEGIN
#define PACKED  __attribute__((__packed__))
#define PACKED_END
#endif
void osal_thread_exit();

#if (MAX_SOEM_TASK == 1)
typedef char mtx_t;
#define osal_mutex_create() 0
#define osal_mutex_lock(mtx) mtx = 1
#define osal_mutex_unlock(mtx) (mtx) = 0
#else
typedef SemaphoreHandle_t mtx_t;
#if(configSUPPORT_DYNAMIC_ALLOCATION)
#define osal_mutex_create() xSemaphoreCreateMutex()
#else
SemaphoreHandle_t osalSemaphoreCreateMutexStatic();
#define osal_mutex_create() osalSemaphoreCreateMutexStatic()
#endif
#define osal_mutex_lock(mtx) xSemaphoreTake(mtx, portMAX_DELAY)
#define osal_mutex_unlock(mtx) xSemaphoreGive(mtx)
#endif

#define OSAL_THREAD_HANDLE TaskHandle_t
#define OSAL_THREAD_FUNC void
#define OSAL_THREAD_FUNC_RT void

#endif
