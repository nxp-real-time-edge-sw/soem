/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/* 
 * Copyright 2022 NXP
*/

#ifndef _osal_defs_
#define _osal_defs_
#include <stdint.h>
#include "soem_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef EC_DEBUG
#define EC_PRINT printf
#else
#define EC_PRINT(...) do {} while (0)
#endif


#ifndef PACKED
#define PACKED_BEGIN
#define PACKED  __attribute__((__packed__))
#define PACKED_END
#endif
typedef char mtx_t;
#define osal_mutex_create() 0
#define osal_mutex_lock(mtx) (mtx) = 1
#define osal_mutex_unlock(mtx) (mtx) = 0

#define OSAL_THREAD_HANDLE  int *
#define OSAL_THREAD_FUNC void
#define OSAL_THREAD_FUNC_RT void

#ifdef __cplusplus
}
#endif

#endif
