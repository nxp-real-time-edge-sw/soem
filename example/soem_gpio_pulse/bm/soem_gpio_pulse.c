/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>
#include <stdio.h>

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_gpt.h"
#include "fsl_phyksz8081.h"
#include "fsl_debug_console.h"

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"
#include "enet/soem_enet.h"
#include "enet/enet.h"
#include "soem_port.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define NUM_1M      (1000000UL)
#define SOEM_PERIOD 125 /* 125 us */

#define OSEM_PORT_NAME "enet0"

#define ENET_RXBD_NUM (4)
#define ENET_TXBD_NUM (4)

#define ENET_RXBUFF_SIZE (ENET_FRAME_MAX_FRAMELEN)
#define ENET_TXBUFF_SIZE (ENET_FRAME_MAX_FRAMELEN)

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Buffer descriptors should be in non-cacheable region and should be align to "ENET_BUFF_ALIGNMENT". */
AT_NONCACHEABLE_SECTION_ALIGN(static enet_rx_bd_struct_t g_rxBuffDescrip[ENET_RXBD_NUM], ENET_BUFF_ALIGNMENT);
AT_NONCACHEABLE_SECTION_ALIGN(static enet_tx_bd_struct_t g_txBuffDescrip[ENET_TXBD_NUM], ENET_BUFF_ALIGNMENT);

/*! @brief The data buffers can be in cacheable region or in non-cacheable region.
 * If use cacheable region, the alignment size should be the maximum size of "CACHE LINE SIZE" and "ENET_BUFF_ALIGNMENT"
 * If use non-cache region, the alignment size is the "ENET_BUFF_ALIGNMENT".
 */
AT_NONCACHEABLE_SECTION_ALIGN(
    static uint8_t g_rxDataBuff[ENET_RXBD_NUM][SDK_SIZEALIGN(ENET_RXBUFF_SIZE, ENET_BUFF_ALIGNMENT)],
    ENET_BUFF_ALIGNMENT);
AT_NONCACHEABLE_SECTION_ALIGN(
    static uint8_t g_txDataBuff[ENET_TXBD_NUM][SDK_SIZEALIGN(ENET_TXBUFF_SIZE, ENET_BUFF_ALIGNMENT)],
    ENET_BUFF_ALIGNMENT);

static enet_buffer_config_t buffConfig[] = {{
    ENET_RXBD_NUM,
    ENET_TXBD_NUM,
    SDK_SIZEALIGN(ENET_RXBUFF_SIZE, ENET_BUFF_ALIGNMENT),
    SDK_SIZEALIGN(ENET_TXBUFF_SIZE, ENET_BUFF_ALIGNMENT),
    &g_rxBuffDescrip[0],
    &g_txBuffDescrip[0],
    &g_rxDataBuff[0][0],
    &g_txDataBuff[0][0],
    true,
    true,
    NULL,
}};

static char IOmap[100];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void irq_wake_task(void)
{
    return;
}

void control_task(char *ifname)
{
    int oloop, iloop;
    int expectedWKC;
    volatile int wkc;
    int old_switch0 = 0;
    int old_switch1 = 0;
    struct timeval current_time;
    struct timeval sleep_time;
    struct timeval target_time;
    struct timeval last_time;
    struct timeval diff_time;
    sleep_time.tv_sec  = 0;
    sleep_time.tv_usec = 125;

    /* initialise SOEM, and if_port */
    if (ec_init(ifname))
    {
        PRINTF("ec_init on %s succeeded.\r\n", ifname);
        if (ec_config_init(FALSE) > 0)
        {
            PRINTF("%d slaves found and configured.\r\n", ec_slavecount);
            ec_config_map(&IOmap);
            ec_configdc();
            PRINTF("Slaves mapped, state to SAFE_OP.\r\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
            oloop = ec_slave[2].Obytes;
            iloop = ec_slave[3].Ibytes;
            PRINTF("oloop = %d, iloop = %d\r\n", oloop, iloop);
            PRINTF("segments : %d : %d %d %d %d\r\n", ec_group[0].nsegments, ec_group[0].IOsegment[0],
                   ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            PRINTF("Request operational state for all slaves\r\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            PRINTF("Calculated workcounter %d\r\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            char chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            PRINTF("Operational state reached for all slaves.\r\n");
            /* cyclic loop */
            int is_expired;
            ec_send_processdata();
            osal_gettime(&last_time);
            while (1)
            {
                osal_gettime(&current_time);
                timeradd(&current_time, &sleep_time, &target_time);
                wkc = ec_receive_processdata(EC_TIMEOUTRET);
                if (wkc >= expectedWKC)
                {
                    if ((*(ec_slave[3].inputs) & 0x01) && old_switch0 == 0)
                    {
                        if (*(ec_slave[2].outputs) & 0x04)
                            *(ec_slave[2].outputs) &= ~0x04;
                        else
                            *(ec_slave[2].outputs) |= 0x04;
                    }

                    if ((*(ec_slave[3].inputs) & 0x02) && old_switch1 == 0)
                    {
                        if (*(ec_slave[2].outputs) & 0x04)
                            *(ec_slave[2].outputs) &= ~0x04;
                        else
                            *(ec_slave[2].outputs) |= 0x04;
                    }

                    old_switch0 = *(ec_slave[3].inputs) & 0x01;
                    old_switch1 = *(ec_slave[3].inputs) & 0x02;
                    if (*(ec_slave[2].outputs) & 0x02)
                        *(ec_slave[2].outputs) &= ~0x02;
                    else
                        *(ec_slave[2].outputs) |= 0x02;
                }
                osal_gettime(&current_time);
                timersub(&current_time, &last_time, &diff_time);
                is_expired = timercmp(&target_time, &current_time, <=);
                if (diff_time.tv_usec > 130)
                {
                    last_time.tv_usec = current_time.tv_usec;
                }
                last_time.tv_usec = current_time.tv_usec;
                last_time.tv_sec  = current_time.tv_sec;
                if (!is_expired)
                {
                    timersub(&target_time, &current_time, &sleep_time);
                    ec_send_processdata();
                    osal_usleep(sleep_time.tv_usec);
                    sleep_time.tv_usec = SOEM_PERIOD;
                }
                else
                {
                    while (1)
                        ;
                }
            }
        }
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    BOARD_InitHardware();

    PRINTF("Start the soem_gpio_pulse baremetal example...\r\n");

    osal_timer_init(NUM_1M, 0);
    if_port_init();
    control_task(OSEM_PORT_NAME);
    return 0;
}
