// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#ifndef __SOEM_NETC_H___
#define __SOEM_NETC_H___
#include "fsl_phy.h"
#include "fsl_netc_endpoint.h"
#include "fsl_netc_mdio.h"
#include <stdint.h>

struct netc_ep_if_port
{
	ep_handle_t g_ep_handle;
	netc_mdio_handle_t s_emdio_handle;
    phy_handle_t phy_handle;
	ep_config_t *ep_config;
	netc_bdr_config_t *bdrConfig;
	uint8_t *g_txFrame; 
	bool txOver;
};
#endif
