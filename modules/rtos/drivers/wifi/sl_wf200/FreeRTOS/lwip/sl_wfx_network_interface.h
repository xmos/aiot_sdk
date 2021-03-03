// Copyright 2021 XMOS LIMITED. This Software is subject to the terms of the
// XMOS Public License: Version 1

#ifndef SL_WFX_NETWORK_INTERFACE_H_
#define SL_WFX_NETWORK_INTERFACE_H_

#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/netifapi.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "lwip/init.h"
#include "lwip/debug.h"

err_t sta_wifi_if_init(struct netif *netif);
err_t ap_wifi_if_init(struct netif *netif);

void lwip_station_connected();
void lwip_station_disconnected();
void lwip_ap_connected();
void lwip_ap_disconnected();

#endif /* SL_WFX_NETWORK_INTERFACE_H_ */
