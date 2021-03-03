/**************************************************************************//**
 * Copyright 2018, Silicon Laboratories Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Device includes. */
#include "sl_wfx.h"
#include "sl_wfx_host.h"
#include "sl_wfx_iot_wifi.h"
#include "wifi.h"

/* LwIP includes. */
#include "lwip/timeouts.h"
#include "netif/etharp.h"

/***************************************************************************//**
 * Defines
******************************************************************************/
#define STATION_NETIF0 's'
#define STATION_NETIF1 't'
#define SOFTAP_NETIF0  'a'
#define SOFTAP_NETIF1  'p'

#define PRINT_MAC_ADDR( A ) rtos_printf("%x:%x:%x:%x:%x:%x\n", A[0], A[1], A[2], A[3], A[4], A[5])

/***************************************************************************//**
 * Variables
******************************************************************************/
// extern sl_wfx_context_t sl_wfx_context;
static struct netif sta_netif, ap_netif;
/***************************************************************************//**
 * @brief
 *    Initializes the hardware parameters. Called from ethernetif_init().
 *
 * @param[in] netif: the already initialized lwip network interface structure
 *
 * @return
 *    None
 ******************************************************************************/
static void low_level_init(struct netif *netif)
{
    if (WIFI_IsConnected() == pdFAIL) {
        EventBits_t bits;

        /* WiFi is not yet connected to an AP */

        while( sl_wfx_context == NULL || sl_wfx_event_group == NULL)
        {
            /* sl_wfx_init() has not yet set the context or created the
            event group. Wait a bit and check again. */
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        /* The WFX200 has not yet completed initialization.
        Wait for it to do so. */
        do {
            bits = xEventGroupWaitBits(sl_wfx_event_group,
                                       SL_WFX_INITIALIZED,
                                       pdFALSE, /* Do not clear this bit */
                                       pdTRUE,
                                       portMAX_DELAY);
        } while( ( bits & SL_WFX_INITIALIZED ) == 0 );

        /* Wait until we get the lock */
        while( WIFI_GetLock() != eWiFiSuccess ) {
            vTaskDelay(pdMS_TO_TICKS(250));
        }

        if( ( sl_wfx_context->state & SL_WFX_STARTED ) == 0 )
        {
            /* This probably shouldn't happen */
            xassert(0);
        }

        if( WIFI_IsConnected() == pdFALSE )
        {
            /* set netif MAC hardware address length */
            netif->hwaddr_len = ETH_HWADDR_LEN;

            /* Check which netif is initialized and set netif MAC hardware address */
            if (netif->name[0] == SOFTAP_NETIF0)
            {
            netif->hwaddr[0] =  sl_wfx_context->mac_addr_1.octet[0];
            netif->hwaddr[1] =  sl_wfx_context->mac_addr_1.octet[1];
            netif->hwaddr[2] =  sl_wfx_context->mac_addr_1.octet[2];
            netif->hwaddr[3] =  sl_wfx_context->mac_addr_1.octet[3];
            netif->hwaddr[4] =  sl_wfx_context->mac_addr_1.octet[4];
            netif->hwaddr[5] =  sl_wfx_context->mac_addr_1.octet[5];
            }
            else
            {
            netif->hwaddr[0] =  sl_wfx_context->mac_addr_0.octet[0];
            netif->hwaddr[1] =  sl_wfx_context->mac_addr_0.octet[1];
            netif->hwaddr[2] =  sl_wfx_context->mac_addr_0.octet[2];
            netif->hwaddr[3] =  sl_wfx_context->mac_addr_0.octet[3];
            netif->hwaddr[4] =  sl_wfx_context->mac_addr_0.octet[4];
            netif->hwaddr[5] =  sl_wfx_context->mac_addr_0.octet[5];
            }

            /* Set netif maximum transfer unit */
            netif->mtu = 1500;

            /* Accept broadcast address and ARP traffic */
            netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

            // /* Set netif link flag */
            // netif->flags |= NETIF_FLAG_LINK_UP;

            WIFI_ReleaseLock();

            do {
                bits = xEventGroupWaitBits(sl_wfx_event_group,
                                           SL_WFX_CONNECT | SL_WFX_START_AP,
                                           pdFALSE, /* Do not clear these bits */
                                           pdFALSE,
                                           portMAX_DELAY);
            } while( ( bits & ( SL_WFX_CONNECT | SL_WFX_START_AP ) ) == 0 );
        }
        else {
            /* The WIFI managed to connect to an AP before we got a chance to
            check to see if the MAC address should get changed. */
            WIFI_ReleaseLock();
        }
    }
}

/***************************************************************************//**
 * @brief
 *    This function should does the actual transmission of the packet(s).
 *    The packet is contained in the pbuf that is passed to the function.
 *    This pbuf might be chained.
 *
 * @param[in] netif: the lwip network interface structure
 *
 * @param[in] p: the packet to send
 *
 * @return
 *    ERR_OK if successful
 ******************************************************************************/
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  err_t errval = ERR_MEM;
  struct pbuf *q;
  sl_wfx_send_frame_req_t* tx_buffer;
  sl_status_t result;
  uint8_t *buffer ;
  uint32_t frame_length;

  if( WIFI_GetLock() == eWiFiSuccess )
  {
      if( WIFI_IsConnected() != pdFALSE )
      {
          /* Compute packet frame length */
          frame_length = SL_WFX_ROUND_UP(p->tot_len, 2);

          sl_wfx_allocate_command_buffer((sl_wfx_generic_message_t**)(&tx_buffer),
                                         SL_WFX_SEND_FRAME_REQ_ID,
                                         SL_WFX_TX_FRAME_BUFFER,
                                         frame_length + sizeof(sl_wfx_send_frame_req_t));
          buffer = tx_buffer->body.packet_data;

          for(q = p; q != NULL; q = q->next)
          {
            /* Copy the bytes */
            memcpy(buffer, q->payload, q->len);
            buffer += q->len;
          }

          /* Transmit to the station or softap interface */
          if (netif->name[0] == SOFTAP_NETIF0)
          {
            result = sl_wfx_send_ethernet_frame(tx_buffer,
                                                frame_length,
                                                SL_WFX_SOFTAP_INTERFACE,
                                                0);
          }
          else
          {
            result = sl_wfx_send_ethernet_frame(tx_buffer,
                                                frame_length,
                                                SL_WFX_STA_INTERFACE,
                                                0);
          }
          sl_wfx_free_command_buffer((sl_wfx_generic_message_t*) tx_buffer,
                                     SL_WFX_SEND_FRAME_REQ_ID,
                                     SL_WFX_TX_FRAME_BUFFER);

          if(result == SL_STATUS_OK)
          {
            errval = ERR_OK;
          }
      }
      WIFI_ReleaseLock();
  }

  return errval;
}

/***************************************************************************//**
 * @brief
 *    This function transfers the receive packets from the wf200 to lwip.
 *
 * @param[in] netif: the lwip network interface structure
 *
 * @param[in] rx_buffer: the ethernet frame received by the wf200
 *
 * @return
 *    LwIP pbuf filled with received packet, or NULL on error
 ******************************************************************************/
static struct pbuf *low_level_input(struct netif *netif, sl_wfx_received_ind_t* rx_buffer)
{
  struct pbuf *p = NULL, *q;
  uint8_t *buffer;

  /* Obtain the packet by removing the padding. */
  buffer = (uint8_t *)&(rx_buffer->body.frame[rx_buffer->body.frame_padding]);

  if (rx_buffer->body.frame_length > 0)
  {
    /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
    p = pbuf_alloc(PBUF_RAW, rx_buffer->body.frame_length, PBUF_POOL);
  }

  if (p != NULL)
  {
    for(q = p; q != NULL; q = q->next)
    {
      /* Copy remaining data in pbuf */
      memcpy(q->payload, buffer, q->len);
      buffer += q->len;
    }
  }

  return p;
}

/***************************************************************************//**
 * @brief
 *    This function implements the wf200 received frame callback.
 *
 * @param[in] rx_buffer: the ethernet frame received by the wf200
 *
 * @return
 *    None
******************************************************************************/
#pragma stackfunction 2000
void sl_wfx_host_received_frame_callback(sl_wfx_received_ind_t* rx_buffer)
{
  struct pbuf *p;
  struct netif *netif;

  /* Check packet interface to send to AP or STA interface */
  if((rx_buffer->header.info & SL_WFX_MSG_INFO_INTERFACE_MASK) ==
     (SL_WFX_STA_INTERFACE << SL_WFX_MSG_INFO_INTERFACE_OFFSET))
  {
    /* Send to station interface */
    netif = &sta_netif;
  }else{
    /* Send to softAP interface */
    netif = &ap_netif;
  }

  if (netif != NULL)
  {
    p = low_level_input(netif, rx_buffer);
    if (p != NULL)
    {
        rtos_printf("call input\n");
      if (netif->input(p, netif) != ERR_OK)
      {
        pbuf_free(p);
      }
    }
  }
}

/***************************************************************************//**
 * @brief
 *    called at the beginning of the program to set up the network interface.
 *
 * @param[in] netif: the lwip network interface structure
 *
 * @return
 *    ERR_OK if successful
 ******************************************************************************/
err_t sta_wifi_if_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

  /* Set the netif name to identify the interface */
  netif->name[0] = STATION_NETIF0;
  netif->name[1] = STATION_NETIF1;

  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);
  sta_netif = *netif;

  return ERR_OK;
}

/***************************************************************************//**
 * @brief
 *    called at the beginning of the program to set up the network interface.
 *
 * @param[in] netif: the lwip network interface structure
 *
 * @return
 *    ERR_OK if successful
 ******************************************************************************/
err_t ap_wifi_if_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

  /* Set the netif name to identify the interface */
  netif->name[0] = SOFTAP_NETIF0;
  netif->name[1] = SOFTAP_NETIF1;

  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);
  ap_netif = *netif;

  return ERR_OK;
}
