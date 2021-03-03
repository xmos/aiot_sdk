// Copyright 2021 XMOS LIMITED. This Software is subject to the terms of the
// XMOS Public License: Version 1

#include <stdlib.h>
#include <string.h>

#include "sl_wfx.h"
#include "FreeRTOS/sl_wfx_host.h"

#include "FreeRTOS_IP.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"

sl_status_t sl_wfx_host_allocate_buffer(void **buffer,
                                        sl_wfx_buffer_type_t type,
                                        uint32_t buffer_size)
{
/*
 * Zero copy RX is no longer supported. This code is being left in place
 * for the future if support for these is ever added back in.
 */
#if 0
    if (ipconfigZERO_COPY_RX_DRIVER != 0 && type == SL_WFX_RX_FRAME_BUFFER) {
        NetworkBufferDescriptor_t *network_buffer;
        network_buffer = pxGetNetworkBufferWithDescriptor( buffer_size, 0 );
        if (network_buffer != NULL) {
            /*
             * The pointer that will be returned to the driver is behind the beginning
             * of the Ethernet Frame by the size of the received indication message and
             * SL_WFX_NORMAL_FRAME_PAD_LENGTH padding bytes. This way frames with 2 pad
             * bytes will begin at where pucEthernetBuffer points to.
             *
             * When the padding is other than SL_WFX_NORMAL_FRAME_PAD_LENGTH bytes then
             * this needs to be dealt with in the receive callback.
             *
             * This requires that the FreeRTOS+TCP option ipBUFFER_PADDING be large enough
             * to hold the sl_wfx_received_ind_t message.
             */
            *buffer = network_buffer->pucEthernetBuffer - sizeof(sl_wfx_received_ind_t) - SL_WFX_NORMAL_FRAME_PAD_LENGTH;
        } else {
            *buffer = NULL;
        }
    } else {
        *buffer = pvPortMalloc(buffer_size);
    }
#else
    *buffer = pvPortMalloc(buffer_size);
#endif

    if (buffer != NULL) {
        return SL_STATUS_OK;
    } else {
        return SL_STATUS_NO_MORE_RESOURCE;
    }
}

sl_status_t sl_wfx_host_free_buffer(void *buffer, sl_wfx_buffer_type_t type)
{
//    if (ipconfigZERO_COPY_RX_DRIVER == 0 || type != SL_WFX_RX_FRAME_BUFFER) {
        vPortFree(buffer);
//    }

    return SL_STATUS_OK;
}
