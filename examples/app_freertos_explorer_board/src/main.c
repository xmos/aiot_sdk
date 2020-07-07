// Copyright (c) 2020, XMOS Ltd, All rights reserved

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "queue.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_DHCP.h"

/* Library headers */

/* App headers */
#include "audio_pipeline.h"
#include "network.h"
#include "UDPCommandInterpreter.h"
#include "thruput_test.h"
#include "gpio_ctrl.h"
#include "app_conf.h"
#include "sntpd.h"


#if( configAPPLICATION_ALLOCATED_HEAP == 1 )
#if XCOREAI_EXPLORER
__attribute__((section(".ExtMem_data")))
#endif
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#endif


void soc_tile0_main(
        int tile)
{
    /* Create UDP CLI */
    vStartUDPCommandInterpreterTask( portTASK_STACK_DEPTH(vUDPCommandInterpreterTask), appconfCLI_UDP_PORT, appconfCLI_TASK_PRIORITY );

    /* Create the thruput test */
    thruput_test_create( appconfTHRUPUT_TEST_TASK_PRIORITY );

    /* Create the gpio control task */
    gpio_ctrl_create( appconfGPIO_TASK_PRIORITY );

    /* Initialize WiFi */
    initalize_wifi();

    vTaskStartScheduler();
}

void vApplicationDaemonTaskStartupHook( void )
{
    /* Create audio pipeline */
    audio_pipeline_create( appconfAUDIO_PIPELINE_TASK_PRIORITY );

    /* Create SNTPD */
    sntp_create( appconfSNTPD_TASK_PRIORITY );
}

void vApplicationMallocFailedHook(void)
{
    debug_printf("Malloc failed!\n");
    //configASSERT(0);
}
