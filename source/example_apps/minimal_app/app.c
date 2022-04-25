/* Copyright 2017 Wirepas Ltd. All Rights Reserved.
 *
 * See file LICENSE.txt for full license details.
 *
 */

/*
 * \file    app.c
 * \brief   This file is a minimal app to start the stack
 */

#include <stdlib.h>

#include "api.h"
#include "node_configuration.h"
#include "spi.h"
#include "app_scheduler.h"
#include "nrf_gpio.h"
#include "board.h"

/** Period to send PERIODIC message, in us. */
//static uint32_t m_period_ms;


uint8_t spi_tx_buff[256];
uint8_t spi_rx_buff[256];
uint8_t tx_buff_size;
uint8_t rx_buff_size;
uint16_t var;


spi_xfer_t spi_dt_buffer;
spi_conf_t spi_0;

/** Endpoint used to communicate. */
#define DATA_EP        (1u)

/** Application periodic message sending period default value in seconds. */
#define DEFAULT_PERIOD_S    (10u)
/**
 * Application periodic message sending period default value in milliseconds.
 */
#define DEFAULT_PERIOD_MS   (DEFAULT_PERIOD_S*1000u)

/**
 * \brief   Value to return from task or as initial time to be executed ASAP
 */
#define APP_SCHEDULER_SCHEDULE_ASAP (0)


/** Time needed to execute the periodic work, in us. */
#define PERIODIC_WORK_EXECUTION_TIME_US (250u)



//chip select manual control function

void sensor_select_chip(int select)
{
    if (select)
    {
        nrf_gpio_pin_clear(BOARD_SPI_CS_PIN);
    }
    else
    {
        nrf_gpio_pin_set(BOARD_SPI_CS_PIN);
    }
}

// This function triggers the spi transfer

void spi_transmission(uint8_t * tx_buff, uint8_t tx_size,uint8_t * rx_buff,uint8_t rx_size)
{
spi_dt_buffer.write_ptr  = tx_buff;
spi_dt_buffer.write_size = tx_size;
spi_dt_buffer.read_ptr   = rx_buff;
spi_dt_buffer.read_size  = rx_size;
SPI_init(&spi_0);
//sensor_select_chip(1);
nrf_gpio_pin_clear(BOARD_SPI_CS_PIN);
SPI_transfer(&spi_dt_buffer,NULL);
//sensor_select_chip(0);
nrf_gpio_pin_set(BOARD_SPI_CS_PIN);
SPI_close();

}
// selecting thermocouple type as k-type
void set_thermocoupletype()
{
uint8_t var = 0;
spi_tx_buff[0] = 0x01;
spi_rx_buff[0] = 0x00;    
tx_buff_size   = 0x02;
rx_buff_size   = 0x01;
spi_transmission(spi_tx_buff,tx_buff_size,spi_rx_buff,rx_buff_size);

var  =  spi_rx_buff[0] & 0xF0; 
var |=  0x03;

spi_tx_buff[0] = 0x81;
spi_tx_buff[1] = var;    
tx_buff_size   = 0x02;
rx_buff_size   = 0x00;
spi_transmission(spi_tx_buff,tx_buff_size,spi_rx_buff,rx_buff_size);

}

void measure_temp()
{
     var = 0;
     spi_tx_buff[0] = 0x0A;
     spi_tx_buff[1] = 0x00;    
    spi_dt_buffer.write_ptr=spi_tx_buff;
	spi_dt_buffer.write_size=sizeof(spi_tx_buff);
	spi_dt_buffer.read_ptr=spi_rx_buff;
	spi_dt_buffer.read_size=3U;
		
		SPI_init(&spi_0);
		nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
		SPI_transfer(&spi_dt_buffer, NULL);
		nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.
		SPI_close();
      var = spi_rx_buff[1];
      var <<= 8;
      var |= spi_rx_buff[2];
      var /= 256;
}
/**
 * @brief   Task to send periodic message.
 *
 *          This task will be executed every m_period_ms milliseconds by
 *          the app_scheduler. You can do anything you want for
 *          PERIODIC_WORK_EXECUTION_TIME_US.
 *          In this example, a monotonically increasing 32-bit value followed
 *          by the data pattern "0x0A 0x0B 0x0B 0x0A 0x0A 0x0C 0x0D 0x0C"
 *          is sent to the sink.
 *
 * @return  next task execution time moment.
 */
#if 0
static uint32_t task_send_periodic_msg(void)
{
   // static uint32_t counter_value = 0;

#if 0
    payload_periodic_t payload; /* Message payload data. */

    payload.counter_value = counter_value;
    memcpy(payload.data_pattern,
           m_periodic_data_pattern,
           sizeof(m_periodic_data_pattern));

    /* Send message. */
    if (send_uplink_msg(MSG_ID_PERIODIC_MSG,
                        (uint8_t *)&payload) != APP_LIB_DATA_SEND_RES_SUCCESS)
    {
        /*s
         * Message was not accepted for sending.
         * Error handling can be performed here.
         */
    }

    /* Increment value to send. */
    counter_value++;
#endif 
    measure_temp();

    app_lib_data_to_send_t data_to_send;
    data_to_send.bytes =  (const uint8_t *)&var;
    data_to_send.num_bytes = sizeof(var);
    data_to_send.dest_address = APP_ADDR_ANYSINK;
    data_to_send.src_endpoint = DATA_EP;
    data_to_send.dest_endpoint = DATA_EP;
    data_to_send.qos = APP_LIB_DATA_QOS_HIGH;
    data_to_send.delay = 0;
    data_to_send.flags = APP_LIB_DATA_SEND_FLAG_NONE;
    data_to_send.tracking_id = APP_LIB_DATA_NO_TRACKING_ID;

    // Send the data packet
    lib_data->sendData(&data_to_send);
    /*
     * Inform the stack that this function should be called again in
     * m_period_ms milliseconds. By returning APP_SCHEDULER_STOP_TASK,
     * the scheduler will remove the task.
     */
    return m_period_ms;
}
#endif
// initializing sensor registers
void sensor_init()
{
// setting fault register bits 
spi_tx_buff[0] = 0x82;
spi_tx_buff[1] = 0x00;    
tx_buff_size   = 0x02;
rx_buff_size   = 0x00;

spi_transmission(spi_tx_buff,tx_buff_size,spi_rx_buff,rx_buff_size);
//setting config reg 0
spi_tx_buff[0] = 0x80;
spi_tx_buff[1] = 0x50;    
tx_buff_size   = 0x02;
rx_buff_size   = 0x00;
spi_transmission(spi_tx_buff,tx_buff_size,spi_rx_buff,rx_buff_size);
//setting cold junction temperature offset value here
spi_tx_buff[0] = 0x89;
spi_tx_buff[1] = 0x00;    
tx_buff_size   = 0x02;
rx_buff_size   = 0x00;
spi_transmission(spi_tx_buff,tx_buff_size,spi_rx_buff,rx_buff_size);
}

// initializing SPI parameters clock freq, mode and bit order 
void spi_setup()
{
spi_conf_t spi_0;
spi_0.clock = 1000000;
spi_0.mode = SPI_MODE_HIGH_SECOND;
spi_0.bit_order = SPI_ORDER_MSB;
SPI_init(&spi_0);
}

#if 1

void App_init()
{
	spi_conf_t spi_0;
	spi_0.clock = 1000000;
	spi_0.mode = SPI_MODE_LOW_SECOND; // CPOL=1, CPHA=1 (You might need to test the SPI_MODE_LOW_FIRST if it does not work)
	spi_0.bit_order = SPI_ORDER_MSB;
	nrf_gpio_pin_dir_set(BOARD_SPI_CS_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_cfg_output(BOARD_SPI_CS_PIN);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN);
   spi_xfer_t spi_dt_buffer;

	uint8_t spi_tx_buff[2]; /// = 0x04; // Read CJLF register (i.e Cold-Junction Low Fault Threshold)
	uint8_t spi_rx_buff[2];		// Store CJLF register value. Should be 0xC0 by default
	
	// Init transfert.
	nrf_gpio_pin_dir_set(17, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_cfg_output(17);
    nrf_gpio_pin_dir_set(18, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_cfg_output(18);

    spi_tx_buff[0] = 0x82;
    spi_tx_buff[1] = 0x00; 
    
	spi_dt_buffer.write_ptr=spi_tx_buff;
	spi_dt_buffer.write_size=sizeof(spi_tx_buff);
	spi_dt_buffer.read_ptr=spi_rx_buff;
	spi_dt_buffer.read_size=1U;
		
		SPI_init(&spi_0);
		nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
		SPI_transfer(&spi_dt_buffer, NULL);
		nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.
		SPI_close();

   spi_tx_buff[0] = 0x80;
   spi_tx_buff[1] = 0x50;
    
	spi_dt_buffer.write_ptr=spi_tx_buff;
	spi_dt_buffer.write_size=sizeof(spi_tx_buff);
	spi_dt_buffer.read_ptr=spi_rx_buff;
	spi_dt_buffer.read_size=1U;
		
		SPI_init(&spi_0);
		nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
		SPI_transfer(&spi_dt_buffer, NULL);
		nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.
		SPI_close();

    spi_tx_buff[0] = 0x89;
    spi_tx_buff[1] = 0x00;    
    
	spi_dt_buffer.write_ptr=spi_tx_buff;
	spi_dt_buffer.write_size=sizeof(spi_tx_buff);
	spi_dt_buffer.read_ptr=spi_rx_buff;
	spi_dt_buffer.read_size=1U;
		
		SPI_init(&spi_0);
		nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
		SPI_transfer(&spi_dt_buffer, NULL);
		nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.
		SPI_close();
 	while(1)
	{
     
    
#if 1
     spi_tx_buff[0] = 0x0C;
     spi_tx_buff[1] = 0x00;  
    
	spi_dt_buffer.write_ptr=spi_tx_buff;
	spi_dt_buffer.write_size=sizeof(spi_tx_buff);
	spi_dt_buffer.read_ptr=spi_rx_buff;
	spi_dt_buffer.read_size=4U;
		
		SPI_init(&spi_0);
		nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
		SPI_transfer(&spi_dt_buffer, NULL);
		nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.
		SPI_close();
#endif		
		// at this point spi_rx_buff should be  equal to 0xC0.
		if (spi_rx_buff[0] == 0xA0)
		{
				// Do something (e.g switch on a LED)
                nrf_gpio_pin_set(18);
                nrf_gpio_pin_clear(17);
		}
		else
		{
			// Do something (e.g switch on another LED or switch the one from above OFF
            nrf_gpio_pin_set(17);
            nrf_gpio_pin_clear(18);
            
		}
	}
}
#endif

#if 0
void App_init()
{
	spi_conf_t spi_0;
	spi_0.clock = 1000000;
	spi_0.mode = SPI_MODE_LOW_SECOND; // CPOL=1, CPHA=1 (You might need to test the SPI_MODE_LOW_FIRST if it does not work)
	spi_0.bit_order = SPI_ORDER_MSB;
	
	uint8_t spi_tx_buff = 0x04; // Read CJLF register (i.e Cold-Junction Low Fault Threshold)
	uint8_t spi_rx_buff[2];		// Store CJLF register value. Should be 0xC0 by default
	
	// Init transfert.
	spi_xfer_t spi_dt_buffer;
	spi_dt_buffer.write_ptr=&spi_tx_buff;
	spi_dt_buffer.write_size=sizeof(spi_tx_buff);
	spi_dt_buffer.read_ptr=spi_rx_buff;
	spi_dt_buffer.read_size=1U;

	nrf_gpio_pin_dir_set(BOARD_SPI_CS_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_cfg_output(BOARD_SPI_CS_PIN);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN);
	nrf_gpio_pin_dir_set(17, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_cfg_output(17);
    nrf_gpio_pin_dir_set(18, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_cfg_output(18);
 	while(1)
	{
		
		SPI_init(&spi_0);
		nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
		SPI_transfer(&spi_dt_buffer, NULL);
		nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.
		SPI_close();
		
		// at this point spi_rx_buff should be  equal to 0xC0.
		if (spi_rx_buff[0] == 0xC0)
		{
				// Do something (e.g switch on a LED)
                nrf_gpio_pin_set(18);
                nrf_gpio_pin_clear(17);
		}
		else
		{
			// Do something (e.g switch on another LED or switch the one from above OFF
            nrf_gpio_pin_set(17);
            nrf_gpio_pin_clear(18);
            
		}
	}
}
#endif

/**
 * \brief   Initialization callback for application
 *
 * This function is called after hardware has been initialized but the
 * stack is not yet running.
 *
 */
#if 0
void App_init(const app_global_functions_t * functions)
{
    app_lib_settings_role_t role = app_lib_settings_create_role(APP_LIB_SETTINGS_ROLE_HEADNODE, APP_LIB_SETTINGS_ROLE_FLAG_LL);
    
    nrf_gpio_pin_dir_set(BOARD_SPI_CS_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_cfg_output(BOARD_SPI_CS_PIN);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN);
    // Basic configuration of the node with a unique node address
    if (configureNodeFromBuildParameters() != APP_RES_OK)
    {
        // Could not configure the node
        // It should not happen except if one of the config value is invalid
        return;
    }

    lib_settings->setNodeRole(role);


    /*spi init*/
    spi_setup(); 
    
    /*Temperature sensor initialization*/
    sensor_init();

    set_thermocoupletype();
     /* Init application scheduler. */
    App_Scheduler_init();

    /* Set a periodic task to be called after DEFAULT_PERIOD_MS. */
    m_period_ms = DEFAULT_PERIOD_MS;
    App_Scheduler_addTask_execTime(task_send_periodic_msg,
                                   APP_SCHEDULER_SCHEDULE_ASAP,
                                   PERIODIC_WORK_EXECUTION_TIME_US);

    /*
     * Start the stack.
     * This is really important step, otherwise the stack will stay stopped and
     * will not be part of any network. So the device will not be reachable
     * without reflashing it
     */
    
    lib_state->startStack();
}
#endif
