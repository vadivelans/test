/* Copyright 2020 Wirepas Ltd. All Rights Reserved.
 *
 * See file LICENSE.txt for full license details.
 *
 */

/*
 * @file    app.c
 * @brief   This file is a helloworld app enabling wirepas mesh network
 *          capabilities experiments
 *          (see README for additional information)
 */

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "shared_appconfig.h"
#include "api.h"
#include "node_configuration.h"
#include "led.h"
#include "button.h"
#include "shared_data.h"
#include "app_scheduler.h"
#include "nrf_gpio.h"
#include "board.h"
#include "spi.h"

/** Buffer for used for copy the TC value */
static uint32_t TC_temperature_value;    

/** Buffer for used for copy the CJ value */
static uint32_t CJ_temperature_value;

/** Buffer for used for functional the TC/CJ value */
float f_cold_junction_temperature, f_linearized_tc_temperature;
 
/** CJ resolution value */
#define Cold_Junction_Resolution 0.015625

/** TC resolution value */
#define TC_Resolution 0.0078125

/**  To stop conversion value  */
#define Stop_Conversion_Bit (uint8_t)0x3F

/** use to set one shot conversion mode */
#define One_Shot_Conversion (uint8_t)0x40

/** used for automatic conversion mode value */
#define Automatic_Conversion (uint8_t)0x80

#define DEBUG_LOG_MODULE_NAME "EVAL_APP"
/** To activate logs, configure the following line with "LVL_INFO". */
#define DEBUG_LOG_MAX_LEVEL LVL_NOLOG

#include "debug_log.h"

/** Convert message travel delay from 1/128th second to milliseconds. */
#define COARSE_TO_MS(delay) (1000u * (delay) >> 7)

/** Application periodic message fixed data pattern byte length. */
#define PERIODIC_MSG_DATA_PATTERN_LEN (8u)

/** Application periodic message sending period default value in seconds. */
#define DEFAULT_PERIOD_S (10u)
/**
 * Application periodic message sending period default value in milliseconds.
 */
#define DEFAULT_PERIOD_MS (DEFAULT_PERIOD_S * 1000u)

/** The application will register for this type of app_config, corresponding to the measurement rate. */
#define CUSTOM_PERIOD_TYPE 0xC3

/** Time needed to execute the periodic work, in us. */
#define PERIODIC_WORK_EXECUTION_TIME_US (250u)

/** Button pressed event ID. */
#define BUTTON_PRESSED_STATE (2u)
/** Time needed to execute the send "button pressed message" task, in us. */
#define TASK_EXEC_TIME_US_SEND_BUTTON_PRESSED_MSG (250u)

/**
 *  In this example the periodic data transfer interval is changed according
 *  to received configuration data or via regular downlink message.
 *  Configuration data structure is application dependent, in this example it
 *  is structured as followed:
 *      A period and eventually a custom data, to be filled with what you need.
 * */
typedef struct __attribute__((packed))
{
    uint32_t interval; /** Measurment rate in seconds. */
    /* uint16_t custom_data; */
} app_config_t;

/**
 * Application "periodic message period setup message" minimal allowed
 * period value in milliseconds (2 seconds).
 */
#define PERIODIC_MSG_PERIOD_SET_MIN_VAL_MS (2000ul)
/**
 * Application "periodic message period setup message" maximal allowed
 * period value in milliseconds (20 minutes).
 */
#define PERIODIC_MSG_PERIOD_SET_MAX_VAL_MS (1200000ul)

/** Maximal supported number of LEDs by this app. */
#define MAX_LED_PER_BOARD (4u)

/** Endpoint used to communicate. */
#define DATA_EP (1u)

/** thermocouple status register. */
#define ADDRESS_SR_READ 0x0F

/** spi struct variable */
spi_xfer_t spi_dt_buffer;                
spi_xfer_t spi_ft_buffer;
spi_conf_t spi_0;

/** Read CJLF register (i.e Cold-Junction Low Fault Threshold) */
uint8_t spi_tx_buff[20];   

/** Store CJLF register value. Should be 0xC0 by default  */
uint8_t spi_rx_buff[20] = {0};      

/** buffer to store the read sensor data */
uint8_t send_bytes[20];   

/** variable used for config single shot mode */             
uint8_t uch_cr0;              

/** Filter to process shared_appconfig updates. */
static uint16_t m_filter_id;

/**
 * @brief Message ID.
 * (range splitted in two (0-127 => UPLINK packet /
 * 128-255 => DOWNLINK packet)).
 */
typedef enum
{
    /** Evaluation app "Periodic message" message ID. */
    MSG_ID_PERIODIC_MSG = 0,
    /** Evaluation app "Button pressed event" message ID. */
    MSG_ID_BUTTON_EVENT_MSG = 1,
    /** Evaluation app "Echo command response" message ID. */
    MSG_ID_ECHO_RESPONSE_MSG = 2,
    /** Evaluation app "LED get state response" message ID. */
    MSG_ID_LED_GET_STATE_RESPONSE_MSG = 3,
    /** Evaluation app "Periodic message period set" message ID. */
    MSG_ID_PERIODIC_MSG_PERIOD_SET_MSG = 128,
    /** Evaluation app "LED set state" message ID. */
    MSG_ID_LED_SET_STATE_MSG = 129,
    /** Evaluation app "LED get state" message ID. */
    MSG_ID_LED_GET_STATE_MSG = 130,
    /** Evaluation app "Echo command" message ID. */
    MSG_ID_ECHO_COMMAND_MSG = 131,
} message_id_e;

/** @brief LED state. */
typedef enum
{
    /** LED is switched OFF. */
    LED_STATE_OFF = 0,
    /** LED is switched ON. */
    LED_STATE_ON = 1
} led_state_e;

/** @brief Periodic message payload structure. */
typedef struct __attribute__((packed))
{
    /** Running counter value. */
    uint32_t counter_value;
    /** Easy to spot data pattern. */
    uint8_t data_pattern[PERIODIC_MSG_DATA_PATTERN_LEN];
} payload_periodic_t;

/** @brief Periodic message set period payload data structure. */
typedef struct __attribute__((packed))
{
    /**
     * New periodic message period in milliseconds.
     * Note: Millisecond granulatity can be used when Low-latency operating
     * mode is enabled. When in Low-energy mode a millisecond granulatity
     * does not make sense as message travel time is long.
     */
    uint32_t new_period_ms;
} payload_periodic_set_t;

/** @brief Button event message payload structure. */
typedef struct __attribute__((packed))
{
    /** Id of pressed button. */
    uint8_t button_id;
    /** Button state value. */
    uint8_t button_state;
} payload_button_event_t;

/** @brief Echo command response payload struture. */
typedef struct __attribute__((packed))
{
    /** Sink to node "echo command" travel time in milliseconds. */
    uint32_t travel_time_ms;
} payload_response_echo_t;

/** @brief LED set state payload structure. */
typedef struct __attribute__((packed))
{
    /** LED ID to change state. */
    uint8_t led_id;
    /** LED state to apply according to @ref led_state_e. */
    uint8_t led_state;
} payload_led_state_set_t;

/** @brief LED get state payload structure. */
typedef struct __attribute__((packed))
{
    /** LED ID to get state from. */
    uint8_t led_id;
} payload_led_state_get_t;

/** @brief LED get state response payload structure. */
typedef struct __attribute__((packed))
{
    /** LED ID. */
    uint8_t led_id;
    /** LED state (see @ref led_state_e). */
    uint8_t led_state;
} payload_response_led_state_get_t;

/** @brief Application message data structure. */
typedef struct __attribute__((packed))
{
    uint8_t id;
    union
    {
        payload_periodic_t periodic;
        payload_periodic_set_t periodic_set_period;
        payload_button_event_t button_event;
        payload_response_echo_t resp_echo;
        payload_led_state_set_t led_state_set;
        payload_led_state_get_t led_state_get;
        payload_response_led_state_get_t resp_led_state_get;
    } payload;
} msg_t;

/** Application periodic message fixed data pattern. */
#if 0
static const uint8_t m_periodic_data_pattern[PERIODIC_MSG_DATA_PATTERN_LEN] = 
                                            {0x0A,0x0B,0x0B,0x0A,0x0A,0x0C,0x0D,0x0C};
#endif
/** Button ID which was pressed */
static volatile uint8_t m_button_pressed_id;

/** Period to send PERIODIC message, in us. */
static uint32_t m_period_ms;

/** Table holding all board's LED state. */
static led_state_e m_table_led_state[MAX_LED_PER_BOARD] = {0};

/** Unicast & broadcast messages handling callback */
static app_lib_data_receive_res_e unicast_broadcast_data_received_cb(
    const shared_data_item_t *item,
    const app_lib_data_received_t *data);

/** Unicast messages filter */
static shared_data_item_t unicast_packets_filter =
    {
        .cb = unicast_broadcast_data_received_cb,
        .filter = {
            .mode = SHARED_DATA_NET_MODE_UNICAST,
            /* Filtering by source endpoint. */
            .src_endpoint = DATA_EP,
            /* Filtering by destination endpoint. */
            .dest_endpoint = DATA_EP,
            .multicast_cb = NULL}};

/** Broadcast messages filter */
static shared_data_item_t broadcast_packets_filter =
    {
        .cb = unicast_broadcast_data_received_cb,
        .filter = {
            .mode = SHARED_DATA_NET_MODE_BROADCAST,
            /* Filtering by source endpoint. */
            .src_endpoint = DATA_EP,
            /* Filtering by destination endpoint. */
            .dest_endpoint = DATA_EP,
            .multicast_cb = NULL}};

int power(int x, unsigned int y)
{
    if (y == 0)
        return 1;
    else if (y % 2 == 0)
        return power(x, y / 2) * power(x, y / 2);
    else
        return x * power(x, y / 2) * power(x, y / 2);
}

// Reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * power(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

/**
 * @brief   Function sending uplink data.

 * @param   id
 *          ID of message to send, @ref message_id_e.
 * @param   payload
 *          Pointer to message payload data to send.
 *
 * @return  Result code, APP_LIB_DATA_SEND_RES_SUCCESS means that data was
 *          accepted for sending. See @ref app_res_e for other result codes.
 */
static app_lib_data_send_res_e send_uplink_msg(message_id_e id,
                                               uint8_t *payload)
{
    msg_t msg;                             /* Create uplink message structure. */
    size_t msg_byte_size = sizeof(msg.id); /* Message to send byte size. */

    msg.id = (uint8_t)id;

    switch (msg.id)
    {
    case MSG_ID_PERIODIC_MSG:
        memcpy(&msg.payload.periodic,
               (payload_periodic_t *)payload,
               sizeof(payload_periodic_t));

        msg_byte_size += sizeof(payload_periodic_t);
        break;

    case MSG_ID_BUTTON_EVENT_MSG:
        memcpy(&msg.payload.button_event,
               (payload_button_event_t *)payload,
               sizeof(payload_button_event_t));

        msg_byte_size += sizeof(payload_button_event_t);
        break;
    case MSG_ID_ECHO_RESPONSE_MSG:
        memcpy(&msg.payload.resp_echo,
               (payload_response_echo_t *)payload,
               sizeof(payload_response_echo_t));

        msg_byte_size += sizeof(payload_response_echo_t);
        break;
    case MSG_ID_LED_GET_STATE_RESPONSE_MSG:
        memcpy(&msg.payload.resp_led_state_get,
               (payload_response_led_state_get_t *)payload,
               sizeof(payload_response_led_state_get_t));

        msg_byte_size += sizeof(payload_response_led_state_get_t);
        break;
    default:
        /* Invalid message ID given : send only invalid msg ID. */
        break;
    }

    /* Create a data packet to send. */
    app_lib_data_to_send_t data_to_send;
    data_to_send.bytes = (const uint8_t *)&msg;
    data_to_send.num_bytes = msg_byte_size;
    data_to_send.dest_address = APP_ADDR_ANYSINK;
    data_to_send.src_endpoint = DATA_EP;
    data_to_send.dest_endpoint = DATA_EP;
    data_to_send.qos = APP_LIB_DATA_QOS_HIGH;
    data_to_send.delay = 0;
    data_to_send.flags = APP_LIB_DATA_SEND_FLAG_NONE;
    data_to_send.tracking_id = APP_LIB_DATA_NO_TRACKING_ID;

    /* Send the data packet. */
    return Shared_Data_sendData(&data_to_send, NULL);
}

void one_short_mode_enable()
{
    uint8_t addr = 0x00;
    uint8_t buffer, one_shot_buff[2];
    spi_dt_buffer.write_ptr = &addr;
    spi_dt_buffer.write_size = sizeof(addr);
    spi_dt_buffer.read_ptr = &buffer;
    spi_dt_buffer.read_size = 1U;

    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN);       // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN);         // drive_chip_cs_pin high.

    uch_cr0 = buffer;
    uch_cr0 &= Stop_Conversion_Bit;
    uch_cr0 |= 0x40;

    one_shot_buff[0] = 0x80;
    one_shot_buff[1] = uch_cr0;
    spi_dt_buffer.write_ptr = one_shot_buff;
    spi_dt_buffer.write_size = sizeof(one_shot_buff);
    spi_dt_buffer.read_ptr = &buffer;
    spi_dt_buffer.read_size = 1U;

    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN);       // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN);         // drive_chip_cs_pin high.
}

void measure_temp()
{

    uint8_t buffer[6];                          // variable for the copy the CJ/TC hex
    char res[10];

    one_short_mode_enable();                    // By calling this enable the one shot conversion mode

    spi_tx_buff[0] = 0x00;                      // Reading 16 bytes of sensor data from 0x0 register
    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 17U;

    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN);       // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN);         // drive_chip_cs_pin high.

    Print_Log("Sensor data =");
    //copy the 16 bytes sensor data to the local varable
    for (int i = 0; i < 16; i++)
    {
        send_bytes[i] = spi_rx_buff[i + 1];

        Print_Log(" %x ", send_bytes[i]); // By enabling this we can print the 16 byte of sensor data on serial teriminal
    }
    Print_Log("\n");

    buffer[0] = send_bytes[10];                        // copying CJ/TC hex value into local buffer
    buffer[1] = send_bytes[11];
    buffer[2] = send_bytes[12];
    buffer[3] = send_bytes[13];
    buffer[4] = send_bytes[14];

    // cold junction temperatore conversion
    CJ_temperature_value = (buffer[0] << 8 | buffer[1]) >> 2;

    if ((buffer[0] & 0x80) == 0x80)                   // Checking sign if negative condition match prints -ve degree C
    {
        CJ_temperature_value = 0x3FFF - CJ_temperature_value + 1;
        f_cold_junction_temperature = 0 - CJ_temperature_value * (-0.015625);
        Print_Log("Cold Junction temperature=");      
        ftoa(f_cold_junction_temperature, res, 6);   // ftoa to convert the float value into string format
        Print_Log("\"-%s\"\n", res);
    }
    else // prints +ve degree C
    {
        f_cold_junction_temperature = CJ_temperature_value * Cold_Junction_Resolution;
        Print_Log("Cold Junction temperature=");
        ftoa(f_cold_junction_temperature, res, 6);   // ftoa to convert the float value into string format
        Print_Log("\"%s\"\n", res);
    }

    //   Thermocouple temperature conversion
    TC_temperature_value = (buffer[2] << 16 | buffer[3] << 8 | buffer[4]) >> 5;

    if ((buffer[2] & 0x80) == 0x80)                  // Checking sign if negative condition match prints -ve degree C
    {
        TC_temperature_value = 0x7FFFF - TC_temperature_value + 1;
        f_linearized_tc_temperature = 0 - TC_temperature_value * (-TC_Resolution);
        Print_Log("Thermocouple temperature=");
        ftoa(f_linearized_tc_temperature, res, 6);   // ftoa to convert the float value into string format
        Print_Log("\"-%s\"\n", res);
    }
    else // prints +ve degree C
    {
        f_linearized_tc_temperature = TC_temperature_value * TC_Resolution;
        Print_Log("Thermocouple temperature=");
        ftoa(f_linearized_tc_temperature, res, 6);   // ftoa to convert the float value into string format          
        Print_Log("\"%s\"\n", res);
    }
    Print_Log("\n");
}

void sensor_setup()
{
    // Configuration 0 Register (CR0)
    spi_tx_buff[0] = 0x80;
    spi_tx_buff[1] = 0x15; // open circult fault enable/one short conversion mode

    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 1U;

    SPI_init(&spi_0);                     // initilazing the SPI0 for sensor
    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.

    // Configuration 1 Register 
    spi_tx_buff[0] = 0x81;
    spi_tx_buff[1] = 0x03; // Thermocouple K Type (default)

    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 1U;

    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN);
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN);

    /*Fault Mask Register this register by defult masked by
      configuring 0 we can read the below fault didection */

    spi_tx_buff[0] = 0x82;
    spi_tx_buff[1] = 0xc0;               // CJ High-\CJ Low \TC High \TC Low \OV UV \Open FAULT Mask all are enabled

    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 1U;

    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN);
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN);

    // Cold-Junction High Fault Threshold Register (CJHF) 
    spi_tx_buff[0] = 0x83;
    spi_tx_buff[1] = 0x7F;  // default configuration 

    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 1U;

    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.

    // Cold-Junction Low Fault Threshold Register (CJLF) 
    spi_tx_buff[0] = 0x84;
    spi_tx_buff[1] = 0xC0;  // default configuration

    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 1U;
    
    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.

    //Linearized Temperature High Fault Threshold Register, MSB (LTHFTH) 
    spi_tx_buff[0] = 0x85;
    spi_tx_buff[1] = 0x7F; // default configuration

    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 1U;

    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.

    //Linearized Temperature High Fault Threshold Register, LSB (LTHFTL)
    spi_tx_buff[0] = 0x86;
    spi_tx_buff[1] = 0xFF;  // default configuration

    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 1U;

    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.

    //Linearized Temperature Low Fault Threshold Register, MSB (LTLFTH) 
    spi_tx_buff[0] = 0x87;
    spi_tx_buff[1] = 0x80;  // default configuration

    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 1U;

    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.

    //Linearized Temperature Low Fault Threshold Register, LSB (LTLFTL) 
    spi_tx_buff[0] = 0x88;
    spi_tx_buff[1] = 0x00;  // default configuration

    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 1U;

    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.
    
    //Cold-Junction Temperature Offset Register (CJTO)
    spi_tx_buff[0] = 0x89;
    spi_tx_buff[1] = 0x00;  // default configuration

    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 1U;

    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN);   // drive_chip_cs_pin high.
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
static uint32_t task_send_periodic_msg(void)
{
 
    /*
     * by calling measure_temp function sensor data read
     */
    measure_temp();                  

    // In below filing the packet format to send pirodic message over network
    app_lib_data_to_send_t data_to_send;
    data_to_send.bytes = (const uint8_t *)send_bytes; 
    data_to_send.num_bytes = sizeof(send_bytes);
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

/**
 * @brief   Task handling "button pressed" message sending
 *
 *          This task will be called each time a button pressed event
 *          is generated.
 *          Note: Some event might be missed if new interrupts are generated and
 *          some previous message sending process is not over.
 *          Only the last event is sent.
 *
 *          This task is called when a button is pressed down.
 */
static uint32_t task_send_button_pressed_msg(void)
{
    payload_button_event_t payload; /* Message payload data. */

    /* Prepare payload data field. */
    payload.button_id = m_button_pressed_id;
    payload.button_state = (uint8_t)BUTTON_PRESSED_STATE;

    /* Send message. */
    if (send_uplink_msg(MSG_ID_BUTTON_EVENT_MSG,
                        (uint8_t *)&payload) != APP_LIB_DATA_SEND_RES_SUCCESS)
    {
        /*
         * Message was not accepted for sending.
         * Error handling can be performed here.
         */
    }

    return APP_SCHEDULER_STOP_TASK;
}

/**
 * @brief   Callback for button pressed event which store information to send in
 *          the "button event message"
 *
 *          This function will be called each time a button pressed event
 *          is generated.
 *
 * @param   button_id
 *          Button's number that was pressed
 * @param   event
 *          Always BUTTON_PRESSED here
 *
 * This function is called when a button is pressed down.
 */
static void button_pressed_handler(uint8_t button_id, button_event_e event)
{
    /* Store button_id to process it in a dedicated application task. */
    m_button_pressed_id = button_id;

    /*
     * Send "button pressed message" in a single shot application task (called
     * each time button is pressed) as we are here in an IRQ context.
     */
    App_Scheduler_addTask_execTime(task_send_button_pressed_msg,
                                   APP_SCHEDULER_SCHEDULE_ASAP,
                                   TASK_EXEC_TIME_US_SEND_BUTTON_PRESSED_MSG);
}

/**
 * @brief   Echo command response sending function
 * @param   delay
 *          Sink to node message delay
 */
static void send_echo_response_msg(uint32_t delay)
{
    /* Message payload data. */
    payload_response_echo_t payload =
        {
            .travel_time_ms = 0};

    /* Prevent overflow. In case of error "travel_time_ms" value is nil. */
    if (delay <= UINT32_MAX / 1000u)
    {
        /*
         * Downlink propagation time is expressed in 1/128th second.
         * Have to convert it into millisecond.
         */
        payload.travel_time_ms = COARSE_TO_MS(delay);
    }

    /* Send message. */
    if (send_uplink_msg(MSG_ID_ECHO_RESPONSE_MSG,
                        (uint8_t *)&payload) != APP_LIB_DATA_SEND_RES_SUCCESS)
    {
        /*
         * Message was not accepted for sending.
         * Error handling can be performed here.
         */
    }
}

/**
 * @brief   Periodic message period setup function
 *
 * @param   new_period_ms
 *          Received period value to apply
 */
static void set_periodic_msg_period(uint32_t new_period_ms)
{
    /*
     * Check new period value is in a valid range
     * and update period accordingly.
     */
    if ((new_period_ms >= PERIODIC_MSG_PERIOD_SET_MIN_VAL_MS) &&
        (new_period_ms <= PERIODIC_MSG_PERIOD_SET_MAX_VAL_MS))
    {
        m_period_ms = new_period_ms;

        /* Reschedule task to apply new period value. */
        App_Scheduler_addTask_execTime(task_send_periodic_msg,
                                       APP_SCHEDULER_SCHEDULE_ASAP,
                                       PERIODIC_WORK_EXECUTION_TIME_US);
    }
    else
    {
        /* Invalid period value do not change anything. */
    }
}

/**
 * @brief   LED state control function
 * @param   led_id
 *          LED ID of led to change state
 * @param   led_state
 *          Requested LED state to apply.
 */
static void set_led_state(uint8_t led_id, uint8_t led_state)
{
    /* Check if requested LED is available on the board. */
    uint8_t leds_num = Led_getNumber();

    if ((led_id < MAX_LED_PER_BOARD) && (led_id < leds_num) && (leds_num > 0))
    {
        /*
         * Valid LED ID requested:
         * execute command if valid LED state received.
         */
        if (led_state == LED_STATE_OFF)
        {
            Led_set(led_id, false); /* Switch off LED. */
            m_table_led_state[led_id] = LED_STATE_OFF;
        }
        else if (led_state == LED_STATE_ON)
        {
            Led_set(led_id, true); /* Switch on LED. */
            m_table_led_state[led_id] = LED_STATE_ON;
        }
        else
        {
            /* Invalid LED state received : do nothing. */
        }
    }
}

/**
 * @brief   LED get state send response function
 * @param   led_id
 *          LED ID to get status from.
 */
static void send_led_state(uint8_t led_id)
{
#if 0
    payload_response_led_state_get_t payload; /* Message payload data. */

    /*
     * Check if requested LED is available on the board
     * and that led state table is big enough to hold all led's status.
     */
    uint8_t leds_num = Led_getNumber();

    if ((led_id < MAX_LED_PER_BOARD) && (led_id < leds_num) && (leds_num > 0))
    {
        /* Prepare payload data. */
        payload.led_id = led_id;
        payload.led_state = (uint8_t)m_table_led_state[led_id];

        /* Send message. */
        if (send_uplink_msg(MSG_ID_LED_GET_STATE_RESPONSE_MSG,
                            (uint8_t *)&payload) != APP_LIB_DATA_SEND_RES_SUCCESS)
        {
            /*
             * Message was not accepted for sending.
             * Error handling can be performed here.
             */
        }
    }
#endif
    // triggering one shot temperature
    spi_tx_buff[0] = 0x80;
    spi_tx_buff[1] = 0x40;

    spi_dt_buffer.write_ptr = spi_tx_buff;
    spi_dt_buffer.write_size = sizeof(spi_tx_buff);
    spi_dt_buffer.read_ptr = spi_rx_buff;
    spi_dt_buffer.read_size = 1U;

    SPI_init(&spi_0);
    nrf_gpio_pin_clear(BOARD_SPI_CS_PIN); // drive_chip_cs_pin low.
    SPI_transfer(&spi_dt_buffer, NULL);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN); // drive_chip_cs_pin high.
    SPI_close();
}

/**
 * @brief   Data reception callback
 * @param   data
 *          Received data, @ref app_lib_data_received_t
 * @return  Result code, @ref app_lib_data_receive_res_e
 */
static app_lib_data_receive_res_e unicast_broadcast_data_received_cb(
    const shared_data_item_t *item,
    const app_lib_data_received_t *data)
{
    msg_t msg = *((msg_t *)data->bytes);
    uint8_t msg_size = data->num_bytes;

    if ((msg_size < sizeof(msg.id)))
    {
        /* Data is not for this application. */
        return APP_LIB_DATA_RECEIVE_RES_NOT_FOR_APP;
    }

    msg_size -= sizeof(msg.id);

    /* Process incoming message according to message ID. */
    switch (msg.id)
    {
    /*
     * First check received message length match expected one.
     * If not the case do not change anything else execute action.
     */
    case MSG_ID_ECHO_COMMAND_MSG:
        if (msg_size == 0)
        {
            send_echo_response_msg(data->delay);
        }
        break;

    case MSG_ID_PERIODIC_MSG_PERIOD_SET_MSG:
        if (msg_size == sizeof(payload_periodic_set_t))
        {
            set_periodic_msg_period(
                msg.payload.periodic_set_period.new_period_ms);
        }
        break;

    case MSG_ID_LED_SET_STATE_MSG:
        if (msg_size == sizeof(payload_led_state_set_t))
        {
            set_led_state(msg.payload.led_state_set.led_id,
                          msg.payload.led_state_set.led_state);
        }
        break;

    case MSG_ID_LED_GET_STATE_MSG:
        if (msg_size == sizeof(payload_led_state_get_t))
        {
            send_led_state(msg.payload.led_state_get.led_id);
        }
        break;

    default: /* Unknown message ID : do nothing. */
        break;
    }

    /* Data handled successfully. */
    return APP_LIB_DATA_RECEIVE_RES_HANDLED;
}

/**
 * @brief Period change callback
 * This function is called when an app config message concerning period change has been received
 */
static void appConfigPeriodReceivedCb(uint16_t type,
                                      uint8_t length,
                                      uint8_t *value_p)
{
    app_config_t *config;

    if (type != CUSTOM_PERIOD_TYPE)
    {
        /* It should never happen as we registered only this type with this cb. */
        LOG(LVL_ERROR, "Wrong app config type");
        return;
    }

    if (length != sizeof(app_config_t))
    {
        /* Wrong size. */
        LOG(LVL_ERROR, "Wrong app config size");
        return;
    }

    config = (app_config_t *)value_p;

    LOG(LVL_INFO,
        "New app configuration  interval_s=%d",
        config->interval);

    /* Set new periodic data transfer interval. */
    set_periodic_msg_period(config->interval * 1000);
}

/**
 * @brief   Initialization callback for application
 *
 * This function is called after hardware has been initialized but the
 * stack is not yet running.
 *
 */
void App_init(const app_global_functions_t *functions)
{
    LOG_INIT();

    LOG(LVL_INFO, "Evaluation App example start");

    shared_app_config_filter_t app_config_period_filter;

    /* configuring the spi to comunicate with sensor */
    spi_0.clock = 1000000;
    spi_0.mode = SPI_MODE_HIGH_FIRST;           // CPOL=1, CPHA=1 (You might need to test the SPI_MODE_LOW_FIRST if it does not work)
    spi_0.bit_order = SPI_ORDER_MSB;
    nrf_gpio_pin_dir_set(BOARD_SPI_CS_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_cfg_output(BOARD_SPI_CS_PIN);
    nrf_gpio_pin_set(BOARD_SPI_CS_PIN);

    /* App config. */
    Shared_Appconfig_init();

    /* Prepare the app_config filter for measurement rate. */
    app_config_period_filter.type = CUSTOM_PERIOD_TYPE;
    app_config_period_filter.cb = appConfigPeriodReceivedCb;
    Shared_Appconfig_addFilter(&app_config_period_filter, &m_filter_id);
    LOG(LVL_INFO, "Filter added for static period with id=%d\n", m_filter_id);

    uint8_t num_buttons;

    /* Basic configuration of the node with a unique node address. */
    if (configureNodeFromBuildParameters() != APP_RES_OK)
    {
        /*
         * Could not configure the node.
         * It should not happen except if one of the config value is invalid.
         */
        return;
    }

    /*
     * Set node operating mode (i.e low-energy or low-latency with autorole)
     * Default is low-energy.
     */
#ifdef ENABLE_LOW_LATENCY_MODE
    lib_settings->setNodeRole(
        app_lib_settings_create_role(APP_LIB_SETTINGS_ROLE_HEADNODE,
                                     APP_LIB_SETTINGS_ROLE_FLAG_LL |
                                         APP_LIB_SETTINGS_ROLE_FLAG_AUTOROLE));
#endif

    /* Init shared data module. */
    Shared_Data_init();

    /* Set up LED. */
    Led_init();

    /* Set up buttons. */
    Button_init();
    num_buttons = Button_get_number();

    for (uint8_t button_id = 0; button_id < num_buttons; button_id++)
    {
        /* Register button pressed event on all user available button. */
        Button_register_for_event(button_id,
                                  BUTTON_PRESSED,
                                  button_pressed_handler);
    }

    /* Init application scheduler. */
    App_Scheduler_init();

    /* Set a periodic task to be called after DEFAULT_PERIOD_MS. */
    m_period_ms = DEFAULT_PERIOD_MS;
    App_Scheduler_addTask_execTime(task_send_periodic_msg,
                                   APP_SCHEDULER_SCHEDULE_ASAP,
                                   PERIODIC_WORK_EXECUTION_TIME_US);

    /* Set unicast & broadcast received messages callback. */
    Shared_Data_addDataReceivedCb(&unicast_packets_filter);
    Shared_Data_addDataReceivedCb(&broadcast_packets_filter);

    /*
     * Init SPI
     * Configuring Thermocuple sensor   
     */
    sensor_setup();
    /*
     * Start the stack.
     * This is really important step, otherwise the stack will stay stopped and
     * will not be part of any network. So the device will not be reachable
     * without reflashing it.
     */
    lib_state->startStack();
}
