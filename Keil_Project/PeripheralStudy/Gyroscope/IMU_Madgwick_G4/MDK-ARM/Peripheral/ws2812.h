/*!
 * @file ws2812.h
 * @brief Define the basic structure of ws2812 class
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2022-08-09
 */
#ifndef WS2812_H
#define WS2812_H

#include "main.h"

/**
 * \brief           Number of elements in statically allocated array
 */
#define ARRAYSIZE(x)                    (sizeof(x) / sizeof((x)[0]))

/**
 * \brief           Number of LEDs (or LED drivers in case of WS2811) connected to single strip in a row
 *
 * This value should be set to your defined length
 */
#define LED_CFG_COUNT                           1

/**
 * \brief           Number of bytes for one LED color description
 * \note            Set to `3` for RGB, or set to `4` for RGBW use case
 */
#define LED_CFG_BYTES_PER_LED                   3

/**
 * \brief           Defines size of raw transmission leds in units of "data-for-leds"
 * \note            `1 unit` represents `30us` at `800 kHz and 24-bit of LED data (RGB)`
 *                      or `40us` at ¨B800 kHz and 32-bit of LED data (RGBW)`
 * 
 * Based on the value:
 * - Greater it is, less frequent are DMA interrupts, but interrupt handling takes potentially more time
 * - Greater it is, larger is the raw dma buffer as it has to accomodate for `2*value` number of elements for led's data
 * 
 * \note            Values should be greater-or-equal than `1` and (advise) not higher than `8¨B
 * \note            Value set to `6` means DMA TC or HT interrupts are triggered at each `180us at 800kHz tim update rate for RGB leds`
 */
#define LED_CFG_LEDS_PER_DMA_IRQ                6

/**
 * \brief           Defines minimum number of "1-led-transmission-time" blocks
 *                  to send logical `0` to the bus, indicating reset before data transmission starts.
 * 
 * This is a must to generate reset value.
 * As per datasheet, different devices require different min reset time, normally min `280us`,
 * that represents minimum `10 led cycles`
 * 
 * \note            Few conditions apply to the value and all must be a pass
 *                      - Must be greater than `0`
 *                      - Must be set to a value to support minimum reset time required by the driver
 * 
 *                  Further advices
 *                      - Set it to `2*LED_CFG_LEDS_PER_DMA_IRQ` or higher
 *                      - Set it as multiplier of \ref LED_CFG_LEDS_PER_DMA_IRQ
 */
#define LED_RESET_PRE_MIN_LED_CYCLES             10

/**
 * \brief           Number of "1-led-transmission-time" units to send all zeros
 *                  after last led has been sent out
 * \note            It indicates minimum required value, while actual may be longer
 *                  and it has to do with multipliers of \ref LED_CFG_LEDS_PER_DMA_IRQ
 */
#define LED_RESET_POST_MIN_LED_CYCLES            8


/**
 * \brief           This buffer acts as raw buffer for DMA to transfer data from memory to timer compare
 * \note            DMA must have access to this variable (memory location)
 * 
 * It is a multi-dimentional array:
 * - First part represents 2 parts, one before half-transfer, second after half-transfer complete
 * - Second part is number of LEDs to transmit before DMA HT/TC interrupts get fired
 * - Third part are entries for raw timer compare register to send logical 1 and 0 to the bus
 * 
 * Type of variable should be unsigned 32-bit, to satisfy TIM2 that is 32-bit timer
 * Type of variable should be unsigned 8-bit, to satisfy 16-bit timer
 */
static uint32_t dma_buffer[(2) * (LED_CFG_LEDS_PER_DMA_IRQ) * (LED_CFG_BYTES_PER_LED * 8)];

/* Used macros */

/* Number of elements in raw DMA buffer array - used by DMA transfer length */
#define DMA_BUFF_ELE_LEN                    ((uint32_t)(sizeof(dma_buffer) / sizeof((dma_buffer)[0])))
/* Half number of elements in raw DMA buffer */
#define DMA_BUFF_ELE_HALF_LEN               ((uint32_t)(DMA_BUFF_ELE_LEN >> 1))
/* Size of (in bytes) half length of array -> used for memory set */
#define DMA_BUFF_ELE_HALF_SIZEOF            ((size_t)(sizeof(dma_buffer[0]) * DMA_BUFF_ELE_HALF_LEN))
/* Number of array indexes required for one led */
#define DMA_BUFF_ELE_LED_LEN                ((uint32_t)(LED_CFG_BYTES_PER_LED * 8))
/* Size of (in bytes) of one led memory in DMA buffer */
#define DMA_BUFF_ELE_LED_SIZEOF             ((size_t)(sizeof(dma_buffer[0]) * DMA_BUFF_ELE_LED_LEN))

/* Control variables for transfer */
static volatile uint8_t     is_updating = 0;            /* Set to `1` when update is in progress */
static volatile uint32_t    led_cycles_cnt;                  /* Counts how many "1-led-time" have been transmitted so far */
static volatile uint8_t     brightness = 0x52;          /* Brightness between 0 and 0xFF */
static volatile uint32_t    color_counter = 1;          /* Color, being increased each fade reaching 0 */

void led_init(void);

/* Start data transfer */
uint8_t  led_start_transfer(void);
/* 
* Type of ptr should be unsigned 32-bit, to satisfy TIM2 that is 32-bit timer
* Type of ptr should be unsigned 8-bit, to satisfy 16-bit timer
*/
static void     led_fill_led_pwm_data(size_t ledx, uint32_t* ptr);
void DMA1_Ch1_1_IRQHandler(void);

/* Test purpose only */
static uint32_t led_fill_counter;
static uint16_t led_fill_counter_arr[LED_CFG_COUNT * 2];
void fade(void);
void Color_Yaw(void);

#endif /* WS2812_H */
