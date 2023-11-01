/*!
 * @file ws2812.c
 * @brief Driver library for ws2812
 * @copyright	USETC, LIMITI
 * @author [JingxianLin]
 * @version V1.0
 * @date 2022-08-09
 */



#include "ws2812.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "Magic.h"
#include "bsp_fdcan.h"

/**
 * \brief           Application array to store led colors for application use case
 *                      Data in format R,G,B,R,G,B,... 
 * 
 * Array used by user application to store data to
 */
uint8_t leds_color_data[LED_CFG_BYTES_PER_LED * LED_CFG_COUNT];

/* Application variables for fading effect */
int8_t      fade_step;
int16_t     fade_value;
int8_t 			color;
/**
 * \brief           Calculate Bezier curve (ease-in, ease-out) for input value
 * 
 * \note            This is experimental function, bezier is not approproate curve
 *                  to fade-in-out LEDs
 *
 * \param[in]       t: Input value between 0 and 1, indicating start to stop position
 * \return          Value between 0 and 1, respecting minimum and maximum
 */
float
bezier_calc(float t) {
    return t * t * (3.0f - 2.0f * t);
}

/**
 * \brief           Calculate cub of an input for lightning approx curve
 * 
 * \param[in]       t: Input value between 0 and 1, indicating start to stop position
 * \return          Value between 0 and 1, respecting minimum and maximum
 */
float
quad_calc(float t) {
    return t * t * t * t;
}

void led_init(void)
{
/* Set up the first leds with dummy color */
    for (size_t i = 0; i < LED_CFG_COUNT; ++i) {
        leds_color_data[i * LED_CFG_BYTES_PER_LED + 0] = 255;
        leds_color_data[i * LED_CFG_BYTES_PER_LED + 1] = 0;
        leds_color_data[i * LED_CFG_BYTES_PER_LED + 2] = 127;
    }

    /* Define fade init values */
    fade_step = 0x02;
    fade_value = 0;	
		
		/*
     * This is where DMA gets configured for data transfer
     *
     * - Circular mode, continuous transmission
     * - Memory length (number of elements) for 2 LEDs of data
     */
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)dma_buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, DMA_BUFF_ELE_LEN);

    /* Clear flags, enable interrupts */
    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_ClearFlag_HT1(DMA1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);

    /* Enable DMA, TIM channel and TIM counter */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
    LL_TIM_EnableCounter(TIM2);
		LL_TIM_EnableAllOutputs(TIM2);

    /* All the rest is happening in DMA interrupt from now on */
		LL_mDelay(100);
		led_start_transfer();
		LL_mDelay(500);
}

/**
 * \brief           Update sequence function,
 *                  called on each DMA transfer complete or half-transfer complete events
 * \param[in]       tc: Set to `1` when function is called from DMA TC event, `0` when from HT event
 */
static void led_update_sequence(uint8_t tc) {
    tc = tc ? 1 : 0;
    if (!is_updating) {
        return;
    }

    /*
     * led_cycles_cnt variable represents minimum number of 
     * led cycles that will be transmitted on the bus.
     * 
     * It is set to non-0 value after transfer and gets increased in each interrupt.
     * 
     * Interrupts are triggered (TC or HT) when DMA transfers `LED_CFG_LEDS_PER_DMA_IRQ` led cycles of data elements
     */
    led_cycles_cnt += LED_CFG_LEDS_PER_DMA_IRQ;

    if (led_cycles_cnt < LED_RESET_PRE_MIN_LED_CYCLES) {
#if LED_CFG_LEDS_PER_DMA_IRQ > 1
        /*
         * We are still in reset sequence for the moment.
         *
         * When LED_CFG_LEDS_PER_DMA_IRQ > 1
         * and pre-reset led cycles is not aligned to LED_CFG_LEDS_PER_DMA_IRQ value,
         * first leds need to be filled already at this stage
         */
        if ((led_cycles_cnt + LED_CFG_LEDS_PER_DMA_IRQ) > LED_RESET_PRE_MIN_LED_CYCLES) {
            uint32_t index = LED_RESET_PRE_MIN_LED_CYCLES - led_cycles_cnt;
            /* We need to preload some values for some leds now */
            /* Otherwise we will miss some leds */
            for (uint32_t i = 0; index < LED_CFG_LEDS_PER_DMA_IRQ && i < LED_CFG_COUNT; ++index, ++i) {
                led_fill_led_pwm_data(i, &dma_buffer[tc * DMA_BUFF_ELE_HALF_LEN + (index % LED_CFG_LEDS_PER_DMA_IRQ) * DMA_BUFF_ELE_LED_LEN]);
            }
        }
#endif /* LED_CFG_LEDS_PER_DMA_IRQ > 1 */
    } else if (led_cycles_cnt < (LED_RESET_PRE_MIN_LED_CYCLES + LED_CFG_COUNT)) {
        /*
         * This is where we prepare data for next cycle only
         *
         * next led is simply calculated by subtracting counter from reset cycle
         * If reset is not aligned, it is important to handle first leds already in previous if statement,
         * otherwise we will miss first x leds (where x depends on how much we are away from alignment)
         */
        uint32_t next_led = led_cycles_cnt - LED_RESET_PRE_MIN_LED_CYCLES;
#if LED_CFG_LEDS_PER_DMA_IRQ == 1
        led_fill_led_pwm_data(next_led, &dma_buffer[tc * DMA_BUFF_ELE_HALF_LEN]);
#endif /* LED_CFG_LEDS_PER_DMA_IRQ == 1 */
#if LED_CFG_LEDS_PER_DMA_IRQ > 1
        uint32_t counter = 0;
        /*
         * Fill buffer with led data
         *
         * If counter stops earlier than buffer ends (low number of leds or not aligned),
         * fill remaining with all zeros to indicate post-reset signal
         */
        for (; counter < LED_CFG_LEDS_PER_DMA_IRQ && next_led < LED_CFG_COUNT; ++counter, ++next_led) {
            led_fill_led_pwm_data(next_led, &dma_buffer[tc * DMA_BUFF_ELE_HALF_LEN + counter * DMA_BUFF_ELE_LED_LEN]);
        }
        if (counter < LED_CFG_LEDS_PER_DMA_IRQ) {
            memset(&dma_buffer[tc * DMA_BUFF_ELE_HALF_LEN + counter * DMA_BUFF_ELE_LED_LEN], 0x00, (LED_CFG_LEDS_PER_DMA_IRQ - counter) * DMA_BUFF_ELE_LED_SIZEOF);
        }
#endif /* LED_CFG_LEDS_PER_DMA_IRQ > 1 */
    } else if (led_cycles_cnt < (LED_RESET_PRE_MIN_LED_CYCLES + LED_CFG_COUNT + LED_RESET_POST_MIN_LED_CYCLES + LED_CFG_LEDS_PER_DMA_IRQ)) {
        /*
         * This is post-reset and must be set to at least 1 level in size
         * and sends all zeros to the leds.
         * 
         * Manually reset array to all zeros using memset, but only if it has not been set already
         * (to not waste CPU resources for small MCUs)
         */
        if (led_cycles_cnt < (LED_RESET_PRE_MIN_LED_CYCLES + LED_CFG_COUNT + 2 * LED_CFG_LEDS_PER_DMA_IRQ)) {   
            memset(&dma_buffer[tc * DMA_BUFF_ELE_HALF_LEN], 0x00, DMA_BUFF_ELE_HALF_SIZEOF);
        }
    } else {
        /*
         * We are now ready to stop DMA and TIM channel,
         * otherwise transfers will continue to occur (circular mode).
         *
         * Disable interrupts prior disabling DMA transfer.
         *
         * Some STM32 (F2, F4, F7) may generate TC or HT interrupts when DMA is manually disabled,
         * and since it is not necessary to receive these interrupts from now-on,
         * it's better to simply disable them
         */
        LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_1);
        LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
        LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH3);
        is_updating = 0;      
    }
}

/**
 * \brief           DMA1 channel 1 interrupt handler
 */
void DMA1_Ch1_1_IRQHandler(void) {
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_1) && LL_DMA_IsActiveFlag_HT1(DMA1)) {
        LL_DMA_ClearFlag_HT1(DMA1);
        led_update_sequence(0);
    }
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_1) && LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_DMA_ClearFlag_TC1(DMA1);
        led_update_sequence(1);
    }
}

/**
 * \brief           Prepares data from memory for PWM output for timer
 * \note            Memory is in format R,G,B, while PWM must be configured in G,R,B[,W]
 * \param[in]       ledx: LED index to set the color
 * \param[out]      ptr: Output array with at least LED_CFG_RAW_BYTES_PER_LED-words of memory
 */
static void led_fill_led_pwm_data(size_t ledx, uint32_t* ptr) {
    const uint32_t arr = TIM2->ARR + 1;
    const uint32_t pulse_high = (3 * arr / 4) - 1;
    const uint32_t pulse_low = (1 * arr / 4) - 1;

    if (ledx < LED_CFG_COUNT) {
        uint32_t r, g, b;

        //led_fill_counter_arr[led_fill_counter++] = ledx;

        r = (uint8_t)(((uint32_t)leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 0] * (uint32_t)brightness) / (uint32_t)0xFF);
        g = (uint8_t)(((uint32_t)leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 1] * (uint32_t)brightness) / (uint32_t)0xFF);
        b = (uint8_t)(((uint32_t)leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 2] * (uint32_t)brightness) / (uint32_t)0xFF);
        for (size_t i = 0; i < 8; i++) {
            ptr[i] =        (g & (1 << (7 - i))) ? pulse_high : pulse_low;
            ptr[8 + i] =    (r & (1 << (7 - i))) ? pulse_high : pulse_low;
            ptr[16 + i] =   (b & (1 << (7 - i))) ? pulse_high : pulse_low;
        }
    }
}

/**
 * \brief           Start with transfer process to update LED on the strip
 * \return          `1` if transfer has started, `0` otherwise
 */
uint8_t led_start_transfer(void) {
    if (is_updating) {
        return 0;
    }

    /* Set initial values */
    is_updating = 1;
    led_cycles_cnt = LED_CFG_LEDS_PER_DMA_IRQ;

    /*
     * At the very beginning we should prepare data for full array length
     *
     * 2 steps are done
     * - Reset buffer to zero, for reset pulse (all zeros)
     * - Manually set data for first round, in case reset lenght is set lower than size of array
     */
    memset(dma_buffer, 0x00, sizeof(dma_buffer));
    for (uint32_t i = 0, index = LED_RESET_PRE_MIN_LED_CYCLES; index < 2 * LED_CFG_LEDS_PER_DMA_IRQ; ++index, ++i) {
        led_fill_led_pwm_data(i, &dma_buffer[index * DMA_BUFF_ELE_LED_LEN]);
    }
		
		/* Clear flags, enable interrupts */
    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_ClearFlag_HT1(DMA1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);

    /* Enable DMA, TIM channel and TIM counter */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
    LL_TIM_EnableCounter(TIM2);

    /* All the rest is happening in DMA interrupt from now on */
    return 1;
}

void fade(void)
{
		/* Calculate fading efect */
		fade_value += fade_step;
		if (fade_value > 0xFF) {
				fade_value = 0xFF;
				fade_step = -fade_step;
		} else if (fade_value < 0) {
				fade_value = 0;
				fade_step = -fade_step;
		}

		/* Check if we need to change the color */
		if (fade_value == 0) {
				for (size_t i = 0; i < LED_CFG_COUNT; ++i) {
						leds_color_data[i * LED_CFG_BYTES_PER_LED + 0] = color_counter & 0x01 ? 0xFF : 0x00;
						leds_color_data[i * LED_CFG_BYTES_PER_LED + 1] = color_counter & 0x02 ? 0xFF : 0x00;
						leds_color_data[i * LED_CFG_BYTES_PER_LED + 2] = color_counter & 0x04 ? 0xFF : 0x00;
				}
				color_counter++;
		}

		/* Calculate new brightness */
		brightness = (uint8_t)(quad_calc((float)fade_value / (float)0xFF) * (float)0x3F);
}

void Color_Yaw(void){
	
	float Angle;
	Angle = Comm_CAN.yaw;
	uint8_t R = 0;
	uint8_t G = 0;
	uint8_t B = 0;
	float Scale = 0.0f;
	
	if(Angle>=0 && Angle<60)
	{
		Scale = Angle/60;
		R = 0;
		G = 255;
		B = 255*Scale;
	}
	else if(Angle>=60 && Angle<120)
	{
		Scale = (Angle-60)/60;
		R = 0;
		G = 255*(1-Scale);
		B = 255;
	}
	else if(Angle>=120 && Angle<=180)
	{
		Scale = (Angle-120)/60;
		R = 255*Scale;
		G = 0;
		B = 255;
	}
	else if(Angle>=-180 && Angle<=-120)
	{
		Scale = (Angle+180)/60;
		R = 255;
		G = 0;
		B = 255*(1-Scale);
	}
	else if(Angle>=-120 && Angle<-60)
	{
		Scale = (Angle+120)/60;
		R = 255;
		G = 255*Scale;
		B = 0;
	}
	else if(Angle>-60 && Angle<0)
	{
		Scale = (Angle+60)/60;
		R = 255*(1-Scale);
		G = 255;
		B = 0;
	}
	for (size_t i = 0; i < LED_CFG_COUNT; ++i) {
		leds_color_data[i * LED_CFG_BYTES_PER_LED + 0] = R;
		leds_color_data[i * LED_CFG_BYTES_PER_LED + 1] = G;
		leds_color_data[i * LED_CFG_BYTES_PER_LED + 2] = B;
	}
}
/********************************End of File************************************/
