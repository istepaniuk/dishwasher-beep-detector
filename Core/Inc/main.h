#ifndef __MAIN_H
#define __MAIN_H

#define GPIO_LED GPIO_PIN_13

// The sampling frequency as determined by the TIM3 interval and prescaling.
#define SAMPLING_FREQ 72000

// The threshold from which to consider that the tone is present
#define TONE_DETECTION_THRESHOLD 120000000

// The frequency of the tone to search for, in Hertz.
#define TONE_FREQ 3969

// The amount of samples in the ADC circular buffer.
// When the buffer is full, a DMA interrupt occurs.
#define ADC_BUFFER_SIZE 1024

#include "stm32f1xx_hal.h"

int main(void);
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static double goertzel(uint16_t *buffer, size_t buffer_size, float coefficient);
void Error_Handler(void);

#endif