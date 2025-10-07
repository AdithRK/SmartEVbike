#include "stm32f4xx.h"
#include "cmn.h"  // Assuming you have delayms() etc.

void ADC1_Init(void);
uint16_t ADC1_Read(void);
void PWM_Init(void);
void GPIO_Init(void);

int main(void)
{
    uint16_t adc_value;
    uint16_t center = 2048;
    uint16_t dead_zone = 200;
    uint16_t pwm_value;

    SystemInit();
    GPIO_Init();
    ADC1_Init();
    PWM_Init();

    while (1)
    {
        adc_value = ADC1_Read();

        if (adc_value > (center + dead_zone))
        {
            // Forward
            GPIOB->ODR |= (1 << 7);   // IN1 HIGH
            GPIOB->ODR &= ~(1 << 8);  // IN2 LOW
            pwm_value = (adc_value - center) * 2;
            if (pwm_value > 4095) pwm_value = 4095;
            TIM4->CCR1 = pwm_value;
        }
        else if (adc_value < (center - dead_zone))
        {
            // Reverse
            GPIOB->ODR &= ~(1 << 7);  // IN1 LOW
            GPIOB->ODR |= (1 << 8);   // IN2 HIGH
            pwm_value = (center - adc_value) * 2;
            if (pwm_value > 4095) pwm_value = 4095;
            TIM4->CCR1 = pwm_value;
        }
        else
        {
            // Stop
            GPIOB->ODR &= ~((1 << 7) | (1 << 8)); // Both low
            TIM4->CCR1 = 0;
        }

        delayms(50);
    }
}

// ---------- GPIO for Motor Control ----------
void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock
    // PB6 (PWM), PB7, PB8 (direction)
    GPIOB->MODER |= (2 << 12); // PB6 alternate function (TIM4_CH1)
    GPIOB->AFR[0] |= (2 << 24); // AF2 for TIM4_CH1
    GPIOB->MODER |= (1 << 14) | (1 << 16); // PB7, PB8 output
}

// ---------- PWM Initialization (TIM4_CH1 @ PB6) ----------
void PWM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    TIM4->PSC = 84 - 1;    // 1 MHz timer clock
    TIM4->ARR = 4095;      // 12-bit resolution
    TIM4->CCMR1 |= (6 << 4);  // PWM mode 1 on CH1
    TIM4->CCER |= TIM_CCER_CC1E;
    TIM4->CR1 |= TIM_CR1_CEN;
}

// ---------- ADC1 Initialization (PC2 = ADC1_IN12) ----------
void ADC1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= (3 << (2 * 2)); // PC2 analog mode

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->SQR3 = 12;  // Channel 12
    ADC1->CR2 |= ADC_CR2_ADON;
}

// ---------- ADC1 Read ----------
uint16_t ADC1_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}
