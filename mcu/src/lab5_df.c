// Daniel Fajardo
// dfajardo@g.hmc.edu
// 10/06/2024
// 
// main script for lab5

#include "STM32L432KC.h"
#include "stm32l4xx.h"

#define PULSEA PA2 // input pin for pulse A
#define PULSEB PA3 // input pin for pulse B

int main(void) {
    // enable motor pulse A of quadrature encoder
    gpioEnable(GPIO_PORT_A);
    pinMode(PULSEA, GPIO_INPUT);
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD2, 0b10); // Set PA2 as pull-down

    // enable motor pulse B of quadrature encoder
    pinMode(PULSEB, GPIO_INPUT);
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD3, 0b10); // Set PA3 as pull-down

    // Initialize timer
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    initTIM(DELAY_TIM);

    // Enable SYSCFG clock domain in RCC
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    // Configure EXTICR for the input button interrupt
    SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR1_EXTI2, 0b000); // Select PA2
    SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR1_EXTI3, 0b000); // Select PA3

    // Enable interrupts globally
    __enable_irq();

    // Configure interrupt for falling edge of GPIO pin for PULSEA
    // Configure mask bit
    EXTI->IMR1 |= (1 << gpioPinOffset(PULSEA)); // Configure the mask bit
    // Disable rising edge trigger
    EXTI->RTSR1 &= ~(1 << gpioPinOffset(PULSEA));// Disable rising edge trigger
    // Enable falling edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(PULSEA));// Enable falling edge trigger
    // Turn on EXTI interrupt in NVIC_ISER
    NVIC->ISER[0] |= (1 << EXTI2_IRQn); // check to make sure IRQn maps to value 8 (position 8 in vector table)

    // Configure interrupt for falling edge of GPIO pin for PULSEB
    // Configure mask bit
    EXTI->IMR2 |= (1 << gpioPinOffset(PULSEB)); // Configure the mask bit
    // Disable rising edge trigger
    EXTI->RTSR2 &= ~(1 << gpioPinOffset(PULSEB));// Disable rising edge trigger
    // Enable falling edge trigger
    EXTI->FTSR2 |= (1 << gpioPinOffset(PULSEB));// Enable falling edge trigger
    // Turn on EXTI interrupt in NVIC_ISER
    NVIC->ISER[1] |= (1 << EXTI3_IRQn); // check to make sure IRQn maps to value 9 (position 9 in vector table)

}

void EXTI2_IRQHandler(void){
    // Check that the pulse was what triggered our interrupt
    if (EXTI->PR1 & (1 << 0)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 0);

        // Record the time of pulse
        // ----
    }
}

void EXTI3_IRQHandler(void){
    // Check that the pulse was what triggered our interrupt
    if (EXTI->PR2 & (1 << 0)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR2 |= (1 << 0);

        // Record the time of pulse
        // ----
    }
}