// Daniel Fajardo
// dfajardo@g.hmc.edu
// 10/22/2024
// 
// main script for lab5

#include "STM32L432KC.h"
#include "stm32l4xx.h"

#define PULSEA PA2 // input pin for pulse A
#define PULSEB PA3 // input pin for pulse B

// global variables
int loopdelay = 500; //
volatile uint32_t timecurrent = 0;
volatile uint32_t timepreva = 0;
volatile uint32_t timeprevb = 0;
volatile int direction = 0;
volatile float angularvelocity = 0;

int main(void) {
    // configureFlash();
    // enable motor pulse A of quadrature encoder
    gpioEnable(GPIO_PORT_A);
    pinMode(PULSEA, GPIO_INPUT);
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD2, 0b10); // Set PA2 as pull-down

    // enable motor pulse B of quadrature encoder
    pinMode(PULSEB, GPIO_INPUT);
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD3, 0b10); // Set PA3 as pull-down

    // Initialize timer
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    initTIM(TIM2);

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
    EXTI->IMR1 |= (1 << gpioPinOffset(PULSEB)); // Configure the mask bit
    // Disable rising edge trigger
    EXTI->RTSR1 &= ~(1 << gpioPinOffset(PULSEB));// Disable rising edge trigger
    // Enable falling edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(PULSEB));// Enable falling edge trigger
    // Turn on EXTI interrupt in NVIC_ISER
    NVIC->ISER[1] |= (1 << EXTI3_IRQn); // check to make sure IRQn maps to value 9 (position 9 in vector table)

    while (1) {
        delay_millis(TIM2, loopdelay);
        printf("%.3f angular velocity = ", angularvelocity);
        printf(" rev/s\n");
        if (direction==1) {printf("direction is cw\n");}
        else if (direction==-1) {printf("direction is ccw\n");}
        else {printf("direction is undetermined\n");}
    }
}

void EXTI2_IRQHandler(void){
    // Check that the pulse was what triggered our interrupt
    if (EXTI->PR1 & (1 << 2)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 2);

        timecurrent = TIM2->CNT; // Record the time of PULSEA

        uint32_t timedelta = timecurrent - timepreva; // Time for 1 revolution
        angularvelocity = 1000/timedelta; // Angular velocity calculation
        
        if (timepreva < timeprevb){
            direction = 1;
        }
        else {
            direction = -1;
        }
        TIM2->CNT = 0; // reset counter (only done in interrupt A to determine direction)
        TIM2->EGR |= (1<<0); // update event

        timepreva = timecurrent; // record previous time


    }
}

void EXTI3_IRQHandler(void){
    // Check that the pulse was what triggered our interrupt
    if (EXTI->PR1 & (1 << 3)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 3);

        timecurrent = TIM2->CNT; // Record the time of PULSEB

        uint32_t timedelta = timecurrent - timeprevb; // Time for 1 revolution
        angularvelocity = 1000/timedelta; // Angular velocity calculation

        timeprevb = timecurrent; // record previous time
    }
}

