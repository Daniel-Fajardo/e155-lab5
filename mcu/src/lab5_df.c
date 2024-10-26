// Daniel Fajardo
// dfajardo@g.hmc.edu
// 10/22/2024
// 
// main script for lab5

#include "STM32L432KC.h"
#include "stm32l4xx.h"
#include <stdio.h>

#define PULSEA PA5 // input pin for pulse A
#define PULSEB PA10 // input pin for pulse B

// function prototypes
float calculate_velocity(int delayloop);
int calculate_direction(void);

// global variables
int loopdelay = 1000000; // in micro seconds (us)
volatile uint32_t timecurrent = 0;
volatile uint32_t timepreva = 0;
volatile uint32_t timeprevb = 0;
volatile uint32_t timediffa = 0;
volatile uint32_t timediffb = 0;
volatile int direction = 0;
volatile float angularvelocity = 0;
volatile int indexa = 0;
volatile int indexb = 0;

int main(void) {
    // configureFlash();
    // enable motor pulse A of quadrature encoder
    gpioEnable(GPIO_PORT_A);
    pinMode(PULSEA, GPIO_INPUT);
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD5, 0b10); // Set PA5 as pull-down

    // enable motor pulse B of quadrature encoder
    pinMode(PULSEB, GPIO_INPUT);
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD10, 0b10); // Set PA10 as pull-down

    // Initialize timer
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    initTIM(TIM2);
    initTIM(TIM6);

    // Enable SYSCFG clock domain in RCC
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    // Configure EXTICR for the input button interrupt
    SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR2_EXTI5, 0b000); // Select PA5
    SYSCFG->EXTICR[2] |= _VAL2FLD(SYSCFG_EXTICR3_EXTI10, 0b000); // Select PA10

    // Enable interrupts globally
    __enable_irq();

    // Configure interrupt for falling edge of GPIO pin for PULSEA
    EXTI->IMR1 |= (1 << gpioPinOffset(PULSEA)); // Configure the mask bit
    EXTI->RTSR1 &= ~(1 << gpioPinOffset(PULSEA));// Disable rising edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(PULSEA));// Enable falling edge trigger
    
    // Turn on EXTI interrupt in NVIC_ISER
    NVIC->ISER[0] |= (1 << EXTI9_5_IRQn); // check to make sure IRQn maps to value 23 (vector table)

    // Configure interrupt for falling edge of GPIO pin for PULSEB
    EXTI->IMR1 |= (1 << gpioPinOffset(PULSEB)); // Configure the mask bit
    EXTI->RTSR1 &= ~(1 << gpioPinOffset(PULSEB));// Disable rising edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(PULSEB));// Enable falling edge trigger

    // Turn on EXTI interrupt in NVIC_ISER
    NVIC->ISER[1] |= (1 << (EXTI15_10_IRQn-32)); // check to make sure IRQn maps to value 40 (vector table) (shift by 32)

    while (1) {
        delay_micros(TIM2, loopdelay);
        calculate_velocity(loopdelay);
        calculate_direction();
        printf("angular velocity = %.3f", angularvelocity); // ensure that segger project settings->printf->floating-point enabled
        printf(" rev/s\n");
        if (direction==1) {printf("direction is cw\n");}
        else if (direction==-1) {printf("direction is ccw\n");}
        else {printf("direction is undetermined\n");}
    }
}

void EXTI9_5_IRQHandler(void){
    // Check that PULSEA was what triggered interrupt
    if (EXTI->PR1 & (1 << PULSEA)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << PULSEA);

        indexa++; // count number of cycles

        timecurrent = TIM6->CNT; // Record the time of PULSEA

        timediffa = timeprevb - timepreva; // Time for 1 revolution
        timepreva = timecurrent; // record previous time
    }
}

void EXTI15_10_IRQHandler(void){
    // Check that PULSEB was what triggered interrupt
    if (EXTI->PR1 & (1 << PULSEB)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << PULSEB);

        indexb++; // count number of cycles

        timecurrent = TIM6->CNT; // Record the time of PULSEB
        
        timediffb = timepreva - timeprevb; // Time for 1 revolution
        timeprevb = timecurrent; // record previous time
    }
}


float calculate_velocity(int delayloop){
    if (indexa!=indexb) {printf("index not equal\n");}

    angularvelocity = indexa*1000000/delayloop; // in pulse/s
    angularvelocity = angularvelocity/120; // in rev/s (120 pulses per revolution)
    indexa = 0; // reset indexes
    indexb = 0;

    return angularvelocity;
}

int calculate_direction(void){
    // check if falling edges are closer for A->B or B->A
    if (timediffa < timediffb){
      direction = 1;
    }
    else {
      direction = -1;
    }
    
    return direction;
}