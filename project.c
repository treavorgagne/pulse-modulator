//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#define myTIM2_PRESCALER ((uint16_t)0x0000) /* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF) /* Maximum possible setting for overflow */

void myGPIOA_Init(); // init for port TIMER and Optocoupler
void myGPIOB_Init(); // init for port for LCD
void myGPIOC_Init(); // init for port for potentiameter
void myTIM2_Init(); // timer interupt init for rising and falling edge of frequency
void myTIM3_Init(); // time interupt init for lcd
void myEXTI_Init(); // init for exit interupt for freq
void myADC_Init();  // init for ADC conversion
void myDAC_Init();  // init for DAC conversion
void myLCD_Init();  // init for LCD
void lcd(uint32_t x); // function for writing a single character to LCD

volatile unsigned int edge = 0;
volatile float res = 0;
volatile float freq = 0;

int main(int argc, char* argv[])
{
    myGPIOA_Init();     /* Initialize I/O port PA */
    myGPIOB_Init();     /* Initialize I/O port PB */
    myLCD_Init();       /* Initialize LCD */
    myGPIOC_Init();     /* Initialize I/O port PC */
    myTIM2_Init();      /* Initialize timer TIM2 */
    myTIM3_Init();      /* Initialize timer TIM3 */
    myEXTI_Init();      /* Initialize EXTI */
    myADC_Init();       /* Initialize ADC */
    myDAC_Init();       /* Initialize DAC */

    int acd_value = 0;

    while (1)
    {
        while((ADC1->ISR & ADC_ISR_EOC) == 0);  // wait until the end of the conversion
        acd_value = ADC1->DR;                   // get adc value
        res = (float) (((acd_value * 3.3)/4095)/ 0.00066); // calculate resistance
        DAC->DHR12R1 = (unsigned int) acd_value; // send acd_value to dac register
    }

    return 0;
}

void myGPIOA_Init()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;      /* Enable clock for GPIOA peripheral */
    GPIOA->MODER &= ~(GPIO_MODER_MODER1_1); /* Configure PA1 as input */
    GPIOA->MODER &= ~(GPIO_MODER_MODER4_0); /* Configure PA4 as output */
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);   /* Ensure no pull-up/pull-down for PA1 */
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);   /* Ensure no pull-up/pull-down for PA4 */
}

void myGPIOB_Init()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;      /* Enable clock for GPIOB peripheral */
    GPIOB->MODER &= ~(GPIO_MODER_MODER7_1); // config PB7 as input
    GPIOB->MODER |= 0x55551500;    // config port B bits involved in lcd to output
    GPIOB->PUPDR &= ~(0xFFFF3F00); // ensure no pull-up/pull-down for port B bits
}

void myGPIOC_Init()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;      /* Enable clock for GPIOC peripheral */
    GPIOC->MODER |= GPIO_MODER_MODER1;      // configure PC1 for input analog
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);   /* Ensure no pull-up/pull-down for PC1 */
}

void myTIM2_Init()
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;/* Enable clock for TIM2 peripheral */
    /* Configure TIM2: buffer auto-reload, count up, stop on overflow,
     * enable update events, interrupt on overflow only */
    TIM2->CR1 = ((uint16_t)0x008C);
    TIM2->PSC = myTIM2_PRESCALER;/* Set clock prescaler value */
    TIM2->ARR = myTIM2_PERIOD;/* Set auto-reloaded delay */
    TIM2->EGR = TIM_EGR_UG;/* Update timer registers */
    NVIC_SetPriority(TIM2_IRQn, 0);/* Assign TIM2 interrupt priority = 0 in NVIC */
    NVIC_EnableIRQ(TIM2_IRQn);/* Enable TIM2 interrupts in NVIC */
    TIM2->DIER |= TIM_DIER_UIE;/* Enable update interrupt generation */
    TIM2->CR1 |= TIM_CR1_CEN;/* Start counting timer pulses */
}

void myTIM3_Init()
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* Enable clock for TIM3 peripheral */
    /* Configure TIM3: buffer auto-reload, count up, stop on overflow,
         * enable update events, interrupt on overflow only */
    TIM3->CR1 = (uint16_t)0x008C;
    TIM3->PSC = (uint16_t)0xFF00;   /* Set clock value delay for LCD */
    TIM3->ARR = (uint16_t)0x0032;   /* Set auto-reloaded delay 50ms */
    TIM3->EGR = TIM_EGR_UG; /* Update timer registers */
    NVIC_SetPriority(TIM3_IRQn, 0);/* Assign TIM3 interrupt priority = 0 in NVIC */
    NVIC_EnableIRQ(TIM3_IRQn);/* Enable TIM3 interrupts in NVIC */
    TIM3->DIER |= TIM_DIER_UIE;/* Enable update interrupt generation */
    TIM3->CR1 |= TIM_CR1_CEN;/* Start counting timer pulses */
}

void myEXTI_Init()
{
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA; /* Map EXTI1 line to PA1 */
    EXTI->RTSR |= EXTI_RTSR_TR1 ;/* EXTI1 line interrupts: set rising-edge trigger */
    EXTI->IMR |= EXTI_IMR_MR1;/* Unmask interrupts from EXTI1 line */
    NVIC_SetPriority(EXTI0_1_IRQn, 0);/* Assign EXTI1 interrupt priority = 0 in NVIC */
    NVIC_EnableIRQ(EXTI0_1_IRQn);/* Enable EXTI1 interrupts in NVIC */
}

void myADC_Init()
{
    ADC1->CR = (uint32_t)0x00000000;    // reset/clear ADC control register
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;  // enable ADC clock
    ADC1->CHSELR |= ADC_CHSELR_CHSEL11; // Enable channel 11 connected to port c
    ADC1->CFGR1 |= ADC_CFGR1_CONT;      // set continuous mode
    ADC1->CR |= ADC_CR_ADEN;            // ADC enable control
    while((ADC1->ISR & ADC_ISR_ADRDY) == 0);  // wait for ready flag
    ADC1->CR |= ADC_CR_ADSTART;         //start ADC i think
}

void myDAC_Init()
{
    DAC->CR = (uint32_t)0x00000000;     // reset/clear DAC control register
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;  // enable DAC clock
    DAC->CR = DAC_CR_EN1;               // enable DAC
}

void myLCD_Init() // code follows slide 5 in interface.ptx
{
    lcd(0x3800); // set lcd to 2 lines of 8 chars with DDRAM access to 8 bit interface
    lcd(0x0C00); // display on with no blinking or cursor
    lcd(0x0600); // auto increment DDRAM address
    lcd(0x0100); // clear display
}

void lcd(uint32_t x)// code follows slide 9 in interface.ptx based on handshake idea
{
    GPIOB->BRR = 0xFFFF;  // reset bits at register
    GPIOB->BSRR = x;    //send var to LDC
    GPIOB->BSRR |= 0x10;    //enable handshake/assert
    while((GPIOB->IDR & GPIO_IDR_7) == 0);  // wait for handshake to finish
    GPIOB->BRR = 0x10;  //deassert enable
    while((GPIOB->IDR & GPIO_IDR_7) != 0);  //wait for deasserted
}

void update_lcd(){

    char numToChar[10] = {'0','1','2','3','4','5','6','7','8','9'};
    int temp = (int) res;

    // get individual values of resitance
    int var4 = ((temp/1000)%10);
    int var3 = ((temp/100)%10);
    int var2 = ((temp/10)%10);
    int var1 = (temp%10);

    lcd(0xC000);    // write to bottom bar
    lcd(0x5220);    // write "R"
    lcd(0x3A20);    // write ":"

    lcd(numToChar[var4] << 8 | 0x0020); // write var4
    lcd(numToChar[var3] << 8 | 0x0020); // write var3
    lcd(numToChar[var2] << 8 | 0x0020); // write var2
    lcd(numToChar[var1] << 8 | 0x0020); // write var1

    lcd(0x4f20);    // write "O"
    lcd(0x6820);    // write "h"

    temp = (int) freq;

    // get individual values of frequency
    var4 = ((temp/1000)%10);
    var3 = ((temp/100)%10);
    var2 = ((temp/10)%10);
    var1 = (temp%10);

    lcd(0x8000);    // write to top bar
    lcd(0x4620);    // write "F"
    lcd(0x3A20);    // write ":"

    lcd(numToChar[var4] << 8 | 0x0020); // write var4
    lcd(numToChar[var3] << 8 | 0x0020); // write var3
    lcd(numToChar[var2] << 8 | 0x0020); // write var2
    lcd(numToChar[var1] << 8 | 0x0020); // write var1

    lcd(0x4820);    // write "H"
    lcd(0x7A20);    // write "z
}

void TIM2_IRQHandler()
{
    if ((TIM2->SR & TIM_SR_UIF) != 0)/* Check if update interrupt flag is indeed set */
    {
        TIM2->SR &= ~(TIM_SR_UIF); // clear update interrupt flag
        TIM2->CR1 |= TIM_CR1_CEN; // restart stopped timer
    }
}

void TIM3_IRQHandler()
{
    if ((TIM3->SR & TIM_SR_UIF) != 0)/* Check if update interrupt flag is indeed set */
    {
        update_lcd(); //updates lcd
        TIM3->SR &= ~(TIM_SR_UIF); // clear update interrupt flag
        TIM3->CR1 |= TIM_CR1_CEN; // restart stopped timer
    }
}

void EXTI0_1_IRQHandler()
{
    float counter = 0;

    /* Check if EXTI1 interrupt pending flag is indeed set */
    if ((EXTI->PR & EXTI_PR_PR1) != 0)
    {
        if(edge == 0){
            TIM2->CNT = 0x00000000; // clear count register
            TIM2->CR1 |= TIM_CR1_CEN; // start counter
            edge = 1; // set edge variable
        }
        else{
            TIM2->CR1 &= ~(TIM_CR1_CEN); // stop count register
            counter = TIM2->CNT; // get count value
            freq = SystemCoreClock/counter; // calculate freq using system core clock and counter
            edge = 0; // set edge variable
        }

        EXTI->PR = EXTI_PR_PR1; // clear interupt pending flag by wrting 1 to it
    }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
