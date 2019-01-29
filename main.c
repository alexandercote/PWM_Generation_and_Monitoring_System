
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: CENG 355 "Microprocessor-Based Systems".
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

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


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

//Initialization Functions

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI_Init(void);
static void myADC1_Init(void);
static void myDAC1_Init(void);
void LCD_Init(void);
void HC595_Setup(void);

// Helper Functions

void itoa(uint16_t, char[]);
void HC595_Setup(void);
void HC595_Write(char);
void Write_LCD_Text(char *);
void Write_LCD_Num(uint16_t);
void Position_LCD(unsigned short, unsigned short);
void refresh_LCD(void);
void Delay(uint32_t);

void myWrite_Cmd(char);
void myWrite_Data(char);

//Global variables
unsigned int edgecounter = 0;

float frequency = 0;
float resistance = 0;

int main(int argc, char* argv[]) {

	trace_printf("This is the Final Project for CENG 355 Lab.\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myTIM3_Init();		/* Initialize timer TIM3 */
	myADC1_Init();		// Initialize ADC
	myDAC1_Init();		// Initialize DAC

	HC595_Setup();
	LCD_Init();

	Position_LCD(0,0);
	Write_LCD_Text("F:");
	Position_LCD(0,6);
	Write_LCD_Text("Hz");
	Position_LCD(1,0);
	Write_LCD_Text("R:");
	Position_LCD(1,6);
	Write_LCD_Text("Oh");


	myEXTI_Init();		/* Initialize EXTI */


	while (1)
	{

		//Start ADC conversion
		ADC1->CR |= ADC_CR_ADSTART;
		// Wait for end of conversion flag
		while(!(ADC1->ISR & ADC_ISR_EOC));
		// Reset end of conversion flag
		ADC1->ISR = ~ADC_ISR_EOC;

		// voltage = Max output voltage * ADC output register / MAX ADC VALUE
		// Max ADC value is 4095, or 0xFFF (12 bits);
		// Max output voltage is 2.968V
		// However Octocoupler has deadzone between 0-1V due to diode.
		// DAC and output voltage are offset to compensate.
		// So voltage = ((MaxOutput - 1) * ADC Output / 4095) + 1

		float voltage = (1.968 * ADC1->DR / 4095) + 1;
		// Then, rescale the voltage to the DAC output
		// Max DAC value is 4095, or 0xFFF (12 bits);
		DAC->DHR12R1 = (4095 *voltage /2.968);
		//(1350 + (0.6666 * (ADC1->DR)));

		//Max potentiometer value of 5kohms.
		resistance = ((float) ADC1->DR *5000 / 4095);
		Position_LCD(0,0);
		Write_LCD_Text("F:");
		Write_LCD_Num((uint16_t)frequency);
		Position_LCD(0,6);
		Write_LCD_Text("Hz");
		Delay(10);
		Position_LCD(1,0);
		Write_LCD_Text("R:");
		Write_LCD_Num(resistance);
		//trace_printf("Resistance: %f\n", resistance);
		Position_LCD(1,6);
		Write_LCD_Text("Oh");
		Delay(10000);
		//trace_printf("ADC Value: %d\n", ADC1->DR);
		// trace_printf("DAC DHR Value: %d\n", DAC->DHR12R1);
		//trace_printf("DAC DOR Value: %d\n", DAC->DOR1);
		// To enable floats in trace_printf:
		// https://www.ece.uvic.ca/~brent/ceng355/printf.html

		/*
		 * Under the projects menu item select properties option.
		 * Expand out the C/C++ Build Section and select the settings option.
		 * Now select the tab "Tool Settings". Under the section "Cross ARM C++ Linker" select Miscellaneous.
		 * Click the checkbox that says "Use float with nano printf (-u_printf_float).
		*/

		//trace_printf("Voltage: %f\n",  voltage);
		//trace_printf("Resistance: %f\n", resistance);


	}

	return 0;

}

// ----- SECTION I: GENERAL INITIALIZATION: GPIOA - ADC/DAC - EXTI -----

void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	//enables peripheral clock for ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	//Enable PA0 as ADC input
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

	/* Configure PA1 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	// GPIOA->MODER &= (GPIO_MODER_MODER4); gives JTAG failure
	//Enable PA4 as DAC output
	GPIOA->MODER &= ~(GPIO_MODER_MODER4);
	GPIOA->MODER &= ~(GPIO_MODER_MODER4);

}

void myADC1_Init()
{
	//Range of ADC is from 0 to 4095
	//Enable clock for ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	//Start ADC calibration:
	ADC1->CR = ADC_CR_ADCAL;
	//wait until the adcal bit is cleared, then the ADC is calibrated
	while(ADC1->CR == ADC_CR_ADCAL);

	//configure CFGR1 in continuous mode, in overrun, right data element = 12 bit
	ADC1->CFGR1 |= 0x00003000;

	//Select channel 0
	ADC1->CHSELR = ADC_CHSELR_CHSEL0;

	//set sampling rate of 239.5
	ADC1->SMPR = ADC_SMPR_SMP; 

	//Turn on the ADC
	//Set ADEN=1 in ADC_CR register, ADRDY flag is set when ready
	//(Turn of ADC with ADDIS = 1)
	ADC1->CR |= ADC_CR_ADEN;

	//Enable interrupts for the ADRDY
	//ADC1->IER = ADC_IER_ADRDYIE;
	//Wait until ADRDY=1 in ADC_ISR register
	while(ADC1->ISR != ADC_ISR_ADRDY);

}

void myDAC1_Init()
{

	//Enable DAC clock
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	//Clear channel trigger enable bits and buffer
	//Need to clear bits 1 and 2 of DAC_CR
	DAC->CR &= 0xFFFFFFF9;

	//Step 2: Enable channel in the DAC
	DAC->CR |= DAC_CR_EN1;

	//DAC->CR |= DAC_CR_BOFF1;

}

void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= 0x0000FF0F;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);

}



// ----- SECTION II: TIMER INITIALIZATION AND DELAY FUNCTION -----

void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR |= ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);
	// Same as: NVIC->IP[3] = ((uint32_t)0x00FFFFFF);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);
	// Same as: NVIC->ISER[0] = ((uint32_t)0x00008000) */

	/* Enable update interrupt generation */
	TIM2->DIER |= TIM_DIER_UIE;
	// Relevant register: TIM2->DIER
	TIM2->CR1 |= TIM_CR1_CEN;
}


void myTIM3_Init()
{
	/* Enable clock for TIM3 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//Configure TIM3: count down
	TIM3->CR1 = 0x10;
	//TIM3->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value: 48MHz/(47999+1) = 1 KHz */
	// 47999 in Hex = 0xBB7F

	TIM2->PSC = 47999;
	/* Default auto-reloaded delay: 100 ms */
	TIM3->ARR = 100;

	/* Update timer registers */
	TIM3->EGR |= 0x0001;
}

void Delay(uint32_t delayms) //delayms in milliseconds
{
	/* Clear timer */
	TIM3->CNT |= 0x0;

	/* Clear update interrupt flag */
	TIM3->SR &= ~TIM_SR_UIF;

	/* Time-out value */
	TIM3->ARR = delayms;

	/* Update registers */
	TIM3->EGR |= 0x0001;

	/* Start timer */
	TIM3->CR1 |= TIM_CR1_CEN;

	/* Wait for time-out delay */
	while(!(TIM3->SR & TIM_SR_UIF));

	/* Stop Timer */
	TIM3->CR1 &= ~(TIM_CR1_CEN);

	/* Clear update interrupt flag */
	TIM3->SR &= ~(TIM_SR_UIF);
}



// ----- SECTION III: LCD INITIALIZATION FUNCTIONS: GPIOB, SPI, AND LCD -----

//Setup SPI
void HC595_Setup(void)
{
	//Enable SPI Timer
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	//Initialize the GPIOB registers for the SPI
	myGPIOB_Init();

	//Set up the the SPI struct, then run SPI_Init

	SPI_InitTypeDef	SPI_InitStructInfo;
	SPI_InitTypeDef	* SPI_InitStruct = &SPI_InitStructInfo;

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	//Speed of the SPI is 48Mhz / 256
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &SPI_InitStructInfo);

	//Initialize the SPI with the defined values. Function defined in stm32f0xx_spi.h on line 535
	SPI_Cmd(SPI1, ENABLE); //Enables the SPI

	trace_printf("SPI INITIALIZED\n");
}

void myGPIOB_Init()
{
	//Used for SPI
	/* Enable clock for GPIOB peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// GPIO Alternate Function config

	// Configure PB3 and PB5 alternate function

	// Configure PB4 in output mode to be used as storage clock input in 74HC595

	//Registers PB5, PB3, and PB4 used
	// PB3 is SPI1 clock SCK
	GPIOB->MODER |= (GPIO_MODER_MODER3_1);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR3);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);

	//PB4 is Storage clock LCK
	GPIOB->MODER |= (GPIO_MODER_MODER4_0);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR4);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

	//PB5 is SPI1 output MOSI
	GPIOB->MODER |= (GPIO_MODER_MODER5_1);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR5);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);
}

void LCD_Init(void)
{
	//write to instruction register IR from MPU

	for(int i = 0; i <= 2; i++){
		// Let EN = 0, RS = 0, DB[7:4] = 0010
		HC595_Write(0x03);
		// Let EN = 1, RS = 0, DB[7:4] = 0010
		HC595_Write(0x83);
		// Let EN = 0, RS = 0, DB[7:4] = 0010
		HC595_Write(0x03);
		Delay(10);
	}

	HC595_Write(0x02);
	HC595_Write(0x82);
	HC595_Write(0x02);

	// 4-bits, 2 lines, 5x7 font
    // [Bit 4:DL=0 (4 bit interface) Bit 3: N = 1 (2 lines), Bit 2: F=0 (5x8)]
    myWrite_Cmd(0x28);
	// Display ON, No cursors, No blinking
    myWrite_Cmd(0x0C);
    // Return Home
    //myWrite_Cmd(0x02);
	// Entry mode- Auto-increment, No Display shifting
    myWrite_Cmd(0x06);
	// Clear display
    myWrite_Cmd(0x01);
    // no shift, cursor moves right
   // myWrite_Cmd(0x14);
}



// ----- SECTION IV: LCD WRITE AND HELPER FUNCTIONS -----

void Position_LCD(unsigned short x, unsigned short y)
{
	//unsigned short address = 128 + y;
	//if (x == 2)
	//	address = address + 64;

	uint8_t address = ((x*0x40) | 0x80 | y);
	myWrite_Cmd(address);
}


//Write a string to LCD
void Write_LCD_Text(char *StrData)
{
	unsigned short p;
	unsigned short q = strlen(StrData);
	for (p = 0; p<q; p++)
	{
		unsigned short temp = StrData[p];
		myWrite_Data(temp);
	}
}


// Writes numbers on LCD
void Write_LCD_Num(uint16_t num)
{
	char Pot_st[4]={'0','0','0','0'};
	itoa(num,Pot_st);
	Write_LCD_Text(Pot_st);
}


//Converts integer to character
void itoa(uint16_t n, char s[])
{
	uint8_t count = 3;
	if (n != 0)
	{
		while (n>0)
		{
			s[count]=(n%10)+48;
			n/=10;
			count--;
		}
	}
}

// Write 8-bit data to LCD
void myWrite_Data(char data)
{
	char Low_Nibble, High_Nibble;
	// Bitwise order is [EN][RS][x][x][4-bit cmd]

	Low_Nibble = data & 0x0F;
	High_Nibble= (data >> 4) & 0x0F;
	//High_Nibble
		// Let RS = 1 and EN = 0
		HC595_Write(0x40 | High_Nibble);
		// Let RS = 1 and EN = 1
		HC595_Write(0xC0 | High_Nibble);
		// Let RS = 1 and EN = 0
		HC595_Write(0x40 | High_Nibble);
		//Low_Nibble
		// Let RS = 1 and EN = 0
		HC595_Write(0x40 | Low_Nibble);
		// Let RS = 1 and EN = 1
		HC595_Write(0xC0 | Low_Nibble);
		// Let RS = 1 and EN = 0
		HC595_Write(0x40 | Low_Nibble);
	Delay(1);
}

// Write 8-bit command to LCD
void myWrite_Cmd(char cmd)
{
	char Low_Nibble, High_Nibble;
	// Bitwise order is [EN][RS][x][x][4-bit cmd]

	Low_Nibble = cmd & 0x0F;
	High_Nibble= (cmd >> 4) & 0x0F;
	//High_Nibble
		// Let RS = 0 and EN = 0
		HC595_Write(0x00 | High_Nibble);
		// Let RS = 0 and EN = 1
		HC595_Write(0x80 | High_Nibble);
		// Let RS = 0 and EN = 0
		HC595_Write(0x00 | High_Nibble);
	//Low_Nibble
		// Let RS = 0 and EN = 0
		HC595_Write(0x00 | Low_Nibble);
		// Let RS = 0 and EN = 1
		HC595_Write(0x80 | Low_Nibble);
		// Let RS = 0 and EN = 0
		HC595_Write(0x00 | Low_Nibble);
	Delay(1);
}

// Send 8-bit data through SPI
void HC595_Write(char data)
{
	// Set LCK = 0
	// PB4 is Storage clock LCK
	// Set PB4 to 0 -> Change output data register bit with GPIOx_BSRR
	GPIOB->BSRR = GPIO_BSRR_BR_4; //Resets bit 4.

	// Wait until not busy
	// Check the TXE flag (SPI has enough space to store data to send)
	// OR Check the BSY flag to see if the SPI is busy
	// Loops until the set flag is triggered
	//while(!((SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == SET)
	//	  ||(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == RESET)));
	//while(!(SPI1->SR & SPI_SR_TXE) || (SPI1->SR & SPI_SR_BSY));
	while(((SPI1->SR & SPI_SR_TXE) == 0) & ((SPI1->SR & SPI_SR_BSY)==1)) {};

	SPI_SendData8(SPI1,data);
	//Delay(1);
	// Wait until not busy
	// This time, only check when BSY = 0, so when the SPI is done sending
	//while(!(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == RESET));
	while(SPI1->SR & SPI_SR_BSY);
	// LCK = 1 (rising edge)
	// Using BSRR, the set registers are the first 16 bits.
	GPIOB->BSRR = GPIO_BSRR_BS_4;

	// 1-ms delay (slow LCD)
	Delay(1);
}



// ----- SECTION V: INTERUPTS -----

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	// Your local variables...
	unsigned int elapsedtime = 0;

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//	- Stop timer (TIM2->CR1).
		TIM2->CR1 = ((uint16_t)0x0000);
		// 1. If this is the first edge:
		if(edgecounter == 0){
			//	- Clear count register (TIM2->CNT)
			TIM2->CNT = 0x00000000; //might need to uint32_t = 0
			//	- Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;
			edgecounter++;
		}
		//	  Else (this is the second edge):
		else{
			//	- Stop timer (TIM2->CR1).
			TIM2->CR1 = ((uint16_t)0x0000);
			//	- Read out count register (TIM2->CNT).
			elapsedtime = TIM2->CNT;
			//	- Calculate signal period and frequency.
			frequency = (double)SystemCoreClock/(double)elapsedtime;
			//	- Print calculated values to the console.
			//	  NOTE: Function trace_printf does not work
			//	  with floating-point numbers: you must use
			//	  "unsigned int" type to print your signal
			//	  period and frequency.
			trace_printf("The frequency is %u Hz. \n", (unsigned int)frequency);
			edgecounter = 0;
		}
		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR = EXTI_PR_PR1;
		//
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
