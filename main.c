#include<stdint.h>
#include<stm32f10x.h>

void En_clock(void);
void gpio_setup(void);
void delay_ms(void);
void delay(uint32_t count);
void systick_config(void);
uint8_t debounce(uint8_t last);

void ADC_config(void);


int main(void){
	uint8_t lastb = 0, currentb=0;
	En_clock();
	gpio_setup();
	systick_config();
	ADC_config();
	
	uint16_t analog_data = 0;
	
	// reading current level to set value max and min
	delay(1000);
	if(ADC1->SR & ADC_SR_EOC){ 
	analog_data = ADC1->DR;
	}
	uint16_t max_val = analog_data - 0x0100;
	uint16_t min_val = max_val - 0x0200;

	while(1){
			
			
	//If conversion is done, read the data
			if(ADC1->SR & ADC_SR_EOC){ 
				analog_data = ADC1->DR;
				//analog_data = analog_data;
				}
	// Turn on 1 led if its dark, 2 if natural, 3 if bright	
				if( analog_data>max_val){
					//turn on A2 and A3, A5
					GPIOA->ODR |= GPIO_ODR_ODR2;
					GPIOA->ODR |= GPIO_ODR_ODR3;
					GPIOA->ODR |= GPIO_ODR_ODR5;
				}
				else if ( analog_data<min_val){
					//Turn on A2. but A3, and A5 turn off
					GPIOA->ODR |= GPIO_ODR_ODR2;
					GPIOA->ODR &= ~GPIO_ODR_ODR3;
					GPIOA->ODR &= ~GPIO_ODR_ODR5;		
				
				}
				else {
					
					GPIOA->ODR |= GPIO_ODR_ODR2;
					GPIOA->ODR |= GPIO_ODR_ODR3;
					GPIOA->ODR &= ~GPIO_ODR_ODR5;	
				
				}
		
			
			//checking loop main function is running 
			GPIOA->ODR |=  GPIO_ODR_ODR6;
			delay(20);
			GPIOA->ODR &= ~GPIO_ODR_ODR6;
			delay(20);
						
	}
	
	return 0;
}


void En_clock(void){
RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
}

void gpio_setup(void){
//PA) as push button
	GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
	GPIOA->CRL |= GPIO_CRL_MODE0;
	
// PA1 as analog in put cnf=00, mode =00;
	GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);
	GPIOA->CRL |= 0UL;
	
//PA5 as led output cnf =00, mode = 11;
	GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
	GPIOA->CRL |= GPIO_CRL_MODE5;
	
//PA2 as push-pull output cnf =00, mode = 11;
	GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
	GPIOA->CRL |= GPIO_CRL_MODE2;

//PA3 as push-pull output cnf =00, mode = 11;
	GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
	GPIOA->CRL |= GPIO_CRL_MODE3;
	
//PA6 as push-pull output cnf =00, mode = 11;
	GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
	GPIOA->CRL |= GPIO_CRL_MODE6;
}


void systick_config(void){
	SysTick->LOAD = 72000-1;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE | SysTick_CTRL_ENABLE;
}

void delay_ms(void){
	while(! ( SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
}

void delay(uint32_t count){
while(count--){
	delay_ms();
}}

uint8_t debounce(uint8_t last){
	uint8_t current = (GPIOA->IDR & GPIO_IDR_IDR0)? 1 : 0;
	
	if ( last!= current){
		delay(5);
		current = (GPIOA->IDR & GPIO_IDR_IDR0)? 1 : 0;
	}
return current;
}

void ADC_config(void){
	// Using ADC1 and pin PA1. which is at channel 1;
	
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_AFIOEN;

	ADC1->CR2 = 0;
	ADC1->SQR3 = 1;
	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT;//ADC power on 
	delay(500);
	ADC1->CR2 |= ADC_CR2_ADON;//enable adc and start continuous conversion 
}
