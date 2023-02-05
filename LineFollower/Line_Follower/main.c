#include "stm32f10x.h"                  // Device header

///////////////////       Preperation        //////////////////
	int sensor_read = 0x00000000;
	int position;
	
	float Kp = 0.002; //set up the constants value
	float Ki = 0.005;
	float Kd = 20;
	float Kr = 0;
	int P, I, D, R;
	int lastError = 0;
	int errors[10] = {0,0,0,0,0,0,0,0,0,0};
	int error_sum = 0;
	int last_end = 0;	// 0 -> Left, 1 -> Right 
	int last_idle = 0;
	
	const uint8_t maxspeedr = 100;
	const uint8_t maxspeedl = 100;
	const uint8_t basespeedr = 22;
	const uint8_t basespeedl = 22;
	const int ARR = 10;
	
	int actives = 0;
	
	void TIM2config(void);
	void PWM(void);
	void initClockHSI (void);
	
	void delay_us(uint16_t us);
	void delay_ms(uint16_t ms);
	
	
	void motor_control (double pos_right, double pos_left);
	void forward_brake(int pos_right, int pos_left) ;
	void past_errors (int error);
	

//	void initGPIO(void);
	///////////////        Time    ///////////////////
	
	void initClockHSI (void)

{                   
	RCC->CR |= RCC_CR_HSION; 
	while(!(RCC->CR & RCC_CR_HSIRDY)){}
	
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_HSI;
			
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI){}
	
}
 void TIM2config(void)
{
//enable timer clk/
	RCC->APB1ENR |= (1<<0);
	//ARR value/
	TIM2->ARR = 0xffff-1;
	//*SET UP PRESCALER */
	TIM2->PSC = 72-1; //--us/
	//enable timer/
	TIM2->CR1 |= (1<<0);
	//Enable update flag/
	while(!(TIM2->SR & (1<<0))); //wait to set/
}
void delay_us(uint16_t us)
{
TIM2->CNT = 0; //set counter to 0/
	while(TIM2->CNT < us);
}
void delay_ms(uint16_t ms)
{ // her clock cycle 1us bu yüzden her 1us de counter 1 artar
for (uint16_t i=0; i<ms; i++)
	{
	delay_us(1000); 
	}
}
////////////////////////////////    Sensor read   ////////////////////////

int QTR_read()
{

//set all a pins as out/
//GPIOA->CRL &= ~(0xffffffff);
GPIOA->CRL |= GPIO_CRL_MODE0 | GPIO_CRL_MODE1 | GPIO_CRL_MODE2 | GPIO_CRL_MODE3 | GPIO_CRL_MODE4 | GPIO_CRL_MODE5 | GPIO_CRL_MODE6 | GPIO_CRL_MODE7; // MODE0 = 1
GPIOA->CRL &= ~GPIO_CRL_CNF0 &  ~GPIO_CRL_CNF1 &  ~GPIO_CRL_CNF2 &  ~GPIO_CRL_CNF3 &  ~GPIO_CRL_CNF4 &  ~GPIO_CRL_CNF5 &  ~GPIO_CRL_CNF6 &  ~GPIO_CRL_CNF7; // CNF0 = 0
/* just write out all  a pins to 1*/
GPIOA -> ODR |= 0xFFFFFFFF;

delay_us(12);

//set a0 as in/
//GPIOA -> CRL &= 0x00000000;
//GPIOA -> CRL |= 0x44444444;/
GPIOA->CRL &= ~GPIO_CRL_MODE0 & ~GPIO_CRL_MODE1 & ~GPIO_CRL_MODE2 & ~GPIO_CRL_MODE3 & ~GPIO_CRL_MODE4 & ~GPIO_CRL_MODE5 & ~GPIO_CRL_MODE6 & ~GPIO_CRL_MODE7; // MODE0 = 0
GPIOA->CRL &= ~GPIO_CRL_CNF0 & ~GPIO_CRL_CNF1 & ~GPIO_CRL_CNF2 &  ~GPIO_CRL_CNF3 & ~GPIO_CRL_CNF4 & ~GPIO_CRL_CNF5 & ~GPIO_CRL_CNF6 & ~GPIO_CRL_CNF7; // CNF0 = 0
GPIOA->CRL |= GPIO_CRL_CNF0_1 | GPIO_CRL_CNF1_1 | GPIO_CRL_CNF2_1 | GPIO_CRL_CNF3_1 | GPIO_CRL_CNF4_1 | GPIO_CRL_CNF5_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1; // CNF0 = 01
	
	//delay_us(11412);
delay_ms(6);
int a = 0;
int b = 1;
int c = 2;
int d = 3;
int e = 4;
int f = 5;
int g = 6;
int h = 7;
	
	sensor_read = 0x00000000;
	int pos = 0;
  int active = 0;
	
	  if ((GPIOA->IDR & (1<<h)) == (1<<h) ) {
    sensor_read |= 0x00000001;
    pos += 1000;
    active++;
    last_end = 1;
  }
  if ((GPIOA->IDR & (1<<g)) == (1<<g)) {
    sensor_read |= 0x00000010;
    pos += 2000;
    active++;
  }
 if ((GPIOA->IDR & (1<<f)) == (1<<f)) {
    sensor_read |= 0x00000100;
    pos += 3000;
    active++;
  }
 if ((GPIOA->IDR & (1<<e)) == (1<<e)) {
    sensor_read |= 0x00001000;
    pos += 4000;
    active++;
  }
 if ((GPIOA->IDR & (1<<d)) == (1<<d)) {
    sensor_read |= 0x00010000;
    pos += 5000;
    active++;
  }
 if ((GPIOA->IDR & (1<<c)) == (1<<c)) {
    sensor_read |= 0x00100000;
    pos += 6000;
    active++;
  }
 if ((GPIOA->IDR & (1<<b)) == (1<<b)) {
    sensor_read |= 0x01000000;
    pos += 7000;
    active++;
  }
 if ((GPIOA->IDR & (1<<a)) == (1<<a)) {
    sensor_read |= 0x10000000;
    pos += 8000;
    active++;
    last_end = 0;
  }
	
	GPIOB->CRL &= (0<<0);
	  actives = active;
	position = pos/active;
	
	if (actives == 0)
		last_idle++;
	else
		last_idle = 0;

	return pos/active;	 		 
}
//////////////     motor control      ///////////////


void motor_control (double pos_right, double pos_left) 
{
    // Set the duty cycle of the right motor
    if (pos_left < 0)
    {
        // Reverse rotation
        TIM1->CCR1 = ARR*0;
        TIM1->CCR2 = (uint32_t)(ARR * pos_left);
    } 
    else 
    {
        // Forward rotation
        TIM1->CCR1 = (uint32_t)(ARR * pos_left);
        TIM1->CCR2 = 0;
    }

    // Set the duty cycle of the left motor
    if (pos_right < 0)
    {
        // Reverse rotation
        TIM1->CCR3 = 0;
        TIM1->CCR4 = (uint32_t)(ARR * pos_right);
    } 
    else 
    {
        // Forward rotation
        TIM1->CCR3 = (uint32_t)(ARR * pos_right);
        TIM1->CCR4 = 0;
    }
}

void sharp_turn () 
	  {
    // Set the duty cycle of one motor to a high forward or reverse value, and the other motor to a low reverse or forward value
    if (last_idle < 25)
    {
        // Smaller turn
        if (last_end == 1)
            motor_control(-20, 100);
        else
            motor_control(100, -20);
    }
    else 
    {
        // Larger turn
        if (last_end == 1)
            motor_control(-53, 70);
        else
            motor_control(70, -53);
    }
}
void forward_brake(int pos_right, int pos_left) 
{
	if (actives == 0)
		sharp_turn();
	else
	  motor_control(pos_right, pos_left);
}

void past_errors (int error) 
{
  for (int i = 9; i > 0; i--) 
      errors[i] = errors[i-1];
  errors[0] = error;
}

int errors_sum (int index, int abs) 
{
  int sum = 0;
  for (int i = 0; i < index; i++) 
  {
    if (abs == 1 & errors[i] < 0) 
      sum += -errors[i]; 
    else
      sum += errors[i];
  }
  return sum;
}
void PID_control() 
	{
	uint16_t position = QTR_read();	
  int error = 4500 - position;
	past_errors(error);

  P = error;
  I = errors_sum(5, 0);
  D = error - lastError;
  R = errors_sum(5, 1);
  lastError = error;
	
  int motorspeed = P*Kp + I*Ki + D*Kd;
  
  int motorspeedl = basespeedl + motorspeed - R*Kr;
  int motorspeedr = basespeedr - motorspeed - R*Kr;
  
  if (motorspeedl > maxspeedl)
    motorspeedl = maxspeedl;
  if (motorspeedr > maxspeedr)
    motorspeedr = maxspeedr;
	
	forward_brake(motorspeedr, motorspeedl);
}
	
void PWM(void)
{
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN;
	
	GPIOA->CRH |= GPIO_CRL_MODE0; 
  GPIOA->CRH |= (GPIO_CRL_CNF0_1);
	GPIOA->CRH &= ~(GPIO_CRL_CNF0_0);
	
	GPIOA->CRH |= GPIO_CRH_MODE9; 
  GPIOA->CRH |= (GPIO_CRH_CNF9_1);
  GPIOA->CRH &= ~(GPIO_CRH_CNF9_0);

  GPIOA->CRH |= GPIO_CRH_MODE10; 
  GPIOA->CRH |= (GPIO_CRH_CNF10_1);
  GPIOA->CRH &= ~(GPIO_CRH_CNF10_0);
	
	GPIOA->CRH |= GPIO_CRL_MODE3; 
  GPIOA->CRH |= (GPIO_CRL_CNF3_1);
	GPIOA->CRH &= ~(GPIO_CRL_CNF3_0);
	
	

	TIM1->PSC = 710;
	TIM1->ARR = 100;
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC4E|TIM_CCER_CC2E | TIM_CCER_CC3E;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1|TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM1->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1|TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
	TIM1->BDTR |= (1 << 15);
	AFIO->MAPR &= ~AFIO_MAPR_TIM1_REMAP;

}

int main(void)
{
	initClockHSI();
	TIM2config();
	PWM();
	QTR_read();
	
//	TIM1->CCER |= TIM_CCER_CC1E;
//TIM1->CR1 |= TIM_CR1_CEN;

//// Enable PWM output on TIM4 channel 2
//TIM1->CCER |= TIM_CCER_CC2E;
//TIM1->CR1 |= TIM_CR1_CEN;
//	
//	TIM1->CCER |= TIM_CCER_CC3E;
//TIM1->CR1 |= TIM_CR1_CEN;

//// Enable PWM output on TIM4 channel 2
//TIM1->CCER |= TIM_CCER_CC4E;
//TIM1->CR1 |= TIM_CR1_CEN;
	while(1)
{
PID_control();
}
}