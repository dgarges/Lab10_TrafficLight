// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****
#define SENSOR_E 								(*((volatile unsigned long *)0x4002401C))  		//GPIO Port E (APB): 0x4002.4000 bits 0,1,2
//#define SENSOR_F 								(*((volatile unsigned long *)0x40025044))	 	//GPIO Port F bits 0,4
#define LIGHT 									(*((volatile unsigned long *)0x400050FC))
//#define NVIC_ST_CTRL_R    			(*((volatile unsigned long *)0xE000E010))
//#define NVIC_ST_RELOAD_R  			(*((volatile unsigned long *)0xE000E014))
//#define NVIC_ST_CURRENT_R 			(*((volatile unsigned long *)0xE000E018))
//#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
	
//#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
//#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
//#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
//#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
//#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
//#define GPIO_PORTE_LOCK_R       (*((volatile unsigned long *)0x40024520))
//#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
//#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
//#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
//#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108)) 
	
//#define SW1 										(*((volatile unsigned long *)0x40025004))
//#define SW2											(*((volatile unsigned long *)0x40025040))
//#define PE0											(*((volatile unsigned long *)0x40024004))
//#define PE1 										(*((volatile unsigned long *)0x40024008))

struct State {
	unsigned long Out; // 6-bit pattern to output
	unsigned long Time; // delay in 10ms units
	unsigned long Next[5];}; // next state for inputs 0,1,2,3,4

typedef const struct State STyp;
#define goN   0
#define waitN 1
#define goE   2
#define waitE 3
#define walk  4

STyp FSM[5]={																			//xxxGY.RGYR
	{0x21, 1000, 	{goN,waitN,goN,waitN,waitN}},  		//Out: 0010.0001 
	{0x22, 300, 	{goE,goE,goE,goE,walk}},					//Out: 0010.0010
	{0x0C, 1000, 	{goE,goE,waitE,waitE,waitE}},			//Out: 0000.1100
	{0x14, 300, 	{goN,goN,goN,goN,walk}},					//Out: 0001.0100
	{0x24, 1000, 	{walk,goE,goN,goN,walk}}};				//Out: 0010.0100

unsigned long S; // index to the current state
unsigned long Input; 

// ***** 3. Subroutines Section *****
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void PortEBF_Init(void);
void PLL_Init_Local(void);
void SysTick_Init(void); 
void SysTick_Wait(unsigned long delay);
void SysTick_Wait10ms(unsigned long delay);
void Delay(void);



int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	EnableInterrupts();
	PLL_Init_Local();
	SysTick_Init();
	PortEBF_Init();
  
	S = 0;
  while(1){
		LIGHT = FSM[S].Out;  // set lights
    SysTick_Wait10ms(FSM[S].Time);

		if (SENSOR_E > 4)
			Input = 4;
		else 
			Input = SENSOR_E;
		
		//if (Input == 0)
			//GPIO_PORTF_DATA_R = 0x08;  // LED is green
		//if (Input == 1)
			//GPIO_PORTF_DATA_R = 0x04;  // LED is blue
		//if (Input == 2)
			//GPIO_PORTF_DATA_R = 0x02;  // LED is red
		//if (Input == 3)
			//GPIO_PORTF_DATA_R = 0x0E;  // LED is white
		//if (Input == 4)
			//GPIO_PORTF_DATA_R = 0x06;  // LED is 
		
    S = FSM[S].Next[Input]; 
		if (S == 4)
			GPIO_PORTF_DATA_R = 0x08;
		else
			GPIO_PORTF_DATA_R = 0x00;
  }
}

void PortEBF_Init(){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x32;  						// 1) B E F
	delay = SYSCTL_RCGC2_R;  						// 2) no need to unlock
	
	//Port F 
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0  
	
	//Port E
  GPIO_PORTE_AMSEL_R	&= 0x00;        // 2) disable analog function
  GPIO_PORTE_PCTL_R  	&= 0x00;   			// 3) GPIO clear bit PCTL  
  GPIO_PORTE_DIR_R 		&= ~0x07;       // 4.1) PE0 and PE1 and PE2 as inputs
  //GPIO_PORTE_DIR_R |= 0x02;         // 4.2) PE1 output, (set to 1)
  GPIO_PORTE_AFSEL_R &= 0x00;        // 5) no alternate function
  //GPIO_PORTE_PUR_R |= 0x10;          // 6) enable pullup resistor on PF4       
  GPIO_PORTE_DEN_R |= 0x07;          // 7) enable digital pins PE2 - PE1 - PE0	
	
	//Port B
	GPIO_PORTB_AMSEL_R &=~0x3F;  				// 3) disable analog function on PB5-0
	GPIO_PORTB_PCTL_R &= ~0x00FFFFFF;  	// 4) enable regular GPIO
	GPIO_PORTB_DIR_R |= 0x3F;  					// 5) outputs on PB5-0
	GPIO_PORTB_AFSEL_R &= ~0x3F;  			// 6) regular function on PB5-0
	GPIO_PORTB_DEN_R |= 0x3F;  					// 7) enable digital on PB5-0
}

void PLL_Init_Local(){
  SYSCTL_RCC2_R |=  0x80000000;  							 // 0) Use RCC2
  SYSCTL_RCC2_R |=  0x00000800;  							 // 1) bypass PLL while initializing// BYPASS2, PLL bypass
  SYSCTL_RCC_R = (SYSCTL_RCC_R &~0x000007C0)   // 2) select the crystal value and oscillator source// clear XTAL field, bits 10-6
                 + 0x00000540;   							 // 10101, configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~0x00000070;  							 // configure for main oscillator source
  SYSCTL_RCC2_R &= ~0x00002000;								 // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R |= 0x40000000;   							 // 4) set the desired system divider // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~ 0x1FC00000) // clear system clock divider
                  + (4<<22);      						 // configure for 80 MHz clock
  while((SYSCTL_RIS_R&0x00000040)==0){};  		 // 5) wait for the PLL to lock by polling PLLLRIS// wait for PLLRIS bit
  SYSCTL_RCC2_R &= ~0x00000800;           		 // 6) enable use of PLL by clearing BYPASS
}

void SysTick_Init(void){
	//NVIC_ST_CTRL_R has been defined at 0xE000E010
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 800000*12.5ns equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(200000);  // wait 10ms
  }
}

void Delay(void){unsigned long volatile time;
  //time = 727240*200/91;  // 0.1sec
	time = 72724*200/91;
  while(time){
		time--;
  }
}
