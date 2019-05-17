#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define GLOBAL_INTERRUPT  SREG |= (1<<7)
#define SET_BIT(PORT,PIN)  PORT|=(1<<PIN)
#define CLR_BIT(PORT,PIN)  PORT&=~(1<<PIN)
#include <Servo.h>
#define mask_var 1
#define SERVO_SIGNAL PD3
#define ANGLE_INPUT PC0
#define UP_SWITCH PC2
#define UP_LIMIT PC3
#define DOWN_SWITCH PC4
#define DOWN_LIMIT PC5
#define ENGINE_ST PD2
#define ENGINE_LED PD6
#define WINDOW_PIN1 PB0
#define WINDOW_PIN2 PB1
#define PIR_INPUT PD5
#define INTERIOR_LIGHT PB5

Servo servo_test; 
volatile uint8_t Engine;
int correctvoicecommand = 0;
int wrongvoicecommand = 0;
unsigned int counter = 0;
uint8_t read_switch;
uint8_t read_mirror_switch;
int angle=0;
int Int_init();

int setup()
{ 
  Serial.begin(9600);
  servo_test.attach(3); 
  gpio_init();
  Int_init();
  pin_config();
  InitADC();
}

int main()
{ setup();
  read_switch=0x00;
  read_mirror_switch=0x00;
  read_switch=PIND;
  while(!(read_switch&(1<<ENGINE_ST)))
    {  
       read_switch=PIND;
       SET_BIT(PORTD, ENGINE_LED);
       if(read_switch&(1<<PIR_INPUT))
       {
         SET_BIT(PORTB,INTERIOR_LIGHT);
          _delay_ms(5);
       }
       if(!(read_switch&(1<<PIR_INPUT)))
       {
        CLR_BIT(PORTB,INTERIOR_LIGHT);
       }
       angle=ReadADC(0);
       angle = map(angle, 0, 1023, 0, 179);
       servo_test.write(angle); 
       _delay_ms(5);
       read_mirror_switch=PINC;
       Serial.println( read_mirror_switch);
        if((read_mirror_switch&(1<<UP_SWITCH)) && ((read_mirror_switch&(1<<UP_LIMIT))) )
        {
         Serial.println("sw1 && SW2");
          SET_BIT(PORTB,WINDOW_PIN2);
          CLR_BIT(PORTB,WINDOW_PIN1);
        }
        if((read_mirror_switch&(1<<DOWN_SWITCH))&&((read_mirror_switch&(1<<DOWN_LIMIT))))
        {
         Serial.println("sw3 && SW4");
          SET_BIT(PORTB,WINDOW_PIN1);
          CLR_BIT(PORTB,WINDOW_PIN2);
        }
        if((!(read_mirror_switch&(1<<DOWN_SWITCH))&& !(read_mirror_switch&(1<<UP_SWITCH))))
        { CLR_BIT(PORTB,WINDOW_PIN1);
          CLR_BIT(PORTB,WINDOW_PIN2);
        }


   /*    if(correctvoicecommand==0 && wrongvoicecommand==0)
       {
    	PORTD &= ~(1<<PD7);
    	VoiceCommandNotReceived();//LED will toggle //DC motor off
       }
    
       else if(correctvoicecommand==1 && wrongvoicecommand==0)
       {
       
         CorrectVoicecommandReceived();//LED will be steady on
         if(PINB & (1<<PB4))
         {
         	DoorLock();//Dc motor on
         }
         else
         {
           PORTD &= ~(mask_var<<PD7);//Dc motor off
         }
        }
        
       else if(correctvoicecommand==0 && wrongvoicecommand==1 )
       {
    
        WrongVoiceCommandReceived();//LED off Dc motor off
      
       }
       else if(correctvoicecommand==1 && wrongvoicecommand==1){
        VoiceCommandNotReceived();//LED off Dc motor off
         
       }*/
    
    }
       
  CLR_BIT(PORTD,ENGINE_LED);
  CLR_BIT(PORTD,SERVO_SIGNAL);
  CLR_BIT(PORTB,INTERIOR_LIGHT);
}


int InitADC()
{
ADMUX=(1<<REFS0);             // For Aref=AVcc;
ADCSRA=(1<<ADEN)|(7<<ADPS0);
ADCSRA=(1<<ADEN);
}

uint16_t ReadADC(uint8_t ch)
{
	//Select ADC Channel ch must be 0-7
	ADMUX&=0xf8;
	ch=ch&0b00000111;
	ADMUX|=ch;
	ADCSRA|=(1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));
	ADCSRA|=(1<<ADIF);
	return(ADC);
}

int pin_config()
{
 CLR_BIT(DDRD,PD2);   // Engine SW
 SET_BIT(PORTD,PD2);  
 SET_BIT(DDRD,PD3);
 CLR_BIT(DDRD,PD5);
 SET_BIT(DDRB,PB5);
 SET_BIT(DDRB,PB0);
 SET_BIT(DDRB,PB1);
 CLR_BIT(DDRC,PC2);
 CLR_BIT(DDRC,PC3);
 CLR_BIT(DDRC,PC4);
 CLR_BIT(DDRC,PC5);
 SET_BIT(PORTC,PC2);
 SET_BIT(PORTC,PC3);
 SET_BIT(PORTC,PC4);
 SET_BIT(PORTC,PC5);
}

void gpio_init()
{
  DDRB |= (mask_var<<PB4); // LED 
  PORTB &=~ (mask_var<<PB4);
 
  DDRB &=~(mask_var<<PB3); //Switch 2
  //PORTB &=~ (mask_var<<PB3);
  
  DDRD &= ~(mask_var<<PD4); // Switch 1
  //PORTD &= ~(1<<PD4);
  
  DDRD |=(mask_var<<PD7); // MOTOR
  PORTD &= ~(mask_var<<PD7);
}


int Int_init()
{
   
  sei();
  PCICR |= (1<<PCIE2);
  PCMSK2 |=(1<<PCINT20);
  PCICR |= (1<<PCIE0);
  PCMSK0 |= (1<<PCINT3);
  
 // timer_init();
}

void timer_init(){
  
TCCR1A &= ~(1<< WGM10);
TCCR1A &= ~(1<< WGM11);
TCCR1B &= ~(1<< WGM12);
TIMSK1 |= (1<<TOIE1);
TCNT1 = 0x00;
clk_source();    
 
}


////////////////clock select function///////////////////////////
void clk_source()
{

  TCCR1B &= ~(1<<CS11);
  TCCR1B |= ((1<<CS10) | (1<<CS12)); 

}

//////////////////clock select function//////////
void CorrectVoicecommandReceived(){
  
  PORTB |=(mask_var<<PB4);
  counter = 0;
}

void DoorLock()
{
  PORTD |= (1<<PD7);
  Serial.println("Motor On");
}

void VoiceCommandNotReceived(){
   PORTB |=(mask_var<<PB4);
  _delay_ms(500);
   PORTB &=~(mask_var<<PB4);
    _delay_ms(500);
   PORTD &= ~(1<<PD7);
}

void WrongVoiceCommandReceived(){

   PORTD &= ~(mask_var<<PD7);
   PORTB &= ~(mask_var<<PB4); 
}

ISR(PCINT2_vect){ // Switch 1 ISR

 //correctvoicecommand = !correctvoicecommand;
  Serial.println("Step 1");
  correctvoicecommand ^= 1;
}

ISR(PCINT0_vect) // Switch 2 ISR
{ 
  Serial.println("Step 2");
  //wrongvoicecommand = !wrongvoicecommand; 
  wrongvoicecommand ^= 1;
}

ISR(TIMER1_OVF_vect)
{
  counter++;
}
