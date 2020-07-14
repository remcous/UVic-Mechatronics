/*########################################################################
# PROGRAM : main.c
# PROJECT : Final_Project
# GROUP : 5
# NAME 1 : Remi Coussement V00903991
# NAME 2 : Martin Figel V00693070
# DESC : This program operates a conveyor belt system to sort parts
		into four bins using a variety of sensors
########################################################################*/

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/********************************************************************
*					    Linked List Structure						*
********************************************************************/

typedef struct {
	char itemCode;
} element;

typedef struct link{
	element		e;
	struct link *next;
} link;

/********************************************************************
*						 FUNCTION PROTOTYPES						*
********************************************************************/

// Timers
void mTimer(int count);
void pwmTimer(char duty);

// Linked List - FIFO
void initLink(link **newLink);
void setup(link **h, link **t);
void clearQueue(link **h, link **t);
void enqueue(link **h, link **t, link **nL);
void dequeue(link **h, link **t, link **deQueuedLink);
element firstValue(link **h);
char isEmpty(link **h);
int size(link **h, link **t);

// Stepper
void init_stepper(void);
void stepper_90_CW(void);
void stepper_180_CW(void);
void stepper_90_CCW(void);

// LCD
void lcd_init(void);
void lcd_home(void);
void lcd_clear(void);
void lcd_move(char, char);
void lcd_command(char);
void lcd_write(char*);
inline void _writeNibble(char);
inline void _writeByte(char);

/********************************************************************
*					 	  Global Variables							*
********************************************************************/

// Sorting variables
volatile unsigned short ALUMINUM_LOW = 0;
volatile unsigned short STEEL_LOW = 300;
volatile unsigned short WHITE_LOW = 800;
volatile unsigned short BLACK_LOW = 860;
#define BLACK 0x00
#define STEEL 0x03
#define WHITE 0x02
#define ALUMINUM 0x01
volatile unsigned char EX_FLAG = 0;
volatile unsigned char PART_PASSED_FLAG = 0;

// Stepper variables - PORT A
unsigned char POSITION[] = {0b00110110, 0b00101110, 0b00101101, 0b00110101};
volatile int CURRENT_POSITION = BLACK;
volatile unsigned char STEPPER_DONE_FLAG = 0x00;
volatile char CURRENT_BIN = BLACK;

// FIFO Variables
volatile link *head, *tail;
volatile link *newLink, *rtnLink;
volatile char FIFO_ADD_FLAG = 0x00;
volatile link *nextPiece = NULL;
volatile link *currentPiece = NULL;

// ADC Variables - PORT F
volatile unsigned short ADC_RESULT;		// int to hold a single ADC result
volatile char ADC_READ = 0;			// flag for ADC read
volatile unsigned short ADC_MIN = 0xFF;		// holds lowest ADC result for current part
volatile char ADC_RESULT_FLAG = 0;	// flag to indicate ADC conversion is complete

// DC Motor Variables - PORT B
#define MOTOR_ON 0b00100000		// PORTB | ON = motor runs
#define MOTOR_OFF 0b10000000	// PORTB & OFF = motor stops
#define MOTOR_DUTY 0x40			// 50% duty cycle - !TEMP initial trial value

// LCD - PORT C
#define E  0b00000010
#define RS 0b00000001
#define D4 0b00010000
#define D5 0b00100000
#define D6 0b01000000
#define D7 0b10000000
volatile char lcd_string[5];

/********************************************************************
*							    MAIN								*
********************************************************************/

int main(void){
	TCCR1B|=_BV(CS10);		// mTimer pre-scale = 1
	cli();					// disable all interrupts
	
	// ACTIVE LOW push button - Motor On/Off
	EIMSK|= _BV(INT0);		// enable INT0 - PORT D - PIN 0
	EICRA|= _BV(ISC01);		// falling edge interrupt
	
	// ACTIVE HIGH push button - Kill switch
	EIMSK|= _BV(INT1);		// enable INT1 - PORT D - PIN 1
	EICRA|= _BV(ISC11) |_BV(ISC10);	// rising edge interrupt
	
	// ACTIVE LOW Optical Sensor for Inductive OI
	EIMSK|= _BV(INT2);		// enable INT2 - PORT D - PIN 2
	EICRA|= _BV(ISC21);		// falling edge interrupt
	
	// ACTIVE HIGH Optical Sensor for Reflective OR
	EIMSK|= _BV(INT3);		// enable INT3 - PORT D - PIN 3
	EICRA|= _BV(ISC31) |_BV(ISC30);	// rising edge interrupt
	
	// ACTIVE LOW Optical Sensor Exit Gate EX
	EIMSK|= _BV(INT4);		// enable INT4 - PORT E - PIN 4
	EICRB|= _BV(ISC41);		// falling edge interrupt

	// default ADC input Right-Adjusted PF0
	ADCSRA|= _BV(ADEN);		// enable ADC
	ADCSRA|= _BV(ADIE);		// enable interrupt of ADC
	ADCSRA|= (_BV(ADPS2) | _BV(ADPS1));	// set prescale of 64
	ADMUX |= _BV(REFS0);	// sets VREF to AVcc external
	
	// PORT I/O Configuration
	DDRA = 0xFF;	// output to stepper motor - bin
	DDRB = 0xFF;	// output to DC Motor - conveyor
	DDRC = 0xFF;	// output to LCD Display
	DDRD = 0x00;	// input to feed interrupts
	DDRE = 0x00;	// input from EX sensor
	DDRF = 0x00;	// input to ADC converter
	
	// Initialize linked list
	newLink = NULL;			// no new link yet
	rtnLink = NULL;			// no link returned yet
	setup(&head, &tail);	// Setup FIFO
	
	lcd_init();		// initialize LCD display
	
	// run first ADC conversion (takes longer)
	lcd_home();
	lcd_write("ADC Test");
	ADCSRA |= _BV(ADSC);
	while((ADCSRA & _BV(ADSC))!=0);
	ADCSRA |= _BV(ADIF);	// clear interrupt flag from first ADC
	lcd_clear();
	lcd_home();
	lcd_write("ADC working");
	
	sei();			// Enable Interrupts
	
	init_stepper();	// initialize stepper to home position
	mTimer(200);	// pause before entering loop

	pwmTimer(MOTOR_DUTY);	// initialize PWM on B7 with constant motor duty cycle
	PORTB &= MOTOR_OFF;		// initialize conveyor motor rotation to off
	
	ADCSRA |= _BV(ADSC);	// run first conversion of ADC (25 cycles vs 13 for regular ADC)
	while((ADCSRA & _BV(ADSC))!=0);
	ADCSRA |= _BV(ADIF);	// clear interrupt flag from first ADC
	ADC_RESULT_FLAG = 0;	// clear flag
	
	lcd_clear();
	lcd_home();
	lcd_write("FIFO: 0");
	lcd_move(0,1);
	lcd_write("ADC: ");
	
	char *temp = NULL;
	
	while(1){
		if(ADC_READ){
			ADC_READ = 0;
			ADC_MIN = 0xFFFF;
			ADCSRA |= _BV(ADSC);	// start an ADC conversion
		}
		if(FIFO_ADD_FLAG){
			FIFO_ADD_FLAG = 0;
			initLink(&newLink);		// create a new link & add to linked list
			enqueue(&head, &tail, &newLink);
			if(nextPiece == NULL){
				nextPiece = newLink;
				temp = &(nextPiece->e.itemCode);
			}
			lcd_move(6,0);
			sprintf(lcd_string, "%d", size(&head, &tail));
			lcd_write(lcd_string);
		}
		if (ADC_RESULT_FLAG){
			ADC_RESULT_FLAG = 0;
			if(ADC_RESULT < ADC_MIN){
				ADC_MIN = ADC_RESULT;
			}
			if((PIND &0x08)==0x08){
				ADCSRA |= _BV(ADSC);
			}else{
				if(ADC_MIN < STEEL_LOW){		// ALUMINUM
					*temp = ALUMINUM;
				}else if(ADC_MIN < WHITE_LOW){	// STEEL
					*temp = STEEL;
				}else if(ADC_MIN < BLACK_LOW){	// WHITE
					*temp = WHITE;
				}else{							// BLACK
					*temp = BLACK;
				}
				lcd_move(5,1);
				sprintf(lcd_string, "%4d", ADC_MIN);
				lcd_write(lcd_string);
				lcd_move(13,0);
				sprintf(lcd_string, "%d", *temp);
				lcd_write(lcd_string);
				lcd_move(13,1);
				sprintf(lcd_string, "%d", CURRENT_BIN);
				lcd_write(lcd_string);
				switch((*temp)-CURRENT_BIN){
					case 1:
					case -3:
						stepper_90_CW();
						break;
					case 2:
					case -2:
						stepper_180_CW();
						break;
					case 3:
					case -1:
						stepper_90_CCW();
						break;
					default:
						STEPPER_DONE_FLAG = 1;
						break;
				}
			}
		}
		if(EX_FLAG){
			if(STEPPER_DONE_FLAG){
				if((PINE & 0x10)==0x10){
					STEPPER_DONE_FLAG = 0;
					EX_FLAG = 0;
					nextPiece = nextPiece->next;
					temp = &(nextPiece->e.itemCode);
				}
				PORTB |= MOTOR_ON;
			}else{
				PORTB &= MOTOR_OFF;
			}
		}
	}
	
	return 0;
}

/********************************************************************
*							  INTERRUPT								*
********************************************************************/

ISR(INT0_vect){
	while((0x01 & PIND) == 0x01);			// Check the button pressed
	if((PORTB & 0b01111111) == MOTOR_ON){
		PORTB &= MOTOR_OFF;
	}else{
		PORTB |= MOTOR_ON;
	}
	while((0x01 & PIND)==0x00);				// Check the button released
}

ISR(INT1_vect){	// KILL SWITCH!
	cli();			// disable interrupts
	DDRD = 0xFF;	// disable inputs
	PORTA = 0x00;	// disable stepper
	PORTB = 0x00;	// disable dc motor
	PORTD = 0x00;	// ensure no signal is sent to inputs
	TCCR0B = 0x00;	// disable PWM timer
	
	lcd_clear();	// print message to LCD
	lcd_home();
	lcd_write("! KILL SWITCH !");
	lcd_move(0,1);
	lcd_write("!!! ENGAGED !!!");
}

ISR(INT2_vect){	// OI sensor
	FIFO_ADD_FLAG = 1;
}

ISR(INT3_vect){	// OR sensor
	ADC_MIN = 0xFFFF;
	ADC_READ = 1;
}

ISR(INT4_vect){	// EX Sensor
	EX_FLAG = 1;
}

ISR(ADC_vect){	// ADC conversion complete flag
	ADC_RESULT_FLAG = 1;
	ADC_RESULT = ADC;
}

ISR(BADISR_vect){
	// enter code for handling bad isr here !TEMP
	lcd_clear();
	lcd_home();
	lcd_write("BAD ISR");
}

/********************************************************************
*							  FUNCTIONS								*
********************************************************************/

//!!!!!!!!!!!!!!!!!
// Timer functions

 void pwmTimer(char duty){	// PWM Output to PORT B PIN 7
	TCCR0A|= _BV(WGM00);			// Set WGM bits 0 and 1 to high
	TCCR0A|= _BV(WGM01);			// which enables fast PWM mode
	
	TCCR0A|= _BV(COM0A1);
	TCCR0B|= _BV(CS01);				// sets pre-scale to 8
	OCR0A = duty;					// Sets output compare register to duty parameter value
}

void mTimer(int count){		// 16 Bit timer
	int i = 0;
	
	// Prepare registers and values for timer operation
	TCCR1B |=_BV(WGM12);			// sets timer to CTC mode
	OCR1A = 0x03e8;					// timer end value = 1000D
	TCNT1 = 0x0000;					// timer initial value
	
	/*	Poll the timer to determine when the timer has reached 0x03e8 */
	while(i<count)
	{
		if((TIFR1 & 0x02) == 0x02)
		{
			TIFR1 |=_BV(OCF1A);		// clear interrupt flag
			i++;					// increment loop counter
		}	// End if
	}	// End while
}	// End mTimer

//!!!!!!!!!!!!!!!!!
// Queue functions

void initLink(link **newLink){	// allocates memory for a new link
	*newLink = malloc(sizeof(link));
	(*newLink)->next = NULL;
}	// End initLink

void setup(link **h, link **t){	// sets head and tail of list to NULL
	*h = NULL;
	*t = NULL;
}	// End setup

void clearQueue(link **h, link **t){	// frees memory from all links in FIFO
	link *temp;	// temporary link
	
	while(*h != NULL){
		temp = *h;		// holds current head pointer
		*h=(*h)->next;	// sets head to next element
		free(temp);		// frees memory of removed element
	}	// End while
	
	*t = NULL;	// sets list tail to NULL
}

void enqueue(link **h, link **t, link **nL){	// adds an element to the list
	if(*t != NULL){		// if the list is not empty
		(*t)->next = *nL;	// set current tail to point to new Link
		*t = *nL;			// set tail of list to new Link
	}	// End if
	else{				// if the list is empty
		*h = *nL;			// set list head to new Link
		*t = *nL;			// set list tail to new Link
	}	// End else
}	// End enqueue

void dequeue(link **h, link **t, link **deQueuedLink){	// remove an element from the list
	*deQueuedLink = *h;		// pointer to current head to retain access outside scope
	
	if(*h != NULL){		// list is not empty
		*h = (*h)->next;	// set the new list head to the next element

		if(*h == NULL){
			*t = NULL;
		}
	}	// End if
}	// End dequeue

element firstValue(link **h){	// gives the element of head Link by value
	return ((*h)->e);
}

char isEmpty(link **h){	// returns true if list is empty, else false
	return (*h == NULL);
}

int size(link **h, link **t){	// returns the number of elements in the list
	link *temp;				// temporary link
	int numElements = 0;	// element counter
	
	temp = *h;				// sets temporary link to head of list
	
	while(temp != NULL){	// iterate until end of list
		numElements++;		// increment number of elements
		temp = temp->next;	// move temp to next element
	}
	
	return (numElements);	// return number of elements
}	// End size

//!!!!!!!!!!!!!!!!!
// Stepper Functions

void init_stepper(){	// Initialize the stepper motor to home position
	lcd_clear();
	lcd_home();
	lcd_write("  Stepper Init  ");
	lcd_move(0,1);
	lcd_write("   In Progress  ");
	
	while((PIND & 0b10000000) == 0b10000000){	// HE sensor = 0 when active
		CURRENT_POSITION++;		// increment stepper position
		if (CURRENT_POSITION==4){	// handle stepper pattern overflow
			CURRENT_POSITION = 0;
		}
		PORTA = POSITION[CURRENT_POSITION];	// set stepper port to position pattern
		mTimer(20);
	}
	
	CURRENT_BIN = BLACK;
	
	lcd_move(0,1);
	lcd_write("    Complete    ");
}

void stepper_90_CW(){   // accelerated stepper 90 degree CW
	int decceleration;
	
	for (int i=0; i<50; i++)
	{
		CURRENT_POSITION++;     // increment stepper position
		if (CURRENT_POSITION==4){   // handle stepper pattern overflow
			CURRENT_POSITION = 0;
		}
		PORTA = POSITION[CURRENT_POSITION]; // set stepper port to position pattern
		
		switch(i){  // handle acceleration and decceleration
			case 0:
			case 49:
			decceleration = 14;
			break;
			case 1:
			case 48:
			decceleration = 13;
			break;
			case 2:
			case 47:
			decceleration = 12;
			break;
			case 3:
			case 46:
			decceleration = 11;
			break;
			case 4:
			case 45:
			decceleration = 9;
			break;
			case 5:
			case 44:
			decceleration = 7;
			break;
			case 6:
			case 43:
			decceleration = 4;
			break;
			default:
			decceleration = 0;
			break;
		}
		
		mTimer(6+decceleration);
	}   // End for
	
	switch(CURRENT_BIN){
		case BLACK:
		CURRENT_BIN = ALUMINUM;
		break;
		case STEEL:
		CURRENT_BIN = BLACK;
		break;
		case WHITE:
		CURRENT_BIN = STEEL;
		break;
		case ALUMINUM:
		CURRENT_BIN = WHITE;
		break;
		default:
		break;
	}

	STEPPER_DONE_FLAG = 1;
}

void stepper_180_CW(){  // accelerated stepper 180 degree CW
	int decceleration;
	
	for (int i=0; i<100; i++)
	{
		CURRENT_POSITION++;     // increment stepper position
		if (CURRENT_POSITION==4){   // handle stepper pattern overflow
			CURRENT_POSITION = 0;
		}
		PORTA = POSITION[CURRENT_POSITION]; // set stepper port to position pattern
		
		switch(i){  // handle acceleration and decceleration
			case 0:
			case 99:
			decceleration = 14;
			break;
			case 1:
			case 98:
			decceleration = 13;
			break;
			case 2:
			case 97:
			decceleration = 12;
			break;
			case 3:
			case 96:
			decceleration = 11;
			break;
			case 4:
			case 95:
			decceleration = 9;
			break;
			case 5:
			case 94:
			decceleration = 7;
			break;
			case 6:
			case 93:
			decceleration = 4;
			break;
			default:
			decceleration = 0;
			break;
		}
		
		mTimer(6+decceleration);
	}   // End for
	
	switch(CURRENT_BIN){
		case BLACK:
		CURRENT_BIN = WHITE;
		break;
		case STEEL:
		CURRENT_BIN = ALUMINUM;
		break;
		case WHITE:
		CURRENT_BIN = BLACK;
		break;
		case ALUMINUM:
		CURRENT_BIN = STEEL;
		break;
		default:
		break;
	}
	
	STEPPER_DONE_FLAG = 1;
}

void stepper_90_CCW(){  // accelerated stepper 90 degree CCW
	int decceleration;
	for (int i=0; i<50; i++)
	{
		CURRENT_POSITION--;     // increment stepper position
		if (CURRENT_POSITION<0){    // handle stepper pattern overflow
			CURRENT_POSITION = 3;
		}
		PORTA = POSITION[CURRENT_POSITION]; // set stepper port to position pattern
		
		switch(i){  // handle acceleration and decceleration
			case 0:
			case 49:
				decceleration = 14;
				break;
			case 1:
			case 48:
				decceleration = 13;
				break;
			case 2:
			case 47:
				decceleration = 12;
				break;
			case 3:
			case 46:
				decceleration = 11;
				break;
			case 4:
			case 45:
				decceleration = 9;
				break;
			case 5:
			case 44:
				decceleration = 7;
				break;
			case 6:
			case 43:
				decceleration = 5;
				break;
			case 7:
			case 42:
				decceleration = 3;
				break;
			case 8:
			case 41:
				decceleration = 1;
				break;
			default:
				decceleration = 0;
				break;
		}
		
		mTimer(6+decceleration);
	}   // End for
	
	switch(CURRENT_BIN){
		case BLACK:
			CURRENT_BIN = STEEL;
			break;
		case STEEL:
			CURRENT_BIN = WHITE;
			break;
		case WHITE:
			CURRENT_BIN = ALUMINUM;
			break;
		case ALUMINUM:
			CURRENT_BIN = BLACK;
			break;
		default:
			break;
	}

	STEPPER_DONE_FLAG = 1;
}


//!!!!!!!!!!!!!!!!!
//	LCD Functions
inline void _writeNibble(char data){	// sends one nibble to the LCD
	data &= 0x0F;	// remove high nibble
	PORTC &= 0x0F;	// clear data bits
	PORTC |= E;		// set enable bit high
	mTimer(1);		// delay 1ms
	
	// Handle data bits
	if((data & 0x01) == 0x01){
		PORTC |= D4;
		}else{
		PORTC &= ~D4;
	}

	if((data & 0x02) == 0x02){
		PORTC |= D5;
		}else{
		PORTC &= ~D5;
	}

	if((data & 0x04) == 0x04){
		PORTC |= D6;
		}else{
		PORTC &= ~D6;
	}

	if((data & 0x08) == 0x08){
		PORTC |= D7;
		}else{
		PORTC &= ~D7;
	}
	
	PORTC &= ~E;	// set enable low
	mTimer(1);		// delay 1ms
}

inline void _writeByte(char data){	// sends one byte split as 2 nibbles
	_writeNibble((data & 0xF0) >> 4);	// send high nibble
	_writeNibble(data & 0x0F);			// send low nibble
}

void lcd_init(){
	DDRC |= (E|RS|D4|D5|D6|D7);	// enable LCD pins to output
	
	mTimer(100);	// wait for power to stabilize
	
	PORTC &= ~E;	// sets enable to low
	PORTC &= ~RS;	// sets register select to low (command mode)
	
	for(int i=0; i<3; i++){
		_writeNibble(0x03);	// initialize LCD to 8-bit mode
		mTimer(5);
	}
	
	_writeNibble(0x02);		// set LCD to 4-bit mode
	
	lcd_command(0b00101000);	// sets 4-bit, 2 line, 5x7 font
	lcd_command(0b00001000);	// turn display off
	lcd_command(0x01);			// clear display
	lcd_command(0b00010100);	// shift cursor, increment right
	lcd_command(0x0E);			// display on, cursor on, blink off
	lcd_home();					// send cursor to home position
}

void lcd_command(char cmd){	// sends a command to the LCD
	PORTC &= ~RS;		// use command register
	_writeByte(cmd);	// handle byte transmission to LCD
}

void lcd_write(char* string){	// outputs a string of text to lcd
	PORTC |= RS;  // set register to data output
	
	while(*string){
		_writeByte(*string++); // write current character
	}
}

void lcd_home(){	// return to home position
	lcd_command(0x02);
}

void lcd_clear(){	// clear screen
	lcd_command(0x01);
}

void lcd_move(char x, char y){  // moves the cursor to x,y position
	lcd_command(0x80 | (x+(0x40*y)));
}