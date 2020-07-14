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
	char stage;
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
void dequeue(link **h, link **deQueuedLink);
element firstValue(link **h);
char isEmpty(link **h);
int size(link **h, link **t);

// Stepper
void init_stepper(void);
void stepper_CW(char);
void stepper_CCW(char);

/********************************************************************
*					 	  Global Variables							*
********************************************************************/

// Stepper variables
unsigned char POSITION[] = {0b00110110, 0b00101110, 0b00101101, 0b00110101};
volatile int CURRENT_POSITION = 0;

// FIFO Variables
volatile link *head, *tail;
volatile link *newLink, *rtnLink;

// ADC Variables
volatile int ADC_RESULT;			// int to hold a single ADC result
volatile char ADC_RESULT_FLAG;		// flag to indicate ADC conversion is complete
volatile char ADC_OUTPUT;			// flag to indicate which sensor the result is from
#define ADMUX_F0_MSK 0b11100000		// ADMUX & MSK = ADC set to PIN F0
#define ADMUX_F1_MSK 0x01			// ADMUX | MSK = ADC set to PIN F1

// DC Motor Variables
#define MOTOR_ON 0b00100000		// PORTB | ON = motor runs
#define MOTOR_OFF 0b10000000	// PORTB & OFF = motor stops
#define MOTOR_DUTY 0x80			// 50% duty cycle - !TEMP initial trial value

/********************************************************************
*							    MAIN								*
********************************************************************/

int main(void){
	TCCR1B|=_BV(CS10);		// mTimer pre-scale = 1
	cli();					// disable all interrupts
	
	// ACTIVE HIGH push button - Kill switch
	EIMSK|= _BV(INT1);		// enable INT1 - PORT D - PIN 1
	EICRA|= _BV(ISC11) |_BV(ISC10);	// rising edge interrupt
	
	// ACTIVE LOW Optical Sensor for Inductive
	EIMSK|= _BV(INT2);		// enable INT2 - PORT D - PIN 2
	EICRA|= _BV(ISC21);		// falling edge interrupt
	
	// ACTIVE LOW Optical Sensor for Reflective
	EIMSK|= _BV(INT3);		// enable INT3 - PORT D - PIN 3
	EICRA|= _BV(ISC31) |_BV(ISC30);	// falling edge interrupt

	// default ADC input Right-Adjusted
	ADCSRA|= _BV(ADEN);		// enable ADC
	ADCSRA|= _BV(ADIE);		// enable interrupt of ADC
	ADMUX|= _BV(REFS0);		// sets VREF to AVcc external
	
	// PORT I/O Configuration
	DDRA = 0xFF;	// output to stepper motor - bin
	DDRB = 0xFF;	// output to DC Motor - conveyor
	DDRC = 0xFF;	// output to LED array
	DDRD = 0x00;	// input to feed interrupts
	DDRF = 0x00;	// input to ADC converter
	
	// Initialize linked list
	newLink = NULL;			// no new link yet
	rtnLink = NULL;			// no link returned yet
	setup(&head, &tail);	// Setup FIFO
	
	sei();			// Enable Interrupts
	
	init_stepper();	// initialize stepper to home position
	mTimer(200);	// pause before entering loop

	pwmTimer(MOTOR_DUTY);	// initialize PWM on B7 with constant motor duty cycle
	PORTB |= MOTOR_ON;		// initialize conveyor motor rotation
	
	while(1){
		if(ADC_RESULT_FLAG){
			// !TEMP
		}
	}
	
	return 0;
}

/********************************************************************
*							  INTERRUPT								*
********************************************************************/

ISR(INT1_vect){	// KILL SWITCH!
	cli();			// disable interrupts
	PORTA = 0x00;	// disable stepper
	PORTB = 0x00;	// disable dc motor
	TCCR0B = 0x00;	// disable PWM timer
	while(1){		// flash LEDs to indicate kill switch active
		PORTC = 0xFF;
		mTimer(200);
		PORTC = 0x00;
		mTimer(200);
	}
}

ISR(INT2_vect){	// OI sensor
	initLink(&newLink);		// create a new link & add to linked list
	enqueue(&head, &tail, &newLink);
	PORTB &= MOTOR_OFF;		// !TEMP - test where part stops when INT2 triggers
	ADMUX &= ADMUX_F0_MSK;	// Set ADC to F0
	ADC_OUTPUT = 0x01;		// flag to indicate ADC is inductive
	ADCSRA |= _BV(ADSC);	// start a ADC conversion
}

ISR(INT3_vect){	// OR sensor - !TEMP
	dequeue(&head, &rtnLink);	// remove element from linked list
	free(rtnLink);				// free memory of element
	if(isEmpty(&head)){			// ensure that empty list is handled appropriately
		setup(&head, &tail);
	}
}

ISR(ADC_vect){	// ADC conversion complete flag
	ADC_RESULT = ADCH << 8 + ADCL;
	ADC_RESULT_FLAG = 1;
}

ISR(BADISR_vect){
	// enter code for handling bad isr here !TEMP
}

/********************************************************************
*							  FUNCTIONS								*
********************************************************************/

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

void dequeue(link **h, link **deQueuedLink){	// remove an element from the list
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

// Stepper Functions

void init_stepper(){	// Initialize the stepper motor to home position
	while((PIND & 0b10000000) == 0b10000000){	// HE sensor = 0 when active
		CURRENT_POSITION++;		// increment stepper position
		if (CURRENT_POSITION==4){	// handle stepper pattern overflow
			CURRENT_POSITION = 0;
		}
		PORTA = POSITION[CURRENT_POSITION];	// set stepper port to position pattern
		mTimer(20);
	}
	
	PORTC = 0xFF;
	mTimer(100);
	PORTC = 0x00;
}

void stepper_CW(char steps){   // accelerated stepper CW
    int decceleration;
    
    for (int i=0; i<steps; i++)
    {
        CURRENT_POSITION++;     // increment stepper position
        if (CURRENT_POSITION==4){   // handle stepper pattern overflow
            CURRENT_POSITION = 0;
        }
        PORTA = POSITION[CURRENT_POSITION]; // set stepper port to position pattern
        
        switch(i){  // handle acceleration and decceleration
            case 0:
            case steps-1:
                decceleration = 14;
                break;
            case 1:
            case steps-2:
                decceleration = 13;
                break;
            case 2:
            case steps-3:
                decceleration = 12;
                break;
            case 3:
            case steps-4:
                decceleration = 11;
                break;
            case 4:
            case steps-5:
                decceleration = 9;
                break;
            case 5:
            case steps-6:
                decceleration = 7;
                break;
            case 6:
            case steps-7:
                decceleration = 4;
                break;
            default:
                decceleration = 0;
                break;
        }
        
        mTimer(6+decceleration);
    }   // End for
}

void stepper_CCW(char steps){  // accelerated stepper CCW
    int decceleration;
    
    for (int i=0; i<steps; i++)
    {
        CURRENT_POSITION--;     // increment stepper position
        if (CURRENT_POSITION<0){    // handle stepper pattern overflow
            CURRENT_POSITION = 3;
        }
        PORTA = POSITION[CURRENT_POSITION]; // set stepper port to position pattern
        
        switch(i){  // handle acceleration and decceleration
            case 0:
            case steps-1:
                decceleration = 14;
                break;
            case 1:
            case steps-2:
                decceleration = 13;
                break;
            case 2:
            case steps-3:
                decceleration = 12;
                break;
            case 3:
            case steps-4:
                decceleration = 11;
                break;
            case 4:
            case steps-5:
                decceleration = 9;
                break;
            case 5:
            case steps-6:
                decceleration = 7;
                break;
            case 6:
            case steps-7:
                decceleration = 5;
                break;
            case 7:
            case steps-8:
                decceleration = 3;
                break;
            case 8:
            case steps-9:
                decceleration = 1;
                break;
            default:
                decceleration = 0;
                break;
        }
        
        mTimer(6+decceleration);
    }   // End for
}