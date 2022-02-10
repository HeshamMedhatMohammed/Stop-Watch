/*
 * stop_watch.c
 *
 *  Created on: ??‏/??‏/????
 *      Author: tahermedhat
 */

#define F_CPU 1000000          // Microcontroller with frequency 1Mhz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// hour1 hour0:min1 min0:sec1 sec0
int sec0 = 0;
int sec1 = 0;
int min0 = 0;
int min1 = 0;
int hour0 = 0;
int hour1 = 0;



ISR(TIMER1_COMPA_vect)
{
	sec0++; //increases one every second
}


void Timer1_CTC_Delay_sec_Init(void)
{
	TCNT1 = 0;		/* Set timer1 initial count to zero */
	OCR1A = 15625;    /* Set the Compare value to 15625 ((64/1 MHz) * 15625 = 1 sec) */
	TIMSK |= (1<<OCIE1A); /* Enable Timer1 Compare A Interrupt */

	/* Configure timer control register TCCR1A
	 * 1. Disconnect OC1A and OC1B  COM1A1=0 COM1A0=0 COM1B0=0 COM1B1=0
	 * 2. FOC1A=1 FOC1B=1 NO PWM
	 * 3. CTC Mode WGM10=0 WGM11=0 (Mode Number 4)
	 */
	TCCR1A = (1<<FOC1A) | (1<<FOC1B);

	/* Configure timer control register TCCR1B
	 * 1. CTC Mode WGM12=1 WGM13=0 (Mode Number 4)
	 * 2. Prescaler = F_CPU/64 CS10=1 CS11=1 CS12=0
	 */
	TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);
}

ISR(INT0_vect)
{
	// Reset
	hour1 = 0;
	hour0 = 0;
	min1 = 0;
	min0 = 0;
	sec1 = 0;
	sec0 = 0;
}

void INT0_Init(void)
{
	SREG  &= ~(1<<7);                   // Disable interrupts by clearing I-bit
	DDRD  &= (~(1<<PD2));               // Configure INT0/PD2 as input pin
	PORTD |= (1<<PD2);                  // enable internal push up
	GICR  |= (1<<INT0);                 // Enable external interrupt pin INT0
	MCUCR |= (1<<ISC01);			   // Trigger INT0 with the falling edge
	MCUCR &= ~(1<<ISC00);
	SREG  |= (1<<7);                    // Enable interrupts by setting I-bit
}

ISR(INT1_vect)
{
    // close the clock -> pause
	TCCR1B &= ~(1<<CS11);
	TCCR1B &= ~(1<<CS10);
}

void INT1_Init(void)
{
	SREG  &= ~(1<<7);      // Disable interrupts by clearing I-bit
	DDRD  &= (~(1<<PD3));  // Configure INT1/PD3 as input pin
	GICR  |= (1<<INT1);    // Enable external interrupt pin INT1
	MCUCR |= (1<<ISC11);   // Trigger INT1 with the raising edge
	MCUCR |= (1<<ISC10);
	SREG  |= (1<<7);       // Enable interrupts by setting I-bit
}

ISR(INT2_vect)
{
	// return the clock -> Resume
	TCCR1B |= (1<<CS11);
	TCCR1B |= (1<<CS10);
}


void INT2_Init(void)
{
	SREG   &= ~(1<<7);       // Disable interrupts by clearing I-bit
	DDRB   &= (~(1<<PB2));   // Configure INT2/PB2 as input pin
	PORTB  |= (1<<PB2);      // enable internal push up
	GICR   |= (1<<INT2);	 // Enable external interrupt pin INT2
	MCUCSR &= ~(1<<ISC2);     // Trigger INT2 with the falling edge
	SREG   |= (1<<7);        // Enable interrupts by setting I-bit
}

int main(void)
{
	SREG |= (1<<7);       // Enable interrupts by setting I-bit
	INT2_Init();
	INT1_Init();
	INT0_Init();
	Timer1_CTC_Delay_sec_Init();
	DDRA = 0x3F;         // set enables of the display as output pins
	PORTA = 0xFF;        // initial value is disable all
	DDRC = 0x0F;		// set input of the decoder as output pins
	PORTC = 0x00;		// display 00:00:00

	while (1)
	{
		if (sec0 == 10) // 00:00:09 -> 00:00:10
		{
			if(sec1 == 5)  // 00:00:59 -> 00:01:00
			{
				if (min0 == 9)  // 00:09:59 -> 00:10:00
				{
					if(min1 == 5)  // 00:59:59 -> 01:00:00
					{
						if (hour0 == 9)  // 09:59:59 -> 10:00:00
						{
							if (hour1 == 9) // 99:59:59 -> 00:00:00 -> overflow (** this stop watch is up to 100 hours**)
							{
								hour1 = 0;
								hour0 = 0;
								min1 = 0;
								min0 = 0;
								sec1 = 0;
								sec0 = 0;
							}
							else
							{
								hour1++;
								hour0 = 0;
								min1 = 0;
								min0 = 0;
								sec1 = 0;
								sec0 = 0;
							}
						}
						else
						{
							hour0++;
							min1 = 0;
							min0 = 0;
							sec1 = 0;
							sec0 = 0;
						}
					}
					else
					{
						min1++;
						min0 = 0;
						sec1 = 0;
						sec0 = 0;
					}
				}
				else
				{
					min0++;
					sec1 = 0;
					sec0 = 0;
				}

			}
			else
			{
				sec1++;
				sec0 = 0;
			}
		}
		else
		{
			// enable the 7-segement which will display and disable the rest
			PORTA = 0x01;
			PORTC = sec0;
			_delay_us(100);
			PORTA = 0x02;
			PORTC = sec1;
			_delay_us(100);
			PORTA = 0x04;
			PORTC = min0;
			_delay_us(100);
			PORTA = 0x08;
			PORTC = min1;
			_delay_us(100);
			PORTA = 0x10;
			PORTC = hour0;
			_delay_us(100);
			PORTA = 0x20;
			PORTC = hour1;
			_delay_us(100);
		}

	}
}
