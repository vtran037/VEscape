#ifndef MOTOR_H
#define MOTOR_H
#include <avr/io.h>

void MotorThumbsDown(){
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);
	
	//Initial Time Interrupts
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted Send_Motor
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST Send_Motor)
	ICR1=4999;  //fSend_Motor=50Hz
	
	DDRD|=(1<<PD4)|(1<<PD5);   //Send_Motor Pins as Output
	
	ADMUX=0x00;// ADC input channel set to PA0
	ADCSRA |= (1<<ADSC); // Start conversion
	while (ADCSRA & (1<<ADSC)); // Wait for conversion to complete
	
	OCR1A = 80;
}

void MotorThumbsUp(){
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);
	
	//Initial Time Interrupts
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted Send_Motor
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST Send_Motor)
	ICR1=4999;  //fSend_Motor=50Hz
	
	DDRD|=(1<<PD4)|(1<<PD5);   //Send_Motor Pins as Output
	
	ADMUX=0x00;// ADC input channel set to PA0
	ADCSRA |= (1<<ADSC); // Start conversion
	while (ADCSRA & (1<<ADSC)); // Wait for conversion to complete
	
	OCR1A = 500;
}

void MotorMiddle(){
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);
	
	//Initial Time Interrupts
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted Send_Motor
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST Send_Motor)
	ICR1=4999;  //fSend_Motor=50Hz
	
	DDRD|=(1<<PD4)|(1<<PD5);   //Send_Motor Pins as Output
	
	ADMUX=0x00;// ADC input channel set to PA0
	ADCSRA |= (1<<ADSC); // Start conversion
	while (ADCSRA & (1<<ADSC)); // Wait for conversion to complete
	
	OCR1A = 180;
}

#endif // MOTOR_H

