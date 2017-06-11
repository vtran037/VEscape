/*
 Vincent Tran
 CS120B Spring 2017
 */ 

#include <avr/io.h>
#include "scheduler.h"
#include "usart.h"
#include "joystick.h"
#include "timer.h"
#include "queue.h"

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

#define STAGES 8

enum ROLES {BLANK, WIN, WALL, PLAYER, DEATH};
enum ROLES Levels[STAGES][8][16] =
{{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 2, 0, 0, 2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3},
{0, 0, 2, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 2, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 2, 2, 2, 2, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 2, 0, 0, 2, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 2, 0, 0, 2, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 2, 0, 0, 2, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0}},

{{0, 0, 0, 2, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0},
{0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 1, 4, 0, 0, 0, 0, 2, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2}},

{{0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3},
{0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0},
{0, 0, 0, 0, 4, 0, 1, 0, 4, 0, 0, 2, 0, 0, 0, 0},
{0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 2, 0, 4}},

{{0, 2, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3},
{2, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 2, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 2, 0},
{0, 0, 2, 0, 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0}, //
{2, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 2},
{0, 4, 1, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0},
{0, 2, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0},
{0, 0, 0, 4, 0, 2, 0, 0, 0, 0, 0, 2, 4, 2, 0, 0}},

{{0, 2, 0, 2, 0, 4, 0, 2, 0, 0, 0, 0, 2, 0, 0, 3},
{0, 0, 0, 2, 2, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0},
{2, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2, 0, 0, 0, 0},
{0, 2, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0},
{1, 0, 0, 4, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2},
{0, 0, 0, 2, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0},
{2, 0, 0, 0, 2, 0, 0, 4, 0, 0, 0, 0, 2, 0, 0, 0},
{0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2, 0}},

{{0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 2, 2, 2, 2, 3},
{0, 0, 2, 0, 2, 0, 0, 0, 0, 2, 2, 0, 0, 0, 2, 0},
{0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 2, 0, 2, 0, 2, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 2, 0},
{2, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 2, 0, 2, 0},
{0, 2, 0, 0, 0, 2, 0, 0, 0, 2, 2, 0, 2, 0, 2, 0},
{0, 0, 2, 2, 2, 0, 1, 2, 0, 0, 0, 0, 2, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2}},

{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},


{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},};


task MotorTask, joyTask, action, stage, head;


enum MotorStates {Middle, thumbsUp, thumbsDown};
int MotorTick(int state) {
	switch(state) {
		case Middle:
			//MotorMiddle();
			break;
		case thumbsUp:
			//MotorThumbsUp();
			break;
		case thumbsDown:
			//MotorThumbsDown();
			break;
	}
	
	return state;
}

signed char currentLevel = -1;
signed char nextLevel = 0;	

enum HeadStates {HeadExecute, HeadLoss, HeadFinish};
int HeadTick(int state) { //Head of game, controls everything
	unsigned char tempA = ~PINA;
	switch(state) {
		case HeadExecute:
			if(tempA & 0x04) { //soft reset
				MotorMiddle();
				nextLevel = currentLevel;
			}
			else if(tempA & 0x08) { //hard reset
				currentLevel = -1;
				nextLevel = 0;
			}
			break;
		case HeadLoss:
			joyTask.running = 0; 
			action.running = 0;
			if(tempA & 0x04) {
				MotorMiddle();
				state = HeadExecute;
				nextLevel = currentLevel;
			}
			break;
		case HeadFinish:
			joyTask.running = 0;
			action.running = 0;
			if(tempA & 0x04) {
				MotorMiddle();
				state = HeadExecute;
				currentLevel = (currentLevel + 1) % STAGES;
				nextLevel = currentLevel;
			}
			break;
	}
	return state;
}

unsigned char nextBound(unsigned char *xBound, unsigned char *yBound) {
	(*yBound)++;
	if((*yBound) % 16 == 0) {
		*yBound = (*yBound) % 16; 
		(*xBound)++;
	}
	
	if(*xBound > 0 && (*xBound) % 8 == 0) {
		return 0;
	}
	
	return 1;
}

Queue moveStack; 

enum MoveStates {actionStart, actionWait, actionMove};
enum stageStates {stageWait, stageLoad};

unsigned char playerPosition[2] = {0, 0};
unsigned char xBound;
unsigned char yBound;

int StageTick(int state) { //State machine for stages
	switch(state) {
		case stageWait:
			if(nextLevel != -1) {
				xBound = 0;
				yBound = 0;
				playerPosition[0] = 0;
				playerPosition[1] = 0;
				head.running = 0;
				joyTask.running = 0;
				action.running = 0;
				
				QueueMakeEmpty(moveStack);
				state = stageLoad;
				
				while(!USART_IsSendReady(0));
				USART_Send(0x00, 0);
				while(!USART_HasTransmitted(0));
				asm("nop"); 
			}
			break;
			
		case stageLoad:
			while(Levels[nextLevel][xBound][yBound] == BLANK && nextBound(&xBound, &yBound));
			if(xBound > 0 && xBound % 8 == 0) {
				head.running = 1;
				action.running = 1;
				joyTask.running = 1;
				action.state = actionStart;
				currentLevel = nextLevel;
				nextLevel = -1;
				state = stageWait;
				break;
			}
			
			while(!USART_IsSendReady(0));
			USART_Send((Levels[nextLevel][xBound][yBound] << 5) | 0x02, 0);
			while(!USART_HasTransmitted(0));
			asm("nop");
			
			while(!USART_IsSendReady(0));
			USART_Send(yBound * 2, 0);
			while(!USART_HasTransmitted(0));
			asm("nop");
			
			while(!USART_IsSendReady(0));
			USART_Send(xBound * 2, 0);
			while(!USART_HasTransmitted(0));
			asm("nop");
			
			if(Levels[nextLevel][xBound][yBound] == PLAYER) {
				playerPosition[0] = xBound;
				playerPosition[1] = yBound;
			}
			
			nextBound(&xBound, &yBound);
	}
	return state;
}
//* Thank you Cody Simmons and Stanley Cohen for joystick.h :)
enum JoyStates {input, output};	
int JoyTick(int state) {
	switch(state) {
		case input:
			if(joyPos != None) {
				QueueEnqueue(moveStack, joyPos);
				state = output;
			}
			break;
			
		case output:
			if(joyPos == None) {
				state = input;
			}
			break;
	}
	return state;
}
//* Thank you Cody Simmons and Stanley Cohen for joystick.h :)

signed char actionMoves[2] = {0, 0};
enum Positions Controls;
int ActionTick(int state) { 
	switch(state) {
		case actionStart:
			actionMoves[0] = 0;
			actionMoves[1] = 0;
			QueueMakeEmpty(moveStack);
			state = actionWait;
			break;
		case actionWait:
			if(!QueueIsEmpty(moveStack)) {
				state = actionMove;
				Controls = QueueDequeue(moveStack);
				
				if(Controls == Up) {		//					 Up
					actionMoves[0] = 0;		//				   [0][-1]
					actionMoves[1] = -1;		//					  ^
					}					//					  |
				else if(Controls == Down) {	//	Left [1][0] <- [0][0] -> [-1][0] Right
					actionMoves[0] = 0;		//					  |
					actionMoves[1] = 1;		//					  v
					}					//					[0][1]
				else if(Controls == Left) {  //					 Down
					actionMoves[0] = 1; 
					actionMoves[1] = 0;
					}
				else if(Controls == Right) {
					actionMoves[0] = -1; 
					actionMoves[1] = 0;
				}
			}
			break;
			
		case actionMove:
			;signed char newPosition[2] = {playerPosition[0] + actionMoves[0], playerPosition[1] + actionMoves[1]}; //temp vector to check where moved
			enum ROLES nextMove = Levels[currentLevel][newPosition[0]][newPosition[1]]; // compares temp vector location to object on our map(level).
				if(nextMove == WALL || nextMove == DEATH) {
					state = actionWait;
					if( ((actionMoves[1] == 1 || actionMoves[1] == -1 || actionMoves[0] == 1 || actionMoves[0] == -1) && (nextMove == DEATH)) ) {
						head.state = HeadLoss;
						
						while(!USART_IsSendReady(0));
						USART_Send(0x04, 0);
						while(!USART_HasTransmitted(0));
						asm("nop"); 
						
						while(!USART_IsSendReady(0));
						USART_Send(2 * playerPosition[1], 0);
						while(!USART_HasTransmitted(0));
						asm("nop");
						
						while(!USART_IsSendReady(0));
						USART_Send(2 * playerPosition[0], 0);
						while(!USART_HasTransmitted(0));
						asm("nop");
						
						MotorThumbsDown();
					}
						break;
				}
				
			while(!USART_IsSendReady(0));
			USART_Send((BLANK << 5) | 0x02, 0);
			while(!USART_HasTransmitted(0));
			asm("nop");
			
			while(!USART_IsSendReady(0));
			USART_Send(playerPosition[1] * 2, 0);
			while(!USART_HasTransmitted(0));
			asm("nop");
			
			while(!USART_IsSendReady(0));
			USART_Send(playerPosition[0] * 2, 0);
			while(!USART_HasTransmitted(0));
			asm("nop");
			
			playerPosition[0] = newPosition[0]; //updates new position
			playerPosition[1] = newPosition[1]; // updates new position
			
			while(!USART_IsSendReady(0));
			USART_Send((PLAYER << 5) | 0x02, 0);
			while(!USART_HasTransmitted(0));
			asm("nop");
			
			while(!USART_IsSendReady(0));
			USART_Send(playerPosition[1] * 2, 0);
			while(!USART_HasTransmitted(0));
			asm("nop");
			
			while(!USART_IsSendReady(0));
			USART_Send(playerPosition[0] * 2, 0);
			while(!USART_HasTransmitted(0));
			asm("nop");
			
				if(playerPosition[0] < 0 || playerPosition[0] >= 8 || playerPosition[1] < 0 || playerPosition[1] >= 16) { //if they hit out of bounds, they die
					head.state = HeadLoss;
					
					while(!USART_IsSendReady(0));
					USART_Send(0x04, 0);
					while(!USART_HasTransmitted(0));
					asm("nop"); 
					
					while(!USART_IsSendReady(0));
					USART_Send(2 * (playerPosition[1] - actionMoves[1]), 0);
					while(!USART_HasTransmitted(0));
					asm("nop"); 
					
					while(!USART_IsSendReady(0));
					USART_Send(2 * (playerPosition[0] - actionMoves[0]), 0);
					while(!USART_HasTransmitted(0));
					asm("nop"); 
					
					MotorThumbsDown();
					
					state = actionWait;
					} 
				else if(Levels[currentLevel][playerPosition[0]][playerPosition[1]] == WIN) { //They won!
					head.state = HeadFinish;
					
					while(!USART_IsSendReady(0));
					USART_Send(0x05, 0);
					while(!USART_HasTransmitted(0));
					asm("nop"); 
					
					while(!USART_IsSendReady(0));
					USART_Send(2 * playerPosition[1], 0);
					while(!USART_HasTransmitted(0));
					asm("nop"); 
					
					while(!USART_IsSendReady(0));
					USART_Send(2 * playerPosition[0], 0);
					while(!USART_HasTransmitted(0));
					asm("nop"); 
					
					MotorThumbsUp();
					
					state = actionWait;
					}
			break;
	}
	return state;
};


int main(void)
{
	DDRA = 0x00; 
	PORTA = 0xFF;
	
	DDRD = 0xFF;
	PORTD = 0x00;
	
	#define num_tasks 4
	
	MotorTask = (task){Middle, 25, 25, &MotorTick, 0};
		
	head = (task){HeadExecute, 25, 25, &HeadTick, 0};
	joyTask = (task){input, 25, 25, &JoyTick, 0};
	stage = (task){stageWait, 50, 50, &StageTick, 1};	
	action = (task){actionWait, 100, 100, &ActionTick, 0};
		
	task *tasks[num_tasks] = {&joyTask, &action, &stage, &head};
	unsigned short period = 1;
	
	TimerSet(period);
	TimerOn();
	
	initUSART(0);
	USART_Flush(0);
	
	joystickInit();
	moveStack = QueueInit(10);
	
    while (1) {
		joystickSample(); 
		for(int i=0; i<num_tasks; i++) {
			if(!tasks[i]->running) continue;
			if(tasks[i]->elapsedTime >= tasks[i]->period) {
				tasks[i]->elapsedTime = 0;
				tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
			}
			tasks[i]->elapsedTime += period;
		}
		while(!TimerFlag);
		TimerFlag = 0;
    }
}

