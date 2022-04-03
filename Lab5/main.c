//************************************/
//Code by: Matthew Cells
//Lab#5
//February 15, 2022
//***********************************/
#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include "..\inc\Motor.h"
#include "..\inc\BumpInt.h"

extern volatile bool wasInterrupt;
extern volatile uint8_t direction;
void TimerInit(void)
{
   P2SEL1 &= ~0b11000000; P2SEL0 |= 0b11000000;  //First initialize TimerA0 for PWM
   //Since the motors are connected to P2.6 and P2.7, use TimerA0, compare blocks 3 & 4
   TA0CTL &= ~0x0030;  //stop the timer
   TA0CTL |= 0x0200; TA0CTL &= ~0x100; //choose SMCLK for the clock source
   TA0CTL |= 0b0000000001000000; TA0CTL &=0b1111111101111111; //choose clock divider of 2
   TA0CCR0 = 59999; //sets the period
   TA0CCTL3 |= 0x00E0;  TA0CCTL4 |= 0x00E0; //Outmode 7: reset/set
  //Now initialize TimerAx for the delay function
  TA2CTL &= ~0x0030;//stop the timer
  TA2CTL |= 0x0200; TA2CTL &= ~0x100; //choose SMCLK for the clock source
  TA2CTL |= 0b0000000010000000;; TA2CTL &= 0b111111110111111; //choose clock divider of 4 : ID = 10
  TA2EX0 |=0b0000000000000100; TA2EX0 &= 0b1111111111111100; //choose second clock divider in TAxEX0 of 5, total divide is 20
  TA2CCR0 = 59999;
}
void Delay(void){
   TA2R=0;
   TA2CTL |= 0x0010;
   while(!(TA2CCTL0 & 0x0001)){}
   TA2CCTL0 &= ~0x0001;
   TA2CTL &= ~0x0030;
}
void main(void){
  WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
  DisableInterrupts();
  Clock_Init48MHz();
  Motor_Init();
  TimerInit();
  BumpInt_Init();
 while(1){
     enum robot_states {backward,right,left,forward} state, prevState; //declare states
      state = forward;
      prevState =!forward;
      uint16_t stateTimer = 0;
      volatile bool isNewState; //declare boolean to know if state has switched
          while(1){
            isNewState = (state != prevState);
              prevState = state;  //save state for next time
              switch(state){
              case(forward):
                  Motor_Forward(15000,15300);
              stateTimer++;
                  if(wasInterrupt == true){
                      state=backward;
                  }
                  break;
              case(backward):
                  if(isNewState){
                      stateTimer =0;
                  }
                Motor_Backward(17000,15300);
                stateTimer++;
               if(stateTimer>=10){
                if(direction == 1){
                    state = left;
                }else{
                    state =right;
                }
               }
                break;
              case(left):
                      if(isNewState){
                          stateTimer=0;
                      }
              Motor_Right(17000,15300);
              stateTimer++;
              if(stateTimer>=15){
                  wasInterrupt =false;
                  state=forward;
              }
              break;
              case(right):
                      if(isNewState){
                          stateTimer=0;
                      }
              Motor_Left(17000,15300);
              stateTimer++;
              if(stateTimer>=15){
                  wasInterrupt=false;
                  state=forward;
              }
              break;
            default:state=forward;
              }
 Delay();
  }
 }
}
