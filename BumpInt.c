// BumpInt.c
// Runs on MSP432, interrupt version
// Provide low-level functions that interface bump switches on the robot.
// Daniel Valvano and Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Negative logic bump sensors
#define Bumper5  0b10000000;       // P4.7 Bump5, left side of robot
#define Bumper4  0b01000000;       // P4.6 Bump4
#define Bumper3  0b00100000;       // P4.5 Bump3
#define Bumper2  0b00001000;       // P4.3 Bump2
#define Bumper1  0b00000100;       // P4.2 Bump1
#define Bumper0  0b00000001;       // P4.0 Bump0, right side of robot

#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include <stdint.h>
#include <stdbool.h>
#include "msp.h"
volatile uint16_t Status = 0; // holds the value of what P4IV is when it checks which bumper was hit
volatile bool wasInterrupt = false;
volatile uint8_t direction;
void BumpInt_Init(void){
    // Initialize Bump sensors
   P4DIR &= 0b00010010;           // Make six Port 4 pins inputs, Pins  7,6,5,3,2,0
   P4REN |= 0b11101101;         // Activate interface pullup
   P4OUT |= 0b11101101;               //pull up ins
   P4IE  |= 0b11101101;            //enables interrupt on Port 4 pins 7,6,5,3,2,0
   P4IES |= 0b11101101;             // Interrupt on falling edge (on touch)
   P4IFG &= ~0b11101101;             //clears flag
   NVIC-> ISER[1] = 0x40;
   EnableInterrupts();
}

// triggered on touch, falling edge
void PORT4_IRQHandler(void){
Status = P4IV;
volatile uint8_t count = 0;
  if(Status == 0x02){                        //checks if bumper 0 was hit
     count =+3; //  increment by +3
     wasInterrupt = true;
     direction = 1;
     P4IFG &= ~0b11101101;
  }else if(Status == 0x06){                  //Checks if bumper 1 was hit
      count =+2; // increment by +2
      wasInterrupt = true;
     P4IFG &= ~0b11101101;
      direction =1;
  }else if(Status == 0x08){                   //Checks if bumper 2 was hit
      count =+1; // increment by +1
      wasInterrupt = true;
      P4IFG &= ~0b11101101;
      direction =1;
  }else if(Status == 0x0C){                //checks to see if bumper 3 was hit
      count =-1; //increments by -1
      P4IFG &= ~0b11101101;
      direction =0;
      wasInterrupt = true;
  }else if(Status == 0x0E){                 // checks to see if bumper 4 was hit
      count =-2; //increments by -2
      wasInterrupt = true;
      P4IFG &= ~0b11101101;
      direction =0;
  }else if(Status == 0x10){              //checks to see if bumper 5 was hit
      count =-3; // increment by -3
      P4IFG &= ~0b11101101;
      direction =0;
      wasInterrupt = true;
  }
   P4IFG &= ~0b11101101; //clears the flag before leaving the function
}
