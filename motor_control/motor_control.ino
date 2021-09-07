/*
 Created April 22, 2021
 by the Young Sauce Boss Jonathan Hoh and Jess Berkheimer

*/ 

#include <Stepper.h>

const int stepsPerRevolution = 200; 

// Put how many degrees per cycle you want the motor to turn

int deg_per_step = 1.8;
int step_size = deg_per_step/(1.8);


// create a value to count up to make sure the code runs properly

int check = 1;

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

void setup() {
  // put your setup code here, to run once:
  myStepper.setSpeed(69);

  Serial.begin(9600);
  Serial.println("Welcome to the cool kid club, enjoy your stay");
  
  

}

void loop() {
  // put your main code here, to run repeatedly:
  
Serial.println("Your motor is still working");  
myStepper.step(step_size);
check += 1;

delay(10000);
}
