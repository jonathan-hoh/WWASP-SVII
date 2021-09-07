#include <Stepper.h>



const byte ledPin = 7;
const byte interruptPin = 6;
const int stepsPerRevolution = 100;
volatile byte state = LOW;
volatile byte search = true;
int safetyStop = 100;
int counter = 0;

Stepper stepper(stepsPerRevolution, 8, 9, 11, 10);


void setup() {
  Serial.begin(9600); 
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(interruptPin), zero, CHANGE);
  stepper.setSpeed(30);
}

void loop() {
  digitalWrite(ledPin, state);
  if(search and counter < safetyStop) {
    stepper.step(-1);
    counter += 1;
    delay(10);
  }
}

void zero() {
  state = !state;
  search = false;
  Serial.println("zero point");
  state = !state;
  search = true;
  stepper.step(counter);
  Serial.println("return journey");
  
  //delay(1000);
  //stepper.step(counter-1);
}
