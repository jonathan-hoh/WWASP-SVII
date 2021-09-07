#include <Stepper.h>

const int DELAY_ROTATE_TO = 1000;
const int DELAY_ZERO_SEARCH = 10;

enum mode {
  none,
  zeroSearch,
  rotateTo
};


const byte ledPin = 7;
const byte interruptPin = 6;
const int stepsPerRevolution = 200;
volatile byte passedZero = false;

mode currentAction = none;
String command; 
int target = 0;
int counter = 0;

Stepper stepper(stepsPerRevolution, 8, 9, 11, 10);


void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(interruptPin), zeroPing, RISING);
  stepper.setSpeed(10);
}

void loop() {
  if(Serial.available() > 0 ) {
    command = Serial.readStringUntil('\n');
    char op = command.charAt(0);
    if(op == 'z') {
      zero();
    } else if(op == 'r') {
      rotate(command.substring(1).toInt());
    }
  } else if(currentAction == zeroSearch) {
    if(passedZero){
      Serial.print("Found zero in ");
      Serial.print(counter);
      Serial.println(" steps");
      currentAction = none;      
    } else {
      stepper.step(-1);
      counter += 1; 
      delay(DELAY_ZERO_SEARCH);
    }
  } else if (currentAction == rotateTo) {
    stepper.step(target);
    delay(DELAY_ROTATE_TO);
    currentAction = none;
    Serial.print("Rotated ");
    Serial.print(target);
    Serial.println(" steps");
  }
}

void rotate(int steps) {
  target = steps;
  currentAction = rotateTo;
}

void zero() {
  passedZero = false;
  counter = 0;
  currentAction = zeroSearch;
}

void zeroPing() {
  passedZero = true;
}
