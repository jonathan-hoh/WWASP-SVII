#include <Stepper.h>
const int stepsPerRevolution = 200;
int counter = 0;
Stepper stepper(stepsPerRevolution, 8, 9, 11, 10);

void setup() {
  // put your setup code here, to run once:
  stepper.setSpeed(42);
}

void loop() {
  // put your main code here, to run repeatedly:
  stepper.step(-1);
  counter += 1;
  delay(10);
}
