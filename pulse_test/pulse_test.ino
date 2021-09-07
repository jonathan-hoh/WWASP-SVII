int pulsepin = 12;

void setup() {
  // put your setup code here, to run once:
  pinMode(pulsepin, INPUT); //set digital pin 12 to input
  Serial.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(digitalRead(pulsepin));
}
