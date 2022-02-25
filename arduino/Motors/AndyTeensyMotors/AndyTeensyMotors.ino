#define DIR 12
#define PWM_motor 3
//RICORDA : INVERTITI I CAVI PWM E DIR RISPETTO A FW GAVRIEL

void setup() {

  pinMode(PWM_motor, OUTPUT);
  pinMode(DIR, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(DIR,HIGH);
  analogWrite(PWM_motor, 50); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,HIGH);
  analogWrite(PWM_motor, 75); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,HIGH);
  analogWrite(PWM_motor, 100); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,HIGH);
  analogWrite(PWM_motor, 75); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,HIGH);
  analogWrite(PWM_motor, 50); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,HIGH);
  analogWrite(PWM_motor, 25); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,HIGH);
  analogWrite(PWM_motor, 0); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,LOW);
  analogWrite(PWM_motor, 25); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,LOW);
  analogWrite(PWM_motor, 50); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,LOW);
  analogWrite(PWM_motor, 75); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,LOW);
  analogWrite(PWM_motor, 100); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,LOW);
  analogWrite(PWM_motor, 75); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,LOW);
  analogWrite(PWM_motor, 50); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,LOW);
  analogWrite(PWM_motor, 25); // Send PWM signal
  delay(1000);
  digitalWrite(DIR,LOW);
  analogWrite(PWM_motor, 0); // Send PWM signal
  delay(1000);
}
