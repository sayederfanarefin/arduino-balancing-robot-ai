//int motor1pin1 = 8;
//int motor1pin2 = 7;
//int motor1pwm = 6;
//
//
//int motor2pin1 = 4;
//int motor2pin2 = 3;
//int motor2pwm = 5;



int motor1pin1 = 4;
int motor1pin2 = 3;
int motor1pwm = 5;


int motor2pin1 = 8;
int motor2pin2 = 7;
int motor2pwm = 6;


void setup() {
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor1pwm, OUTPUT);
  pinMode(motor2pwm, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:   
  analogWrite(motor1pwm, 255);
  analogWrite(motor2pwm, 255);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  delay(1000);

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  delay(1000);
}
