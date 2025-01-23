int pwmA = 9;
int pwmB = 10;

int speed2 = 110;
int speed1 = 70;

int led = 14;


void setup(){

  Serial.begin(9600);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
}

void loop() {

  analogWrite(pwmA, speed2);
  analogWrite(pwmB, speed1);
  delay(2000);

  analogWrite(pwmA, speed1);
  analogWrite(pwmB, speed2);
  delay(2000);
}
