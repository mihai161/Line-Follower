

int pwmA = 9;
int in1A = 5;
int in2A = 16;

// Motor B

int pwmB = 10;
int in1B = 6;
int in2B = A0;

int stby = 14;
// Motor Speed Values - Start at zero

int MotorSpeed1 = 120;
int MotorSpeed2 = 40;

int citire;

int out2 = 8;
int out3 = A3;

void setup(){

  Serial.begin(9600);

  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
  pinMode(stby, OUTPUT);
  digitalWrite(stby,HIGH); 
  pinMode(out2, OUTPUT);
  pinMode(out3, OUTPUT);  

}

void loop() {
  
  Serial.print(digitalRead(out2));
  Serial.println(digitalRead(out3));

  if(digitalRead(out2) == HIGH && digitalRead(out3) == LOW){
    digitalWrite(in1A, HIGH);
    digitalWrite(in2A, LOW);
    digitalWrite(in1B, HIGH);
    digitalWrite(in2B, LOW);
  
    analogWrite(pwmA, MotorSpeed1);
    analogWrite(pwmB, MotorSpeed2);
  }

  if(digitalRead(out2) == LOW && digitalRead(out3) == HIGH){
    digitalWrite(in1A, HIGH);
    digitalWrite(in2A, LOW);
    digitalWrite(in1B, HIGH);
    digitalWrite(in2B, LOW);
  
    analogWrite(pwmA, MotorSpeed2);
    analogWrite(pwmB, MotorSpeed1);
  }
}