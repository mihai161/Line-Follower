#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount]; 

int pwmA = 9;
int pwmB = 10;

int speed = 70;

int led = 14;

int maxValue[SensorCount];
int minValue[SensorCount];

int lastError = 0;
boolean  onoff = 0;
int val, cnt = 0, v[3];

const uint16_t threshold = 500; //  adjustable - can take values between 0 and 1000

//the speed can be between  0 and 255 - 0 is LOW and 255 is HIGH. At a high value, 
//you may risk burning  the motors, because the voltage supplied by the PWM control
//is higher than  6V.
const int maxspeeda = 180;
const int maxspeedb = 180;
const int basespeeda  = 100;
const int basespeedb = 100;
/*If your robot can't take tight curves,  you can set up the robot
  to revolve around the base (and not around one of  the wheels) by
  setting the minspeed to a negative value (-100), so the motors  will go 
  forward and backward. Doing this the motors can wear out faster. 
  If you don't want to do this, set the minspeed to 0, so the motors 
  will  only go forward.
*/
const int minspeeda = 0;
const int minspeedb =  0;


float Kp  = 0.3;
float Ki = 0;
float Kd = 3;
uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
int  P;
int I;
int D;
float Pvalue;
float Ivalue;
float Dvalue;

void setup(){
  QRT_INIT();

  Serial.begin(9600);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
}

void loop() {

  PID_control();

}

void QRT_INIT(){
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){4, 5, 6, 7, 8, A1, A2, A3}, SensorCount);
  qtr.setEmitterPin(16);

  delay(500);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 100; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(led, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    minValue[i] = qtr.calibrationOn.minimum[i];
    Serial.print(minValue[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    maxValue[i] = qtr.calibrationOn.maximum[i];
    Serial.print(maxValue[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  //delay(1000);
}

void  forward_brake(int posa, int posb) {
  analogWrite(pwmA, posa);
  analogWrite(pwmB, posb);
}

void  PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues); //read  the current position
  int error = 3500 - position; //3500 is the ideal position  (the centre)

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                       //needed to be applied to the speed
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb -  motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0)  {
    motorspeedb = 0;
  } 
  forward_brake(motorspeeda, motorspeedb);
}

// cod comentat

// #include <QTRSensors.h>  // Include biblioteca pentru a lucra cu senzorii de linie QTR

// QTRSensors qtr;  // Creează un obiect de tipul QTRSensors pentru a citi valorile senzorilor

// const uint8_t SensorCount = 8;  // Numărul de senzori de linie (în acest caz 8 senzori)
// uint16_t sensorValues[SensorCount];  // Array pentru a stoca valorile citite de la fiecare senzor

// // Definirea pinilor pentru motoare și controlul vitezei acestora
// int pwmA = 9;  // Pinul 9 pentru controlul motorului A
// int pwmB = 10; // Pinul 10 pentru controlul motorului B

// int speed = 70;  // Viteza de bază a motoarelor (valoare PWM între 0 și 255)

// int led = 14;  // Pinul 14 pentru LED-ul de pe placă (pentru a semnaliza starea de calibrare)

// int maxValue[SensorCount];  // Array pentru valorile maxime citite în timpul calibrării
// int minValue[SensorCount];  // Array pentru valorile minime citite în timpul calibrării

// int lastError = 0;  // Variabilă pentru eroarea anterioară, necesară pentru calculul PID
// boolean onoff = 0;  // Flag pentru a controla starea robotului (nu este folosit în acest cod)
// int val, cnt = 0, v[3];  // Variabile auxiliare (nu sunt folosite în acest cod)

// const uint16_t threshold = 500;  // Pragul de detectare a liniei - ajustabil între 0 și 1000

// // Definirea limitelor de viteză pentru motoare
// const int maxspeeda = 180;
// const int maxspeedb = 180;
// const int basespeeda = 100;  // Viteza de bază pentru motorul A
// const int basespeedb = 100;  // Viteza de bază pentru motorul B
// const int minspeeda = 0;     // Viteza minimă pentru motorul A (poate fi negativă pentru a permite rotirea)
// const int minspeedb = 0;     // Viteza minimă pentru motorul B

// // Coeficientii PID
// float Kp = 0.3;  // Coeficientul pentru eroarea proporțională
// float Ki = 0;    // Coeficientul pentru eroarea integrată
// float Kd = 3;    // Coeficientul pentru eroarea derivativă
// uint8_t multiP = 1;  // Multiplicator pentru coeficientul P
// uint8_t multiI = 1;  // Multiplicator pentru coeficientul I
// uint8_t multiD = 1;  // Multiplicator pentru coeficientul D
// uint8_t Kpfinal;  // Coeficientul final P
// uint8_t Kifinal;  // Coeficientul final I
// uint8_t Kdfinal;  // Coeficientul final D
// int P;  // Eroarea proporțională
// int I;  // Eroarea integrată
// int D;  // Eroarea derivativă
// float Pvalue;  // Valoarea erorii proporționale
// float Ivalue;  // Valoarea erorii integrate
// float Dvalue;  // Valoarea erorii derivate

// void setup() {
//   QRT_INIT();  // Inițializează senzorii și calibrarea lor

//   Serial.begin(9600);  // Inițializează comunicația serială pentru a vizualiza datele în monitorul serial
//   pinMode(pwmA, OUTPUT);  // Setează pinul pwmA ca ieșire pentru motorul A
//   pinMode(pwmB, OUTPUT);  // Setează pinul pwmB ca ieșire pentru motorul B
// }

// void loop() {
//   PID_control();  // Apelarea funcției PID pentru a regla viteza motoarelor pe baza erorii
// }

// // Inițializează senzorii și calibrarea acestora
// void QRT_INIT() {
//   qtr.setTypeRC();  // Setează tipul senzorilor QTR pentru a folosi circuitul RC
//   qtr.setSensorPins((const uint8_t[]){4, 5, 6, 7, 8, A1, A2, A3}, SensorCount);  // Setează pinii senzorilor
//   qtr.setEmitterPin(16);  // Setează pinul pentru emitatorul de IR (pinul 16)

//   delay(500);  // Așteaptă 500 ms înainte de a începe calibrarea

//   pinMode(led, OUTPUT);  // Setează pinul LED ca ieșire
//   digitalWrite(led, HIGH);  // Aprinde LED-ul pentru a semnaliza începutul procesului de calibrare

//   // Se calibraza senzorii citind valorile minime și maxime de 100 de ori
//   for (uint16_t i = 0; i < 100; i++) {
//     qtr.calibrate();  // Calibrează senzorii
//   }
  
//   digitalWrite(led, LOW);  // Oprește LED-ul pentru a semnaliza sfârșitul calibrării

//   // Afișează valorile minime obținute în timpul calibrării pe monitorul serial
//   Serial.begin(9600);
//   for (uint8_t i = 0; i < SensorCount; i++) {
//     minValue[i] = qtr.calibrationOn.minimum[i];  // Salvează valorile minime
//     Serial.print(minValue[i]);  // Afișează valoarea minimă pentru fiecare senzor
//     Serial.print(' ');
//   }
//   Serial.println();

//   // Afișează valorile maxime obținute în timpul calibrării pe monitorul serial
//   for (uint8_t i = 0; i < SensorCount; i++) {
//     maxValue[i] = qtr.calibrationOn.maximum[i];  // Salvează valorile maxime
//     Serial.print(maxValue[i]);  // Afișează valoarea maximă pentru fiecare senzor
//     Serial.print(' ');
//   }
//   Serial.println();
//   Serial.println();
//   //delay(1000);  // Așteaptă 1 secundă (comentat, dar poate fi activat dacă este nevoie)
// }

// // Funcția pentru a mișca motoarele înainte, aplicând frânare
// void forward_brake(int posa, int posb) {
//   analogWrite(pwmA, posa);  // Aplică semnalul PWM pe motorul A
//   analogWrite(pwmB, posb);  // Aplică semnalul PWM pe motorul B
// }

// // Funcția PID pentru controlul mișcării robotului
// void PID_control() {
//   uint16_t position = qtr.readLineBlack(sensorValues);  // Citește poziția actuală a liniei de la senzori
//   int error = 3500 - position;  // Calculează eroarea față de poziția ideală (3500 reprezintă centrul)

//   P = error;  // Eroarea proporțională
//   I = I + error;  // Eroarea integrată
//   D = error - lastError;  // Eroarea derivativă
//   lastError = error;  // Actualizează eroarea anterioară

//   // Calculează viteza corectivă pe baza erorii și a coeficientelor PID
//   int motorspeed = P * Kp + I * Ki + D * Kd;
  
//   // Ajustează viteza motoarelor pe baza corectării PID
//   int motorspeeda = basespeeda + motorspeed;
//   int motorspeedb = basespeedb - motorspeed;

//   // Asigură-te că viteza motoarelor nu depășește limitele
//   if (motorspeeda > maxspeeda) {
//     motorspeeda = maxspeeda;
//   }
//   if (motorspeedb > maxspeedb) {
//     motorspeedb = maxspeedb;
//   }
//   if (motorspeeda < 0) {
//     motorspeeda = 0;
//   }
//   if (motorspeedb < 0) {
//     motorspeedb = 0;
//   }

//   // Controlează motoarele pentru a se mișca conform vitezelor calculate
//   forward_brake(motorspeeda, motorspeedb);
// }
