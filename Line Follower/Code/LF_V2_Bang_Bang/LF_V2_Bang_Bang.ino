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

void setup(){
  QRT_INIT();

  Serial.begin(9600);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
}

void loop() {

  uint16_t position = qtr.readLineBlack(sensorValues);
  
  if(position > 5000){
    analogWrite(pwmA, 0);
    analogWrite(pwmB, speed);

  } else if(position < 2000){
    analogWrite(pwmA, speed);
    analogWrite(pwmB, 0);
  }

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
  for (uint16_t i = 0; i < 200; i++)
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
  delay(1000);
}



// COD COMENTAT

// #include <QTRSensors.h>  // Include biblioteca pentru a lucra cu senzori de linie QTR

// QTRSensors qtr;  // Creează un obiect de tipul QTRSensors pentru a citi valorile senzorilor

// const uint8_t SensorCount = 8;  // Numărul de senzori de linie (în acest caz 8 senzori)
// uint16_t sensorValues[SensorCount];  // Array pentru a stoca valorile citite de la fiecare senzor

// int pwmA = 9;  // Pinul 9 pentru controlul motorului A
// int pwmB = 10; // Pinul 10 pentru controlul motorului B

// int speed = 70;  // Viteza la care vor rula motoarele (valoare PWM între 0 și 255)

// int led = 14;  // Pinul 14 pentru LED-ul de pe placă (folosit pentru a semnaliza calibrările)

// int maxValue[SensorCount];  // Array pentru valorile maxime citite în timpul calibrării
// int minValue[SensorCount];  // Array pentru valorile minime citite în timpul calibrării

// void setup() {
//   QRT_INIT();  // Inițializează senzorii QTR și calibrarea acestora

//   Serial.begin(9600);  // Inițializează comunicarea serială pentru a vizualiza datele în monitorul serial
//   pinMode(pwmA, OUTPUT);  // Setează pinul pwmA ca ieșire pentru motorul A
//   pinMode(pwmB, OUTPUT);  // Setează pinul pwmB ca ieșire pentru motorul B
// }

// void loop() {
//   // Citește linia neagră de la senzorii QTR (returnează o valoare care indică poziția liniei)
//   uint16_t position = qtr.readLineBlack(sensorValues);

//   // Dacă valoarea citită depășește un anumit prag, oprește motorul A și mișcă motorul B
//   if(position > 5000){
//     analogWrite(pwmA, 0);  // Oprește motorul A
//     analogWrite(pwmB, speed);  // Rotește motorul B la viteza setată
//   }
//   // Dacă valoarea citită este sub un alt prag, oprește motorul B și mișcă motorul A
//   else if(position < 2000){
//     analogWrite(pwmA, speed);  // Rotește motorul A la viteza setată
//     analogWrite(pwmB, 0);  // Oprește motorul B
//   }
// }

// void QRT_INIT() {
//   // Setează tipul senzorilor QTR și numărul de senzori conectați
//   qtr.setTypeRC();  // Folosește tipul de senzor RC
//   qtr.setSensorPins((const uint8_t[]){4, 5, 6, 7, 8, A1, A2, A3}, SensorCount);  // Setează pinii senzorilor
//   qtr.setEmitterPin(16);  // Setează pinul pentru emitatorul de IR (pinul 16)

//   delay(500);  // Așteaptă 500 ms înainte de a începe calibrarea

//   pinMode(led, OUTPUT);  // Setează pinul LED ca ieșire
//   digitalWrite(led, HIGH);  // Aprinde LED-ul pentru a semnaliza începutul calibrării

//   // Se calibraza senzorii citind valorile minime și maxime de 200 de ori
//   for (uint16_t i = 0; i < 200; i++) {
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
//   delay(1000);  // Așteaptă 1 secundă pentru a finaliza procesul de calibrare
// }
