#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int led = 14;

void setup()
{
  Serial.begin(9600);
  
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){4, 5, 6, 7, 8, A1, A2, A3}, SensorCount);
  qtr.setEmitterPin(16);

  delay(500);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  Serial.println("start calibration");
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(led, LOW); // turn off Arduino's LED to indicate we are through with calibration
  Serial.println("end calibration");
  // print the calibration minimum values measured when emitters were on
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  Serial.println(position);

  //delay(250);

}
