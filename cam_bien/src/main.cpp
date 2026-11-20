#include <Arduino.h>
#include "HSI.h"  
#include "ADCTouchSensor.h"


ADCTouchSensor pressure1(PA0, PA1, 50);
ADCTouchSensor pressure2(PA3, PA1, 50);
ADCTouchSensor pressure3(PA6, PA1, 50);
ADCTouchSensor pressure4(PB1, PA1, 50);

const unsigned int SAMPLES = 90; 
float Ref1 = 0, Ref2 = 0, Ref3 = 0, Ref4 = 0;

void readSensorsInterleaved(unsigned int samples, float &out1, float &out2, float &out3, float &out4) {
  long total1 = 0, total2 = 0, total3 = 0, total4 = 0;

  for (unsigned int i = 0; i < samples; i++) {
    total1 += pressure1.readRaw1();
    total2 += pressure2.readRaw1();
    total3 += pressure3.readRaw1();
    total4 += pressure4.readRaw1();
  }

  out1 = (float)total1 / ADCTOUCH_DIVIDER / samples;
  out2 = (float)total2 / ADCTOUCH_DIVIDER / samples;
  out3 = (float)total3 / ADCTOUCH_DIVIDER / samples;
  out4 = (float)total4 / ADCTOUCH_DIVIDER / samples;

  delayMicroseconds(445);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);

  float test1, test2, test3, test4;
  for (int k = 0; k < 5; k++) {
    readSensorsInterleaved(100, test1, test2, test3, test4);
    delay(50);
  }

  readSensorsInterleaved(500, Ref1, Ref2, Ref3, Ref4);
}

void loop() {
  float raw1, raw2, raw3, raw4;
  readSensorsInterleaved(SAMPLES, raw1, raw2, raw3, raw4);

  float value1 = raw1 - Ref1;
  float value2 = raw2 - Ref2;
  float value3 = raw3 - Ref3;
  float value4 = raw4 - Ref4;

  float avgForce = (value1 + value2 + value3 + value4) / 4.0f;

  Serial.println(avgForce, 2);

  delay(40);
}
