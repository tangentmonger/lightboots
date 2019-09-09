#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdlib.h>
#include <math.h>
#include "FastLED.h"

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// BNO055 accelerometer module
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#define NUM_LEDS 10
CRGB leds[NUM_LEDS];

double max_temperature = 100;
double min_temperature = 0;
double temperature = 100;
double ambient = 0;
double k = 0.5; // rate of cooling 


void setup(void)
{
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, 6>(leds, NUM_LEDS);
  
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // delay(1000);
}

void loop(void)
{
  sensors_event_t linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  int total_data = linearAccelData.acceleration.x + linearAccelData.acceleration.y + linearAccelData.acceleration.x;

  // do some cooling
  temperature = ambient + (temperature * exp( -1 * k * BNO055_SAMPLERATE_DELAY_MS / 1000));

  // do some heating
  temperature += total_data;

  Serial.print(temperature);
  Serial.print(" ");
  Serial.print(linearAccelData.acceleration.x);
  Serial.print(" ");
  Serial.print(linearAccelData.acceleration.y);
  Serial.print(" ");
  Serial.print(linearAccelData.acceleration.z);
  Serial.println();

  for(int i=0; i<NUM_LEDS; i++){
    if(i<=abs(int(temperature/10))){


        leds[i].setRGB(temperature,0,0);
    } 
    else
    {
        leds[i].setRGB(0,0,0);
    }
  }
  FastLED.show();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

