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

#define RED_THRESHOLD 25
#define ORANGE_THRESHOLD 50
#define YELLOW_THRESHOLD 75
#define WHITE_THRESHOLD 100
#define HOTNESS_RATIO 0.4


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
  temperature = min(total_data+temperature, 100);

  Serial.print(temperature);
  Serial.print(" ");
  double hottest_point = temperature * HOTNESS_RATIO;
  //Serial.print(hottest_point);

  double flame_height = double(temperature);


  for(int i=0; i<NUM_LEDS; i++){
    double flame_part = double(i) / double(NUM_LEDS) * 100.0;

    if (flame_part < hottest_point)
    {
        double led_temperature = flame_part / hottest_point * temperature;
        Serial.print(" ");
        Serial.print(led_temperature);

        leds[i].setRGB(led_temperature,0,0);

    }
    else
    {
        double led_temperature = max(temperature - ((flame_part - hottest_point) / (flame_height - hottest_point) * temperature), 0);
        Serial.print(" ");
        Serial.print(led_temperature);
        leds[i].setRGB(led_temperature,0,0);


    }

  }
  Serial.println();

  FastLED.show();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}



double red_percentage(double temperature)
{
    if (temperature > RED_THRESHOLD) return 100;
    return temperature / RED_THRESHOLD * 100;
}

double orange_percentage(double temperature)
{
    if (temperature < RED_THRESHOLD) return 0;
    if (temperature > ORANGE_THRESHOLD) return 100;
    return (temperature - RED_THRESHOLD) / (ORANGE_THRESHOLD - RED_THRESHOLD) * 100;
}

double yellow_percentage(double temperature)
{
    if (temperature < ORANGE_THRESHOLD) return 0;
    if (temperature > YELLOW_THRESHOLD) return 100;
    return (temperature - ORANGE_THRESHOLD) / (YELLOW_THRESHOLD - ORANGE_THRESHOLD) * 100;
}

double white_percentage(double temperature)
{
    if (temperature < YELLOW_THRESHOLD) return 0;
    if (temperature > WHITE_THRESHOLD) return 100;
    return (temperature - YELLOW_THRESHOLD) / (WHITE_THRESHOLD - YELLOW_THRESHOLD) * 100;
}
