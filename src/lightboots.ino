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

#define NUM_LEDS 16
CRGB leds[NUM_LEDS];

#define MAX_TEMPERATURE 100
double min_temperature = 0;
double temperature = 100;
double ambient = 0;
double k = 0.5; // rate of cooling, higher is faster 

#define RED_THRESHOLD 25
#define ORANGE_THRESHOLD 50
#define YELLOW_THRESHOLD 75
#define WHITE_THRESHOLD 100
#define HOTNESS_RATIO 0.3  // how far along the strip is the hottest point of the flame

#define BRIGHTNESS 0.5
#define SENSITIVITY 0.5  // how much to react to acceleration


void setup(void)
{
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, 6>(leds, NUM_LEDS);
  for(int i=0;i<NUM_LEDS;i++)
  {
    leds[i].setRGB(0,127,0);
  }
  FastLED.show();
  delay(300);
  for(int i=0;i<NUM_LEDS;i++)
  {
    leds[i].setRGB(0,0,0);
  }
  FastLED.show();
  
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}

void loop(void)
{
  sensors_event_t linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  int total_data = abs(linearAccelData.acceleration.x) + abs(linearAccelData.acceleration.y) + abs(linearAccelData.acceleration.x);
  Serial.print(total_data);
  Serial.print(": ");

  // do some cooling
  temperature = ambient + (temperature * exp( -1 * k * BNO055_SAMPLERATE_DELAY_MS / 1000));

  // do some heating
  temperature += int(total_data * SENSITIVITY);

  // apply thresholds
  temperature = max(0, min(temperature, MAX_TEMPERATURE));

  Serial.print(temperature);
  Serial.print(": ");
  double hottest_point = temperature * HOTNESS_RATIO;
  //Serial.print(hottest_point);

  double flame_height = double(temperature);

  for(int i=0; i<NUM_LEDS; i++){
    // +1 otherwise the lowest LED is always off
    double flame_part = double(i+1) / double(NUM_LEDS+1) * 100.0;
    double led_temperature;

    if (flame_part < hottest_point)
    {
        led_temperature = flame_part / hottest_point * temperature;
        //Serial.print(" ");
        //Serial.print(led_temperature);

        //leds[i].setRGB(red_value(led_temperature),green_value(led_temperature),0);

    }
    else
    {
        led_temperature = max(temperature - ((flame_part - hottest_point) / (flame_height - hottest_point) * temperature), 0);

        //leds[i].setRGB(led_temperature,0,0);


    }
   
    leds[i].setRGB(int(red_value(led_temperature) * BRIGHTNESS),
                   int(green_value(led_temperature) * BRIGHTNESS),
                   int(blue_value(led_temperature) * BRIGHTNESS));

    Serial.print(" ");
    //Serial.print(led_temperature);

    Serial.print(blue_value(led_temperature));
  }
  Serial.println();

  FastLED.show();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}


int red_value(double temperature)
{
    //return 255;
    if (temperature > RED_THRESHOLD) return 255;
    return int(temperature / RED_THRESHOLD * 255);
}

int green_value(double temperature)
{
    if (temperature < RED_THRESHOLD) return 0;
    if (temperature > YELLOW_THRESHOLD) return 255;
    return (temperature - RED_THRESHOLD) / (YELLOW_THRESHOLD - RED_THRESHOLD) * 255;
}

int blue_value(double temperature)
{
    if (temperature < YELLOW_THRESHOLD) return 0;
    return (temperature - YELLOW_THRESHOLD) / (MAX_TEMPERATURE - YELLOW_THRESHOLD) * 255;
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
