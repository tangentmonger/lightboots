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

// todo: there are bugs that assume temperature is always 0-100
#define MAX_TEMPERATURE 100
#define MIN_TEMPERATURE 0
double temperature = MAX_TEMPERATURE;
double ambient = MIN_TEMPERATURE;
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

    flash_green();

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

    // just add up all the linear acceleration we're seeing
    int total_data = abs(linearAccelData.acceleration.x) + abs(linearAccelData.acceleration.y) + abs(linearAccelData.acceleration.x);
    Serial.print(total_data);
    Serial.print(": ");

    // do some cooling
    temperature = ambient + (temperature * exp( -1 * k * BNO055_SAMPLERATE_DELAY_MS / 1000));

    // do some heating
    temperature += int(total_data * SENSITIVITY);

    // apply thresholds
    temperature = max(MIN_TEMPERATURE, min(temperature, MAX_TEMPERATURE));

    Serial.print(temperature);
    Serial.print(": ");

    // "hottest point" depends on temperature, it's higher as it's hotter
    double hottest_point = temperature * HOTNESS_RATIO;

    double flame_height = double(temperature);

    for(int i=0; i<NUM_LEDS; i++){
        // calculate the temperature of this point in the flame
        // +1 otherwise the lowest LED is always off
        double flame_part = double(i+1) / double(NUM_LEDS+1) * 100.0;
        double led_temperature;

        if (flame_part < hottest_point)
        {
            led_temperature = flame_part / hottest_point * temperature;
        }
        else
        {
            led_temperature = temperature - ((flame_part - hottest_point) / (flame_height - hottest_point) * temperature);
        }

        led_temperature = max(MIN_TEMPERATURE, min(led_temperature, MAX_TEMPERATURE));

        // colour the LED based on the temperature of this point in the flame
        leds[i].setRGB(int(red_value(led_temperature) * BRIGHTNESS),
                int(green_value(led_temperature) * BRIGHTNESS),
                int(blue_value(led_temperature) * BRIGHTNESS));

        Serial.print(" ");
        Serial.print(blue_value(led_temperature));
    }

    Serial.println();

    FastLED.show();

    delay(BNO055_SAMPLERATE_DELAY_MS);
}


void flash_green()
{
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
}

int red_value(double temperature)
{
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
    return (temperature - YELLOW_THRESHOLD) / (WHITE_THRESHOLD - YELLOW_THRESHOLD) * 255;
}

