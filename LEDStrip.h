/*
 * LEDStrip.h
 * Created by: Luis Estevez & Brandon Pragasa
 * Created on: 5/24/22
 */
#ifndef SRC_LEDSTRIP_H_
#define SRC_LEDSTRIP_H_
#include "main.h"

#define LEDCOUNT 60
#define DATA_COUNT 4
#define TIMEOUT 0xFFFF
#define BYTE_SIZE 1
#define UPPER_BITS 0xE0
#define CLEAR_BITS 0x00
#define MAX_BITS 0xFF
#define CENTER 29
#define MIN_HEIGHT 5
#define MAX_HEIGHT 30
#define BRIGHTNESS 31

// COLORS
#define BLUE_MIN_FREQ 1400
#define BLUE_MAX_FREQ 1450
#define BLUE_HEIGHT 7
#define BLUE_R 0
#define BLUE_G 0
#define BLUE_B 255
#define L_BLUE_MIN_FREQ 1451
#define L_BLUE_MAX_FREQ 1550
#define L_BLUE_HEIGHT 9
#define L_BLUE_R 0
#define L_BLUE_G 255
#define L_BLUE_B 255
#define TURQUOISE_MIN_FREQ 1551
#define TURQUOISE_MAX_FREQ 1650
#define TURQUOISE_HEIGHT 11
#define TURQUOISE_R 0
#define TURQUOISE_G 255
#define TURQUOISE_B 196
#define GREEN_MIN_FREQ 1651
#define GREEN_MAX_FREQ 1699
#define GREEN_HEIGHT 13
#define GREEN_R 0
#define GREEN_G 255
#define GREEN_B 0
#define L_GREEN_MIN_FREQ 1700
#define L_GREEN_MAX_FREQ 1799
#define L_GREEN_HEIGHT 15
#define L_GREEN_R 128
#define L_GREEN_G 255
#define L_GREEN_B 0
#define YELLOW_MIN_FREQ 1800
#define YELLOW_MAX_FREQ 1849
#define YELLOW_HEIGHT 17
#define YELLOW_R 255
#define YELLOW_G 255
#define YELLOW_B 0
#define ORANGE_MIN_FREQ 1850
#define ORANGE_MAX_FREQ 1899
#define ORANGE_HEIGHT 20
#define ORANGE_R 255
#define ORANGE_G 125
#define ORANGE_B 0
#define RED_MIN_FREQ 1900
#define RED_MAX_FREQ 1990
#define RED_HEIGHT 25
#define RED_R 255
#define RED_G 0
#define RED_B 0
#define PINK_HEIGHT 3
#define PINK_R 127
#define PINK_G 0
#define PINK_B 127
#define GLOBAL 0
#define RED 3
#define GREEN 2
#define BLUE 1

extern SPI_HandleTypeDef hspi1;
extern uint16_t effect;

void clearArray(void); // Initializes the array for the LED Strip
void setLED(uint8_t index, uint8_t brightness, uint8_t blue, uint8_t green,
            uint8_t red);
void updateStrip(void);
void leftRightEffect(int height, uint8_t blue, uint8_t green, uint8_t red);
void turnOnLEDS_RIGHT(int height, uint8_t blue, uint8_t green, uint8_t red);
void turnOnLEDS_LEFT(int height, uint8_t blue, uint8_t green, uint8_t red);
void scrollEffect(uint8_t brightness, uint8_t blue, uint8_t green, uint8_t red);
void sound_detect(int max_val);

#endif /* SRC_LEDSTRIP_H_ */
