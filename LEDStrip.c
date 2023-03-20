/*
 * LEDStrip.c
 * Created by: Luis Estevez & Brandon Pragasa
 * Created on: 5/24/22
 */
#include "LEDStrip.h"
#include "main.h"
uint8_t LED_ARRAY[LEDCOUNT][DATA_COUNT];

/*
 * initialize the LED Strip
 */
void clearArray(void)
{
    int LED, DATA;
    for (LED = 0; LED < LEDCOUNT; LED++)
    {
        LED_ARRAY[LED][GLOBAL] = UPPER_BITS; // sets the GLOBAL DATA bits
        for (DATA = 1; DATA < DATA_COUNT; DATA++)
        {
            LED_ARRAY[LED][DATA] = CLEAR_BITS; // clears RGB DATA bits
        }
    }
}

/*
 * Sets LED Data in array
 */
void setLED(uint8_t index, uint8_t brightness, uint8_t blue, uint8_t green,
            uint8_t red)
{
    // need the first 3 bits to be 111
    LED_ARRAY[index][GLOBAL] = (UPPER_BITS | brightness);
    LED_ARRAY[index][RED] = red;
    LED_ARRAY[index][GREEN] = green;
    LED_ARRAY[index][BLUE] = blue;
}

/*
 * Send array DATA to the strip
 */
void updateStrip(void)
{
    int i, LED, DATA;
    uint8_t sFrame = CLEAR_BITS;
    uint8_t eFrame = MAX_BITS;
    /* SEND START FRAME */
    for (i = 0; i < DATA_COUNT; i++)
    {
        HAL_SPI_Transmit(&hspi1, (uint8_t *)&sFrame, BYTE_SIZE, TIMEOUT);
    }
    /* SEND LED DATA */
    for (LED = 0; LED < LEDCOUNT; LED++)
    {
        for (DATA = 0; DATA < DATA_COUNT; DATA++)
        {
            HAL_SPI_Transmit(&hspi1, (uint8_t *)&LED_ARRAY[LED][DATA],
                             BYTE_SIZE, TIMEOUT);
        }
    }
    /* SEND END FRAME */
    for (i = 0; i < DATA_COUNT; i++)
    {
        HAL_SPI_Transmit(&hspi1, (uint8_t *)&eFrame, BYTE_SIZE,
                         TIMEOUT);
    }
}

void leftRightEffect(int height, uint8_t blue, uint8_t green, uint8_t red)
{
    int i, j;
    j = CENTER;
    for (i = CENTER; i <= CENTER + height; i++)
    {
        setLED(i, BRIGHTNESS, blue, green, red);
        setLED(j, BRIGHTNESS, blue, green, red);
        updateStrip();
        if (height < MIN_HEIGHT)
        {
            HAL_Delay(30);
        }
        else
        {
            HAL_Delay(10);
        }
        j--;
    }
    j = CENTER - height;
    // affects bar width
    for (i = (CENTER + height); i != MAX_HEIGHT + 1; i--)
    {
        setLED(i, 0, 0, 0, 0);
        setLED(j, 0, 0, 0, 0);
        updateStrip();
        if (height < MIN_HEIGHT)
        {
            HAL_Delay(20);
        }
        else
        {
            HAL_Delay(5);
        }
        j++;
    }
}

/*
 * Effect that turns on leds from right to left
 */
void turnOnLEDS_RIGHT(int height, uint8_t blue, uint8_t green, uint8_t red)
{
    int i;
    for (i = 0; i <= height; i++)
    {
        setLED(i, BRIGHTNESS, blue, green, red);
        updateStrip();
        if (height < MIN_HEIGHT)
            HAL_Delay(20);
        else
            HAL_Delay(2);
    }
    for (i = height; i != -1; i--)
    {
        setLED(i, 0, 0, 0, 0);
        updateStrip();
        if ((height * 2) > MAX_HEIGHT)
            HAL_Delay(2);
        else
            HAL_Delay(10);
    }
}

/*
 * Effect that turns on leds from left to right
 */
void turnOnLEDS_LEFT(int height, uint8_t blue, uint8_t green, uint8_t red)
{
    int i;
    for (i = LEDCOUNT - 1; i >= LEDCOUNT - 1 - height; i--)
    {
        setLED(i, BRIGHTNESS, blue, green, red);
        updateStrip();
        if (height < MIN_HEIGHT)
            HAL_Delay(20);
        else
            HAL_Delay(2);
    }
    for (i = LEDCOUNT - 1 - height; i < LEDCOUNT; i++)
    {
        setLED(i, 0, 0, 0, 0);
        updateStrip();
        if ((height * 2) > MAX_HEIGHT)
            HAL_Delay(2);
        else
            HAL_Delay(10);
    }
}

/*
 * Uses the dB value from the microphone to pick a color associated with a frequency range
 * Displays the current effect
 */
void sound_detect(int max_val)
{
    uint16_t R = 0, B = 0, G = 0, height;
    switch (max_val)
    {
    case BLUE_MIN_FREQ ... BLUE_MAX_FREQ:
        height = BLUE_HEIGHT; // BLUE
        B = BLUE_B;
        G = BLUE_G;
        R = BLUE_R;
        break;
    case L_BLUE_MIN_FREQ ... L_BLUE_MAX_FREQ: // LIGHT BLUE
        height = L_BLUE_HEIGHT;
        B = L_BLUE_B;
        G = L_BLUE_G;
        R = L_BLUE_R;
        break;
    case TURQUOISE_MIN_FREQ ... TURQUOISE_MAX_FREQ: // TURQUOISE
        height = TURQUOISE_HEIGHT;
        B = TURQUOISE_B;
        G = TURQUOISE_G;
        R = TURQUOISE_R;
        break;
    case GREEN_MIN_FREQ ... GREEN_MAX_FREQ: // GREEN
        height = GREEN_HEIGHT;
        B = GREEN_B;
        G = GREEN_G;
        R = GREEN_R;
        break;
    case L_GREEN_MIN_FREQ ... L_GREEN_MAX_FREQ: // LIGHT GREEN
        height = L_GREEN_HEIGHT;
        B = L_GREEN_B;
        G = L_GREEN_G;
        R = L_GREEN_R;
        break;
    case YELLOW_MIN_FREQ ... YELLOW_MAX_FREQ: // YELLOW
        height = YELLOW_HEIGHT;
        B = YELLOW_B;
        G = YELLOW_G;
        R = YELLOW_R;
        break;
    case ORANGE_MIN_FREQ ... ORANGE_MAX_FREQ: // ORANGE
        height = ORANGE_HEIGHT;
        B = ORANGE_B;
        G = ORANGE_G;
        R = ORANGE_R;
        break;
    case RED_MIN_FREQ ... RED_MAX_FREQ: // RED
        height = RED_HEIGHT;
        B = RED_B;
        G = RED_G;
        R = RED_R;
        break;
    default: // PINK
        height = PINK_HEIGHT;
        R = PINK_R;
        G = PINK_G;
        B = PINK_B;
        break;
    }
    clearArray();
    updateStrip();
    switch (effect)
    {
    case (0):
        leftRightEffect(height, B, G, R);
        break;
    case (1):
        turnOnLEDS_LEFT(height * 2, B, G, R);
        break;
    case (2):
        turnOnLEDS_RIGHT(height * 2, B, G, R);
        break;
    default:
        break;
    }
}