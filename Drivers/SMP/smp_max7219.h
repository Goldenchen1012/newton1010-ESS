/**
  ******************************************************************************
  * @file    smp_max7219.h 
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2021/12/30
  * @brief   Header for smp_max7219.c module
  ******************************************************************************
  */

#ifndef __SMP_MAX7219_LIBRARY__
#define __SMP_MAX7219_LIBRARY__
#include <stdint.h>

#define MAX7219_DATA_PORT                      GPIOE->ODR                               // assume "DATA" is on 
#define MAX7219_DATA_BIT                       GPIO_PIN_13
#define MAX7219_DATA_0()                       (MAX7219_DATA_PORT &= ~MAX7219_DATA_BIT)
#define MAX7219_DATA_1()                       (MAX7219_DATA_PORT |=  MAX7219_DATA_BIT)

#define MAX7219_CLK_PORT                       GPIOE->ODR                              // assume "CLK" is on 
#define MAX7219_CLK_BIT                        GPIO_PIN_11
#define MAX7219_CLK_0()                        (MAX7219_CLK_PORT &= ~MAX7219_CLK_BIT)
#define MAX7219_CLK_1()                        (MAX7219_CLK_PORT |=  MAX7219_CLK_BIT)

#define MAX7219_CS_PORT                        GPIOE->ODR                              // assume "CS" is on 
#define MAX7219_CS_BIT                         GPIO_PIN_12
#define MAX7219_CS_0()                         (MAX7219_CS_PORT &= ~MAX7219_CS_BIT)
#define MAX7219_CS_1()                         (MAX7219_CS_PORT |=  MAX7219_CS_BIT)

// Power on or off the LED controllers
//
void maxPowerUp(uint8_t bPowerUp);
//
// Set the intensity (duty cycle of PWM signal) for the LED segments
// valid values are 0 (dimmest) to 15 (brightest)
//
void maxSetIntensity(uint8_t bIntensity);
//
// Set the segment decode mode (BCD or none)
//
void maxSetSegmentMode(uint8_t bMode);
//
// Send image data to the array of controllers
// The image data is transmitted as N by 8 lines tall (N is the number of MAX7219 controllers)
// The pitch (uint8_ts per line) can be any value
//
void maxSendImage(uint8_t *pImage, int iPitch);
//
// Enable (1) or disable (0) test mode
// This mode lights up every LED at max brightness
// It can sometimes power up in test mode
//
void maxSetTestMode(uint8_t bOn);
//
// Number of "digits/rows" to control
// valid values are 1-8 active digits/rows
//
void maxSetLimit(uint8_t bLimit);
//
// Send an ASCII string of digits to a 7-segment display
//
void maxSegmentString(char *pString);
//
// Draw a string of characters into the image buffer
// Normal characters are 8x8 and drawn on uint8_t boundaries
// Small characters are 6x8 and drawn on bit boundaries
//
void maxDrawString(char *pString, uint8_t *pImage, uint8_t iPitch, uint8_t bSmall);
//
// Scroll a bitmap N bits left (positive) or right (negative)
// Valid scroll values are +1 to +7 and -1 to -7
// A bitmap is assumed to be iPitch uint8_ts wide by 8 rows tall
// Bits which scroll off one end are added back to the other end
//
void maxScrollBitmap(uint8_t *pBitmap, int iPitch, int iScroll);
//
// Initialize the controllers
// returns 0 for success, -1 for failure
//
int maxInit(uint8_t iNum, uint8_t bDecodeMode);
//
// Turn off the LED controllers and free resources
//
void maxShutdown(void);

void MAX7219_All_Display(uint8_t data);

#endif // __MAX7219_LIBRARY__
