/* Button_LED.h
 * Initialize Port F button (PF4) and RGB LED (PF1-PF3).
 */
#ifndef BUTTON_LED_H_
#define BUTTON_LED_H_

#include <stdint.h>

void ButtonLED_Init(void);
void ButtonLED_SetColor(uint8_t red, uint8_t green, uint8_t blue);
uint8_t ButtonLED_ButtonPressed(void);
void ButtonLED_ClearButton(void);

#endif // BUTTON_LED_H_
