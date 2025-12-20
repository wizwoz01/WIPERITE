/* Button_LED.c
 * Port F button (PF4) and RGB LED (PF1-PF3) support.
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Button_LED.h"

// Non-blocking debounce driven by a millisecond tick
// provided by the main loop. Declare the external millisecond counter.
extern volatile uint32_t System_ms;

volatile uint8_t g_buttonPressed = 0;
static volatile uint32_t g_last_button_ms = 0;
#define BUTTON_DEBOUNCE_MS 50

void ButtonLED_Init(void){
    // enable clock for Port F
    SYSCTL_RCGCGPIO_R |= 0x20; // port F
    // small delay
    volatile int i;
    for(i=0;i<100;i++);

    // PF1-PF3 outputs (LEDs), PF4 input (button)
    GPIO_PORTF_DIR_R |= 0x0E; // PF1,PF2,PF3 outputs
    GPIO_PORTF_DIR_R &= ~0x10; // PF4 input

    GPIO_PORTF_DEN_R |= 0x1E; // enable PF1-4 digital
    GPIO_PORTF_DATA_R &= ~0x0E; // turn LEDs off

    // pull-up on PF4 (button active low)
    GPIO_PORTF_PUR_R |= 0x10;

    // Configure interrupt on PF4 for falling edge (press)
    GPIO_PORTF_IS_R &= ~0x10;    // edge-sensitive
    GPIO_PORTF_IBE_R &= ~0x10;   // not both edges
    GPIO_PORTF_IEV_R &= ~0x10;   // falling edge
    GPIO_PORTF_ICR_R = 0x10;     // clear any prior
    GPIO_PORTF_IM_R |= 0x10;     // unmask interrupt

    // Enable NVIC for Port F (IRQ 30)
    NVIC_EN0_R = (1<<30);
}

void ButtonLED_SetColor(uint8_t red, uint8_t green, uint8_t blue){
    uint8_t mask = 0;
    if(red) mask |= 0x02;   // PF1
    if(blue) mask |= 0x04;  // PF2
    if(green) mask |= 0x08; // PF3
    GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | (mask & 0x0E);
}

uint8_t ButtonLED_ButtonPressed(void){
    if(!g_buttonPressed) return 0;
    // Button press was signalled by ISR; ensure debounce interval passed
    if((System_ms - g_last_button_ms) >= BUTTON_DEBOUNCE_MS){
        // further confirm the pin is still low (pressed)
        if((GPIO_PORTF_DATA_R & 0x10) == 0){
            // consume the event (caller must clear with ButtonLED_ClearButton())
            return 1;
        }
    }
    // not confirmed yet
    return 0;
}

void ButtonLED_ClearButton(void){
    g_buttonPressed = 0;
}

// Interrupt handler referenced by startup vector (weak symbol)
void GPIOPortF_Handler(void){
    // clear interrupt for PF4
    GPIO_PORTF_ICR_R = 0x10;
    // record event time and set flag; main provides System_ms tick
    g_last_button_ms = System_ms;
    g_buttonPressed = 1;
}
