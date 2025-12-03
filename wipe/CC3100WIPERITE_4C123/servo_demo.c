#include <stdint.h>
#include "SysTick.h"
#include "I2C.h"
#include "PCA9685.h"
#include "writing_arm.h"
#include "ST7735.h"

// Minimal demo to validate I2C1 + PCA9685 + 4 servos
// Call ServoDemo_Run() from your main after clocks are configured.

static void delay_ms(uint32_t ms){
    while(ms--){ SysTick_Wait(T1ms); }
}

void ServoDemo_Run(void){
    // Init timing and bus
    SysTick_Init();
    I2C1_Init();

    // Init PCA9685 on default address and set 50 Hz
    PCA9685_Init(PCA9685_I2C_ADDR_DEFAULT);
    PCA9685_SetPWMFreq(PCA9685_I2C_ADDR_DEFAULT, 50);

    // Use writing arm helpers (they also re-init safely)
    WritingArm_Init(PCA9685_I2C_ADDR_DEFAULT);
    WritingArm_Home();

    // Sweep base and arm a bit
    for(int a=60; a<=120; a+=15){
        WritingArm_MoveToAngles(a, 90);
        delay_ms(250);
    }
    for(int a=120; a>=60; a-=15){
        WritingArm_MoveToAngles(a, 90);
        delay_ms(250);
    }

    for(int b=70; b<=110; b+=10){
        WritingArm_MoveToAngles(90, b);
        delay_ms(250);
    }
    for(int b=110; b>=70; b-=10){
        WritingArm_MoveToAngles(90, b);
        delay_ms(250);
    }

    // Tap pen and eraser
    WritingArm_PenDown(); delay_ms(400); WritingArm_PenUp();
    delay_ms(300);
    WritingArm_EraserDown(); delay_ms(400); WritingArm_EraserUp();

    // Final small sweep on all 4 channels using helper
    WritingArm_ServoSweepTest();
}

// Probe a few PCA9685 registers and show on ST7735 for I2C sanity
void ServoDemo_ProbePCA9685(uint8_t addr){
    SysTick_Init();
    I2C1_Init();

    // Read MODE1, MODE2, PRESCALE
    uint8_t mode1 = I2C1_Receive(addr, 0x00);
    uint8_t mode2 = I2C1_Receive(addr, 0x01);
    uint8_t presc = I2C1_Receive(addr, 0xFE);

    // Read LED0_ON/LED0_OFF
    uint8_t on_l  = I2C1_Receive(addr, 0x06);
    uint8_t on_h  = I2C1_Receive(addr, 0x07);
    uint8_t off_l = I2C1_Receive(addr, 0x08);
    uint8_t off_h = I2C1_Receive(addr, 0x09);

    ST7735_FillScreen(ST7735_BLACK);
    ST7735_SetTextColor(ST7735_WHITE);
    ST7735_SetCursor(0,0);
    ST7735_OutString("PCA9685 Probe\n");
    ST7735_OutString("Addr: "); ST7735_OutUDec(addr); ST7735_OutString("\n");
    ST7735_OutString("MODE1: "); ST7735_OutUDec(mode1); ST7735_OutString("\n");
    ST7735_OutString("MODE2: "); ST7735_OutUDec(mode2); ST7735_OutString("\n");
    ST7735_OutString("PRESC: "); ST7735_OutUDec(presc); ST7735_OutString("\n");
    ST7735_OutString("LED0 ON: "); ST7735_OutUDec(on_h); ST7735_OutChar(':'); ST7735_OutUDec(on_l); ST7735_OutString("\n");
    ST7735_OutString("LED0 OFF: "); ST7735_OutUDec(off_h); ST7735_OutChar(':'); ST7735_OutUDec(off_l); ST7735_OutString("\n");
}
