// servo_demo.c - Writing Arm Demo for WIPERITE
// Clean version with ST7735 display output only

#include <stdint.h>
#include "SysTick.h"
#include "I2C.h"
#include "PCA9685.h"
#include "writing_arm.h"
#include "ST7735.h"
#include "UART0.h"

void ServoDemo_BlindForceInit(uint8_t addr);  // ignore readback, just force init

static void delay_ms(uint32_t ms);
   
// =====================================================================
// Writing Arm Demo - Display status on ST7735 LCD
// Call ServoDemo_Run() from main after clocks are configured
// =====================================================================
// =========================================================================
// BLIND FORCE INIT - Ignores all readbacks, just writes correct values
// Use when MODE1 readback appears corrupted but writes might still work
// =========================================================================
void ServoDemo_BlindForceInit(uint8_t addr){
    UART_OutString((uint8_t*)"\r\n=== BLIND FORCE INIT (ignoring readbacks) ===\r\n");
    
    // Step 1: Put device to sleep (required to change PRESCALE)
    // Write MODE1 = 0x10 (SLEEP only, no EXTCLK, no AI)
    UART_OutString((uint8_t*)"1. Force sleep mode (write 0x10 to MODE1)...\r\n");
    I2C1_Transmit(addr, 0x00, 0x10);
    delay_ms(5);
    
    // Step 2: Set prescaler for 50Hz
    // prescale = round(25MHz / (4096 * 50Hz)) - 1 = 121 = 0x79
    UART_OutString((uint8_t*)"2. Set PRESCALE = 0x79 (50Hz)...\r\n");
    I2C1_Transmit(addr, 0xFE, 0x79);
    delay_ms(5);
    
    // Step 3: Set MODE2 for totem-pole outputs
    UART_OutString((uint8_t*)"3. Set MODE2 = 0x04 (totem-pole)...\r\n");
    I2C1_Transmit(addr, 0x01, 0x04);
    delay_ms(2);
    
    // Step 4: Clear ALL_LED_OFF to enable outputs
    UART_OutString((uint8_t*)"4. Clear ALL_LED_OFF_H (enable outputs)...\r\n");
    I2C1_Transmit(addr, 0xFD, 0x00);
    delay_ms(2);
    
    // Step 5: Wake up - clear SLEEP bit, enable auto-increment
    // Write MODE1 = 0x20 (AI only, SLEEP=0, EXTCLK=0)
    UART_OutString((uint8_t*)"5. Wake up (write 0x20 to MODE1)...\r\n");
    I2C1_Transmit(addr, 0x00, 0x20);
    delay_ms(1);  // Wait for oscillator to stabilize (500us min per datasheet)
    
    // Step 6: Set RESTART bit to resume PWM
    // Write MODE1 = 0xA0 (RESTART + AI)
    UART_OutString((uint8_t*)"6. Trigger RESTART (write 0xA0 to MODE1)...\r\n");
    I2C1_Transmit(addr, 0x00, 0xA0);
    delay_ms(10);
    
    // Step 7: Set a known PWM value on channel 0 for testing
    // 1.5ms pulse at 50Hz = 307 counts = 0x133
    UART_OutString((uint8_t*)"7. Set CH0 to 1.5ms (307 counts)...\r\n");
    uint8_t pwm_data[4] = {0x00, 0x00, 0x33, 0x01};  // ON=0, OFF=0x133
    I2C1_Burst_Transmit(addr, 0x06, pwm_data, 4);
    delay_ms(5);
    
    // Verify PWM register (this should work even if MODE1 reads are broken)
    uint8_t off_l = I2C1_Receive(addr, 0x08);
    uint8_t off_h = I2C1_Receive(addr, 0x09);
    uint16_t off_val = off_l | ((uint16_t)(off_h & 0x0F) << 8);
    UART_OutString((uint8_t*)"   LED0 OFF readback: ");
    UART_OutUDec(off_val);
    UART_OutString((uint8_t*)" (expect 307)\r\n");
    
    // Also set channel 1 with a different pulse width (2ms = 410 counts)
    UART_OutString((uint8_t*)"8. Set CH1 to 2ms (410 counts)...\r\n");
    uint8_t pwm_data2[4] = {0x00, 0x00, 0x9A, 0x01};  // ON=0, OFF=0x19A
    I2C1_Burst_Transmit(addr, 0x0A, pwm_data2, 4);
    delay_ms(5);
    
    UART_OutString((uint8_t*)"9. Init complete. Servos should now be active!\r\n");
}

static void delay_ms(uint32_t ms){
    while(ms--){ SysTick_Wait(T1ms); }
}

// Helper to output a byte as 2-digit hex on ST7735
static void ST7735_OutHex8(uint8_t val){
    const char hex[] = "0123456789ABCDEF";
    ST7735_OutChar(hex[(val >> 4) & 0x0F]);
    ST7735_OutChar(hex[val & 0x0F]);
}

// Display current status on ST7735
static void display_status(const char* msg){
    ST7735_SetCursor(0, 14);
    ST7735_OutString("                    ");  // Clear line
    ST7735_SetCursor(0, 14);
    ST7735_OutString((char*)msg);
}

// Main demo entry point
void ServoDemo_Run(void){
    // Init timing and bus
    SysTick_Init();
    I2C1_Init();
	
	  // === BLIND FORCE INIT - ignores MODE1 readback issues ===
    ServoDemo_BlindForceInit(PCA9685_I2C_ADDR_DEFAULT);
    
    // Display header
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_SetTextColor(ST7735_WHITE);
    ST7735_SetCursor(0, 0);
    ST7735_OutString("=== WIPERITE ===\n");
    ST7735_OutString("Writing Arm Demo\n\n");
    
    // Initialize Writing Arm (includes robust PCA9685 init)
    ST7735_OutString("Init PCA9685...\n");
    WritingArm_Init(PCA9685_I2C_ADDR_DEFAULT);
    delay_ms(500);
    
    // Show PCA9685 status
    uint8_t mode1 = I2C1_Receive(PCA9685_I2C_ADDR_DEFAULT, 0x00);
    uint8_t mode2 = I2C1_Receive(PCA9685_I2C_ADDR_DEFAULT, 0x01);
    uint8_t presc = I2C1_Receive(PCA9685_I2C_ADDR_DEFAULT, 0xFE);
    
    ST7735_OutString("MODE1: 0x"); ST7735_OutHex8(mode1); ST7735_OutString("\n");
    ST7735_OutString("MODE2: 0x"); ST7735_OutHex8(mode2); ST7735_OutString("\n");
    ST7735_OutString("PRESC: 0x"); ST7735_OutHex8(presc); ST7735_OutString("\n\n");
    
    // Check for common issues
    if(mode1 & 0x40){
        ST7735_SetTextColor(ST7735_RED);
        ST7735_OutString("WARN: EXTCLK set!\n");
        ST7735_OutString("Power cycle needed\n");
        ST7735_SetTextColor(ST7735_WHITE);
        delay_ms(8000);
    }
    if(mode1 & 0x10){
        ST7735_SetTextColor(ST7735_YELLOW);
        ST7735_OutString("WARN: SLEEP set\n");
        ST7735_SetTextColor(ST7735_WHITE);
    }
    
    delay_ms(8000);
    
    // Run demos
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_SetCursor(0, 0);
    ST7735_OutString("=== Servo Tests ===\n\n");
    
    // 1. Calibration test
    ST7735_OutString("1. Calibration\n");
    display_status("Sweeping servos...");
    WritingArm_Calibrate();
    delay_ms(500);
    
    // 2. Demo strokes
    ST7735_OutString("2. Demo Strokes\n");
    display_status("Drawing strokes...");
    WritingArm_DemoStrokes();
    delay_ms(500);
    
    // 3. Draw square
    ST7735_OutString("3. Draw Square\n");
    display_status("Drawing square...");
    WritingArm_DemoSquare();
    delay_ms(500);
    
    // 4. Write letters
    ST7735_OutString("4. Write Letters\n");
    display_status("Writing letters...");
    WritingArm_DemoLetters();
    delay_ms(500);
    
    // Complete
    ST7735_SetCursor(0, 12);
    ST7735_SetTextColor(ST7735_GREEN);
    ST7735_OutString("\nDemo Complete!\n");
    ST7735_SetTextColor(ST7735_WHITE);
    
    // Show final servo positions
    WritingArmState_t state = WritingArm_GetState();
    ST7735_OutString("Base: "); ST7735_OutUDec(state.base_angle); ST7735_OutString(" deg\n");
    ST7735_OutString("Arm:  "); ST7735_OutUDec(state.arm_angle); ST7735_OutString(" deg\n");
}

// Quick probe of PCA9685 registers - display on ST7735
void ServoDemo_ProbePCA9685(uint8_t addr){
    SysTick_Init();
    I2C1_Init();
    
    // Check I2C connectivity
    uint32_t mcs_status = I2C1_ProbeStatus(addr);
    
    // Read registers
    uint8_t mode1 = I2C1_Receive(addr, 0x00);
    uint8_t mode2 = I2C1_Receive(addr, 0x01);
    uint8_t presc = I2C1_Receive(addr, 0xFE);
    
    // Read LED0 registers
    uint8_t on_l  = I2C1_Receive(addr, 0x06);
    uint8_t on_h  = I2C1_Receive(addr, 0x07);
    uint8_t off_l = I2C1_Receive(addr, 0x08);
    uint8_t off_h = I2C1_Receive(addr, 0x09);
    
    // Display on ST7735
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_SetTextColor(ST7735_WHITE);
    ST7735_SetCursor(0, 0);
    ST7735_OutString("PCA9685 Probe\n\n");
    
    ST7735_OutString("Addr: 0x"); ST7735_OutHex8(addr); ST7735_OutString("\n");
    ST7735_OutString("MCS:  0x"); ST7735_OutHex8((uint8_t)mcs_status); 
    if(mcs_status & 0x02){
        ST7735_SetTextColor(ST7735_RED);
        ST7735_OutString(" NAK!");
        ST7735_SetTextColor(ST7735_WHITE);
    }
    ST7735_OutString("\n\n");
    
    ST7735_OutString("MODE1: 0x"); ST7735_OutHex8(mode1); ST7735_OutString("\n");
    ST7735_OutString("MODE2: 0x"); ST7735_OutHex8(mode2); ST7735_OutString("\n");
    ST7735_OutString("PRESC: 0x"); ST7735_OutHex8(presc); ST7735_OutString("\n\n");
    
    uint16_t on_val = on_l | ((uint16_t)(on_h & 0x0F) << 8);
    uint16_t off_val = off_l | ((uint16_t)(off_h & 0x0F) << 8);
    ST7735_OutString("LED0 ON:  "); ST7735_OutUDec(on_val); ST7735_OutString("\n");
    ST7735_OutString("LED0 OFF: "); ST7735_OutUDec(off_val); ST7735_OutString("\n");
    
    // Interpret MODE1
    ST7735_OutString("\nMODE1 flags:\n");
    if(mode1 & 0x80) ST7735_OutString(" RESTART\n");
    if(mode1 & 0x40){ 
        ST7735_SetTextColor(ST7735_RED);
        ST7735_OutString(" EXTCLK!\n"); 
        ST7735_SetTextColor(ST7735_WHITE);
    }
    if(mode1 & 0x20) ST7735_OutString(" AI\n");
    if(mode1 & 0x10){ 
        ST7735_SetTextColor(ST7735_YELLOW);
        ST7735_OutString(" SLEEP\n");
        ST7735_SetTextColor(ST7735_WHITE);
    }
}
