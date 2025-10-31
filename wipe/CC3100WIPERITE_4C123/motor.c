// motors.c
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// PE3-0 controls the direction of a Romi Chassis car.
// by Min He, 3/17/2024

#include "tm4c123gh6pm.h"
#include <stdbool.h>
#include <stdint.h>
#include "motor.h"
#include "GPIO.h"   // DIRECTION_E, FORWARD/BACKWARD/LEFTPIVOT/RIGHTPIVOT, UART, LEDs
#include "PWM.h"    // PWM_Init, PWM_Duty, speed constants
#include "UART0.h"
#include "SysTick.h"

// Forward declare SimpleLink cooperative task so long delays keep WiFi alive
extern void _SlNonOsMainLoopTask(void);

// Optional no-op delay used by some helpers
static inline void delay(void){ for(volatile uint32_t i=0;i<50000;i++){} }

// Initialize only direction pins (PE3-0). PWM is enabled later to avoid
// power/noise during WiFi association.
static void Motors_CoreInit(void){
  Car_Dir_Init();     // PE3-0 for direction
}

// ---------------- High-level car control merged from HW_PWM_Car ----------------
// Mode handling removed: direct-drive and patterns are always available.
// PF4 interrupt toggle removed; we keep a no-op handler for safety.
static void PF4_ModeToggle_Init(void); // now a no-op
void GPIOPortF_Handler(void);          // now a minimal no-op

// Forward declarations of pattern helpers
static void Car_Delay(double x);
static void FigureEight(void);
static void Circle(void);
static void Square(void);
static void ZigZag(void);
static void Forward(void);
static void Reverse(void);
static void Right_Wide_Turn(void);
static void Left_Wide_Turn(void);
static void Stop(void);
static void Speed_Up(void);
static void Slow_Down(void);

static volatile uint8_t s_enable_allowed = 1; // runtime lock to prevent enabling PWM
static inline void EnsurePWM(void){
  extern void MotorPWM_Enable(void);
  static uint8_t once = 0;
  if(!once && s_enable_allowed){ MotorPWM_Enable(); once = 1; }
}

void MotorControl_SetEnableAllowed(uint8_t allowed){
  s_enable_allowed = (allowed != 0) ? 1 : 0;
}

// Bit-band style helpers so we never disturb unrelated pins on Port D/E.
#define GPIOE_DATA(mask) (*((volatile uint32_t *)(0x40024000 + ((mask) << 2))))
#define GPIOD_DATA(mask) (*((volatile uint32_t *)(0x40007000 + ((mask) << 2))))

// helper to write 4-bit direction pattern safely across PE3-1 and PD2
// Pattern bits:
//   bit3 -> PE3, bit2 -> PE2, bit1 -> PE1, bit0 -> PD2 (was PE0)
static inline void Dir_Write(uint8_t pat){
  // Update PE1-PE3 (mask 0x0E) directly with bits 1..3 of pattern
  GPIOE_DATA(0x0E) = (pat & 0x0E);
  // Update PD2 from bit0 without touching PD0/PD1 PWM pins
  // PD2 mask = 0x04
  GPIOD_DATA(0x04) = (pat & 0x01) ? 0x04u : 0u;
}

void MotorControl_Init(void){
  Motors_CoreInit();
  // Start with motors idle to avoid brown-outs/noise during WiFi bring-up
  Dir_Write(BRAKE);
  // Initialize PWM hardware at startup (pre-change behavior)
  PWM_Init();
  PWM_Duty(0, 0);
  PortF_Init();          // LEDs and PF4
  PF4_ModeToggle_Init(); // no-op
  UART_OutString((uint8_t*)"WIPERITE control ready (motors idle)\r\n");
}

// Enable PWM hardware and keep duty at 0% initially
void MotorPWM_Enable(void){
  PWM_Init();
  PWM_Duty(0, 0);
}


void Car_ProcessCommand(unsigned char control_symbol){
  // Echo to debug UART0 - COMMENTED OUT to avoid blocking hang during motor activation
  // The UART_OutChar() busy-waits on TX FIFO which can hang if voltage droop from motors
  // affects UART hardware. Main loop already provides command feedback via UI.
  // UART_OutChar(control_symbol);
  // UART_OutChar(CR);
  // UART_OutChar(LF);

  // PWM hardware and pins are initialized at startup and when client connects;
  // avoid any lazy init here to ensure motor commands are strictly non-blocking.

  if(!s_enable_allowed){
    if(control_symbol=='s' || control_symbol=='H' || control_symbol=='h'){
      Stop();
    }
    return;
  }

  switch(control_symbol){
    case 'H':
    case 'h':
      Stop();
      Car_Delay(0.2);
      Stop();
      break;
    case 'S':
      // Uppercase 'S' = Square pattern (sent by client on b/B key press)
      Square();
      break;
    case 'X':
    case 'x':{
      static uint8_t s_swapped = 0;
      s_swapped ^= 1;
      PWM_SetSwapLR(s_swapped);
      UART_OutString((uint8_t*)(s_swapped?"SwapLR=ON\r\n":"SwapLR=OFF\r\n"));
      break;
    }
    case '8':
      FigureEight();
      break;
    case 'C':
    case 'c':
      Circle();
      break;
    case 'Z':
    case 'z':
      ZigZag();
      break;
    case 'F':
    case 'f':
      //Forward();
			move_forward();
      Car_Delay(0.2);
      Stop();
      break;
    case 'B':
    case 'b':
      Reverse();
      Car_Delay(0.2);
      Stop();
      break;
    case 'L':
    case 'l':
      //Left_Wide_Turn();
			pivot_left();
      Car_Delay(0.2);
      Stop();
      break;
    case 'R':
    case 'r':
      //Right_Wide_Turn();
			pivot_right();
      Car_Delay(0.2);
      Stop();
      break;
    case 'U':
    case 'u':
      /* Allow speed changes in any mode */
      Speed_Up();
      break;
    case 'D':
    case 'd':
      /* Allow speed changes in any mode */
      Slow_Down();
      break;
    default:
      break;
  }
}

// ---- PF4 mode toggle setup (complements PortF_Init) ----
static void PF4_ModeToggle_Init(void){
  // Modes removed. Leave PF4 as input with pull-up (already configured in PortF_Init).
  // Interrupts on PF4 are not used.
}

// ---- Port F ISR: toggles mode + LED color ----
void GPIOPortF_Handler(void){
  // Modes removed; if PF4 interrupt somehow fires, just clear and return
  if(GPIO_PORTF_RIS_R & 0x10){
    GPIO_PORTF_ICR_R = 0x10;  // clear flag
  }
}

// ----------------- Pattern helpers -----------------
static void FigureEight(void){
  // This pattern is too long and blocks the main loop.
  // It should be implemented as a state machine or sequence of commands from the client.
  // For now, we will just perform a short action to show it was received.
  forward_left();
  Car_Delay(0.2);
  stop_the_car();
}

static void Circle(void){
  // This pattern is too long and blocks the main loop.
  // It should be implemented as a state machine or sequence of commands from the client.
  // For now, we will just perform a short action to show it was received.
  pivot_left();
  Car_Delay(0.2);
  stop_the_car();
}

static void Square(void){
  // This pattern is too long and blocks the main loop.
  // It should be implemented as a state machine or sequence of commands from the client.
  // For now, we will just perform a short action to show it was received.
  move_forward();
  Car_Delay(0.2);
  stop_the_car();
}

static void ZigZag(void){
  // This pattern is too long and blocks the main loop.
  // It should be implemented as a state machine or sequence of commands from the client.
  // For now, we will just perform a short action to show it was received.
  move_forward();
  Car_Delay(0.2);
  pivot_right();
  Car_Delay(0.1);
  stop_the_car();
}

static void Forward(void){
  Dir_Write(FORWARD);
  PWM_Duty(SPEED_35, SPEED_35);
}

static void Reverse(void){
  Dir_Write(BACKWARD);
  PWM_Duty(SPEED_35, SPEED_35);
}

static void Right_Wide_Turn(void){
  Dir_Write(FORWARD);
  PWM_Duty(SPEED_35, SPEED_20);
}

static void Left_Wide_Turn(void){
  Dir_Write(FORWARD);
  PWM_Duty(SPEED_20, SPEED_35);
}

static void Stop(void){
  Dir_Write(BRAKE);
  PWM_Duty(0, 0);
}

static void Speed_Up(void){
  PWM_Duty(SPEED_50, SPEED_50);
}

static void Slow_Down(void){
  PWM_Duty(SPEED_10, SPEED_10);
}

static void Car_Delay(double x){
  // Cooperative delay: use SysTick 1ms ticks and service SimpleLink driver
  if (x <= 0) return;
  uint32_t ms_total = (uint32_t)(x * 1000.0);
  if (ms_total == 0) ms_total = 1;
  for(uint32_t i = 0; i < ms_total; i++){
    SysTick_Wait(T1ms);
    // Do NOT block here, just service the driver and continue
    _SlNonOsMainLoopTask();
  }
}

// The following functions are now redundant with the main command processor
// but are kept for potential direct use or testing.

// void Start_L(void) { PWM_PD0_Duty(SPEED_35); }
// void Start_R(void) { PWM_PD1_Duty(SPEED_35); }
// void Stop_L(void) { PWM_PD0_Duty(0); }
// void Stop_R(void) { PWM_PD1_Duty(0); }
// void Start_Both_Wheels(void){ PWM_Duty(SPEED_35, SPEED_35); }
// void Stop_Both_Wheels(void) { PWM_Duty(0, 0); }
// void Set_L_Speed(uint16_t duty){ PWM_PD0_Duty(duty); }
// void Set_R_Speed(uint16_t duty){ PWM_PD1_Duty(duty); }

void move_forward(void) {
  Dir_Write(FORWARD);
  PWM_Duty(SPEED_35, SPEED_35);
}

void move_backward(void) {
  Dir_Write(BACKWARD);
  PWM_Duty(SPEED_35, SPEED_35);
}

void stop_the_car(void) {
  Dir_Write(BRAKE);
  PWM_Duty(0, 0);
}

void forward_left(void) {
  Dir_Write(FORWARD);
  PWM_Duty(SPEED_10, SPEED_35);
}

void forward_right(void) {
  Dir_Write(FORWARD);
  PWM_Duty(SPEED_35, SPEED_10);
}

void backward_left(void) {
  Dir_Write(BACKWARD);
  PWM_Duty(SPEED_10, SPEED_35);
}

void backward_right(void) {
  Dir_Write(BACKWARD);
  PWM_Duty(SPEED_35, SPEED_10);
}

void pivot_left(void) {
  Dir_Write(LEFTPIVOT);
  PWM_Duty(SPEED_35, SPEED_35);
}

void pivot_right(void) {
  Dir_Write(RIGHTPIVOT);
  PWM_Duty(SPEED_35, SPEED_35);
}
