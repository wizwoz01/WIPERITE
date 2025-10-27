// motors.c
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// PE3-0 controls the direction of a Romi Chassis car.
// by Min He, 3/17/2024

// Keep includes minimal and align with HW_PWM_Car usage
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

// Initialize direction pins (PE3-0) and PWM on PD0/PD1
// Unified init
static void Motors_CoreInit(void){
  Car_Dir_Init();     // PE3-0 for direction
  PWM_Init();         // PD0/PD1 for speed PWM
}

// ---------------- High-level car control merged from HW_PWM_Car ----------------
volatile unsigned char g_mode = 1; // Mode 1 (patterns) vs 2 (direct drive)

// Require-mode macro
#define REQUIRE_MODE(M) do{ \
  if(g_mode != (M)){ \
    UART_OutString((uint8_t*)"Blocked by current mode\r\n"); \
    break; \
  } \
}while(0)

static void PF4_ModeToggle_Init(void);
void GPIOPortF_Handler(void);   // ISR must match vector table name

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

void MotorControl_Init(void){
  Motors_CoreInit();
  PWM_Duty(SPEED_35, SPEED_35);
  PortF_Init();          // LEDs and PF4
  PF4_ModeToggle_Init(); // optional mode toggle on PF4
  UART_OutString((uint8_t*)"WIPERITE control ready\r\n");
}


void Car_ProcessCommand(unsigned char control_symbol){
  // Echo to debug UART0
  UART_OutChar(control_symbol);
  UART_OutChar(CR);
  UART_OutChar(LF);

  switch(control_symbol){
    case '8':
      FigureEight();
      break;
    case 'C':
    case 'c':
      Circle();
      break;
    case 'S':
      Square();
      break;
    case 'Z':
    case 'z':
      ZigZag();
      break;
    case 'F':
    case 'f':
      Forward();
      break;
    case 'B':
    case 'b':
      Reverse();
      break;
    case 'L':
    case 'l':
      Left_Wide_Turn();
      break;
    case 'R':
    case 'r':
      Right_Wide_Turn();
      break;
    case 's':
      /* Stop should always work */
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
  volatile unsigned long delayReg;
  // Ensure Port F clock on (safe even if already on)
  SYSCTL_RCGC2_R |= 0x20;
  delayReg = SYSCTL_RCGC2_R; (void)delayReg;

  GPIO_PORTF_IS_R  &= ~0x10;  // edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;  // single edge
  GPIO_PORTF_IEV_R &= ~0x10;  // falling edge (press)
  GPIO_PORTF_ICR_R  =  0x10;  // clear any prior flag
  GPIO_PORTF_IM_R  |=  0x10;  // arm PF4

  NVIC_EN0_R |= 1 << 30;      // NVIC enable for Port F (IRQ 30)
}

// ---- Port F ISR: toggles mode + LED color ----
void GPIOPortF_Handler(void){
  if(GPIO_PORTF_RIS_R & 0x10){        // PF4?
    GPIO_PORTF_ICR_R = 0x10;          // clear flag
    Car_Delay(0.10);                  // debounce

    if((GPIO_PORTF_DATA_R & 0x10) == 0){
      g_mode = (g_mode == 1) ? 2 : 1;

      // LED color indicator: PF3=Green, PF2=Blue
      if(g_mode == 1){
        GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | 0x08;  // Green
      }else{
        GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | 0x04;  // Blue
      }

      while((GPIO_PORTF_DATA_R & 0x10) == 0){} // wait release
      Car_Delay(0.05);
    }
  }
}

// ----------------- Pattern helpers -----------------
static void FigureEight(void){
  //forward left turn
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_10, SPEED_35);
  Car_Delay(1.5);

  //forward right turn
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_10);
  Car_Delay(1.5);

  //stop
  DIRECTION_E = BRAKE;
  PWM_Duty(0, 0);
  Car_Delay(1.5);
}

static void Circle(void){
  //foward left turn
  DIRECTION_E = FORWARD;
  PWM_Duty(0, SPEED_35); // Left wheel stopped, right wheel forward
  Car_Delay(1.3);

  //stop
  DIRECTION_E = BRAKE;
  PWM_Duty(0, 0);
  Car_Delay(1);
}

static void Square(void){
  // moving forward
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(1);

  // right pivot turn
  DIRECTION_E = RIGHTPIVOT;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(0.175);

  // moving forward
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(1);

  //right pivot
  DIRECTION_E = RIGHTPIVOT;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(0.175);

  // moving forward
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(1);

  //right pivot
  DIRECTION_E = RIGHTPIVOT;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(0.175);

  // moving forward
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(1);

  //right pivot
  DIRECTION_E = RIGHTPIVOT;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(0.175);

  //stop
  DIRECTION_E = BRAKE;
  PWM_Duty(0, 0);
  Car_Delay(1);
}

static void ZigZag(void){
  // moving forward
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(1);

  //right pivot
  DIRECTION_E = RIGHTPIVOT;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(0.175);

  // moving forward
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(1);

  //left pivot
  DIRECTION_E = LEFTPIVOT;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(0.175);

  // moving forward
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(1);

  //right pivot
  DIRECTION_E = RIGHTPIVOT;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(0.175);

  // moving forward
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(1);

  //left pivot
  DIRECTION_E = LEFTPIVOT;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(0.175);

  // moving forward
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_35);
  Car_Delay(1);

  //stop
  DIRECTION_E = BRAKE;
  PWM_Duty(0, 0);
  Car_Delay(1);
}

static void Forward(void){
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_35);
}

static void Reverse(void){
  DIRECTION_E = BACKWARD;
  PWM_Duty(SPEED_35, SPEED_35);
}

static void Right_Wide_Turn(void){
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_20);
}

static void Left_Wide_Turn(void){
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_20, SPEED_35);
}

static void Stop(void){
  DIRECTION_E = BRAKE;
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
  while (ms_total--) {
    SysTick_Wait(T1ms);
    _SlNonOsMainLoopTask();
  }
}

// The following functions are now redundant with the main command processor
// but are kept for potential direct use or testing.

void Start_L(void) { PWM_PD0_Duty(SPEED_35); }
void Start_R(void) { PWM_PD1_Duty(SPEED_35); }
void Stop_L(void) { PWM_PD0_Duty(0); }
void Stop_R(void) { PWM_PD1_Duty(0); }
void Start_Both_Wheels(void){ PWM_Duty(SPEED_35, SPEED_35); }
void Stop_Both_Wheels(void) { PWM_Duty(0, 0); }
void Set_L_Speed(uint16_t duty){ PWM_PD0_Duty(duty); }
void Set_R_Speed(uint16_t duty){ PWM_PD1_Duty(duty); }

void move_forward(void) {
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_35);
}

void move_backward(void) {
  DIRECTION_E = BACKWARD;
  PWM_Duty(SPEED_35, SPEED_35);
}

void stop_the_car(void) {
  DIRECTION_E = BRAKE;
  PWM_Duty(0, 0);
}

void forward_left(void) {
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_10, SPEED_35);
}

void forward_right(void) {
  DIRECTION_E = FORWARD;
  PWM_Duty(SPEED_35, SPEED_10);
}

void backward_left(void) {
  DIRECTION_E = BACKWARD;
  PWM_Duty(SPEED_10, SPEED_35);
}

void backward_right(void) {
  DIRECTION_E = BACKWARD;
  PWM_Duty(SPEED_35, SPEED_10);
}

void pivot_left(void) {
  DIRECTION_E = LEFTPIVOT;
  PWM_Duty(SPEED_35, SPEED_35);
}

void pivot_right(void) {
  DIRECTION_E = RIGHTPIVOT;
  PWM_Duty(SPEED_35, SPEED_35);
}
