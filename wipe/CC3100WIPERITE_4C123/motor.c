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
#include "GPIO.h"   // DIRECTION_D, DIRECTION_C, FORWARD/BACKWARD/LEFTPIVOT/RIGHTPIVOT, UART, LEDs
#include "PWM.h"    // PWM_Init, PWM_Duty, PWM_PB5_Duty, PWM_PE5_Duty, speed constants
#include "UART0.h"

// Optional no-op delay used by some helpers
static inline void delay(void){ for(volatile uint32_t i=0;i<50000;i++){} }

// Initialize direction pins (PD2-0, PC6) and PWM on PE5/PB5
// Unified init
static void Motors_CoreInit(void){
  Car_Dir_Init();     // PD2-0, PC6 for direction
  PWM_Init();    // PE5/PB5 for speed PWM (M0PWM5/M0PWM3)
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
      REQUIRE_MODE(1);
      FigureEight();
      break;
    case 'C':
    case 'c':
      REQUIRE_MODE(1);
      Circle();
      break;
    case 'S':
      REQUIRE_MODE(1);
      Square();
      break;
    case 'Z':
    case 'z':
      REQUIRE_MODE(1);
      ZigZag();
      break;
    case 'F':
    case 'f':
      REQUIRE_MODE(2);
      Forward();
      break;
    case 'B':
    case 'b':
      REQUIRE_MODE(2);
      Reverse();
      break;
    case 'L':
    case 'l':
      REQUIRE_MODE(2);
      Left_Wide_Turn();
      break;
    case 'R':
    case 'r':
      REQUIRE_MODE(2);
      Right_Wide_Turn();
      break;
    case 's':
      REQUIRE_MODE(2);
      Stop();
      break;
    case 'U':
    case 'u':
      REQUIRE_MODE(2);
      Speed_Up();
      break;
    case 'D':
    case 'd':
      REQUIRE_MODE(2);
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
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_10, SPEED_35);
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheels
  Car_Delay(1.5);

  //forward right turn
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_35, SPEED_10);
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheels
  Car_Delay(1.5);

  //stop
  DIRECTION_D = (BRAKE & 0x07);
  DIRECTION_C = (BRAKE & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R &= ~0x00000028; // stop both wheels
  Car_Delay(1.5);
}

static void Circle(void){
  //foward left turn
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_35, SPEED_35);
  PWM0_ENABLE_R |= 0x00000020; // Enable right wheel
  PWM0_ENABLE_R &= ~0x00000008; // Disable left wheel
  Car_Delay(1.3);

  //stop
  DIRECTION_D = (BRAKE & 0x07);
  DIRECTION_C = (BRAKE & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R &= ~0x00000028; // stop both wheels
  Car_Delay(1);
}

static void Square(void){
  // moving forward
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // enable both wheels
  Car_Delay(1);

  // right pivot turn
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (RIGHTPIVOT & 0x07);
  DIRECTION_C = (RIGHTPIVOT & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheels
  Car_Delay(0.175);

  // moving forward
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // enable both wheels
  Car_Delay(1);

  //right pivot
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (RIGHTPIVOT & 0x07);
  DIRECTION_C = (RIGHTPIVOT & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheels
  Car_Delay(0.175);

  // moving forward
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // enable both wheels
  Car_Delay(1);

  //right pivot
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (RIGHTPIVOT & 0x07);
  DIRECTION_C = (RIGHTPIVOT & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheels
  Car_Delay(0.175);

  // moving forward
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // enable both wheels
  Car_Delay(1);

  //right pivot
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (RIGHTPIVOT & 0x07);
  DIRECTION_C = (RIGHTPIVOT & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheels
  Car_Delay(0.175);

  //stop
  DIRECTION_D = (BRAKE & 0x07);
  DIRECTION_C = (BRAKE & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R &= ~0x00000028; // stop both wheels
  Car_Delay(1);
}

static void ZigZag(void){
  // moving forward
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // enable both wheels
  Car_Delay(1);

  //right pivot
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (RIGHTPIVOT & 0x07);
  DIRECTION_C = (RIGHTPIVOT & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheels
  Car_Delay(0.175);

  // moving forward
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // enable both wheels
  Car_Delay(1);

  //left pivot
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (LEFTPIVOT & 0x07);
  DIRECTION_C = (LEFTPIVOT & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheels
  Car_Delay(0.175);

  // moving forward
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // enable both wheels
  Car_Delay(1);

  //right pivot
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (RIGHTPIVOT & 0x07);
  DIRECTION_C = (RIGHTPIVOT & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheels
  Car_Delay(0.175);

  // moving forward
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // enable both wheels
  Car_Delay(1);

  //left pivot
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (LEFTPIVOT & 0x07);
  DIRECTION_C = (LEFTPIVOT & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheels
  Car_Delay(0.175);

  // moving forward
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // enable both wheels
  Car_Delay(1);

  //stop
  DIRECTION_D = (BRAKE & 0x07);
  DIRECTION_C = (BRAKE & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R &= ~0x00000028; // stop both wheels
  Car_Delay(1);
}

static void Forward(void){
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // enable both wheels
}

static void Reverse(void){
  PWM_Duty(SPEED_35, SPEED_35);
  DIRECTION_D = (BACKWARD & 0x07);
  DIRECTION_C = (BACKWARD & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R |= 0x00000028; // enable both wheels
}

static void Right_Wide_Turn(void){
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_20, SPEED_35);
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheel
}

static void Left_Wide_Turn(void){
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_35, SPEED_20);
  PWM0_ENABLE_R |= 0x00000028; // Enable both wheel
}

static void Stop(void){
  DIRECTION_D = (BRAKE & 0x07);
  DIRECTION_C = (BRAKE & 0x08) ? 0x40 : 0x00;
  PWM0_ENABLE_R &= ~0x00000028; // stop both wheels
}

static void Speed_Up(void){
  PWM_Duty(SPEED_50, SPEED_50);
}

static void Slow_Down(void){
  PWM_Duty(SPEED_10, SPEED_10);
}

static void Car_Delay(double x){
  unsigned long volatile time;
  time = 727240*500/91 * x;  // ~0.1755 sec unit
  while(time){ time--; }
}

//enum robot_modes { INACTIVE, OBJECT_FOLLOWER, WALL_FOLLOWER };
//volatile enum robot_modes mode = INACTIVE;

// Start left wheel
void Start_L(void) { PWM0_ENABLE_R |= 0x00000008; } // PB5/M0PWM3

// Start right wheel
void Start_R(void) { PWM0_ENABLE_R |= 0x00000020; } // PE5/M0PWM5

// Stop left wheel
void Stop_L(void) { PWM0_ENABLE_R &= ~0x00000008; }

// Stop right wheel
void Stop_R(void) { PWM0_ENABLE_R &= ~0x00000020; }

void Start_Both_Wheels(void){ PWM0_ENABLE_R |= 0x00000028; }

void Stop_Both_Wheels(void) { PWM0_ENABLE_R &= ~0x00000028; }

// Set duty cycle for Left/Right Wheel using PWM module on PB5/PE5
void Set_L_Speed(uint16_t duty){ PWM_PB5_Duty(duty); }
void Set_R_Speed(uint16_t duty){ PWM_PE5_Duty(duty); }

void move_forward(void) {
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_35, SPEED_35);
  Start_Both_Wheels();
}

void move_backward(void) {
  DIRECTION_D = (BACKWARD & 0x07);
  DIRECTION_C = (BACKWARD & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_35, SPEED_35);
  Start_Both_Wheels();
}

void stop_the_car(void) {
  DIRECTION_D = (BRAKE & 0x07);
  DIRECTION_C = (BRAKE & 0x08) ? 0x40 : 0x00;
  Stop_Both_Wheels();
  PWM_Duty(1, 1);
}

void forward_left(void) {
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_10, SPEED_35);
  Start_Both_Wheels();
}

void forward_right(void) {
  DIRECTION_D = (FORWARD & 0x07);
  DIRECTION_C = (FORWARD & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_35, SPEED_10);
  Start_Both_Wheels();
}

void backward_left(void) {
  DIRECTION_D = (BACKWARD & 0x07);
  DIRECTION_C = (BACKWARD & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_10, SPEED_35);
  Start_Both_Wheels();
}

void backward_right(void) {
  DIRECTION_D = (BACKWARD & 0x07);
  DIRECTION_C = (BACKWARD & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_35, SPEED_10);
  Start_Both_Wheels();
}

void pivot_left(void) {
  DIRECTION_D = (LEFTPIVOT & 0x07);
  DIRECTION_C = (LEFTPIVOT & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_35, SPEED_35);
  Start_Both_Wheels();
}

void pivot_right(void) {
  DIRECTION_D = (RIGHTPIVOT & 0x07);
  DIRECTION_C = (RIGHTPIVOT & 0x08) ? 0x40 : 0x00;
  PWM_Duty(SPEED_35, SPEED_35);
  Start_Both_Wheels();
}
