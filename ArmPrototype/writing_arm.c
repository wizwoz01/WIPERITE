/*
* writing_arm.c
*
* Writing arm controller using PCA9685 servo driver.
* Supports kinematics for a 2-DOF arm + pen/eraser linear servos,
* plus a tiny 5x5 font for drawing text.
*
* Author: Ricardo C. 12-17-2025
* Team 9
*/
#include <stdint.h>
#include "I2C.h"
#include "SysTick.h"
#include "PCA9685.h"
#include "writing_arm.h"
#include "UART0.h"
#include <math.h>

// Some compilers/standard libraries don't define M_PI by default
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Writing arm controller using PCA9685 at 50Hz
// 4 channels total: base, arm, pen (linear), eraser (linear)
#ifndef WRITING_ARM_PCA_ADDR
#define WRITING_ARM_PCA_ADDR PCA9685_I2C_ADDR_DEFAULT
#endif

#define SERVO_BASE_CH      0
#define SERVO_ARM_CH       1
#define SERVO_PEN_CH       2  // linear pen
#define SERVO_ERASER_CH    3  // linear eraser

// IMPORTANT: these are *servo angles* (0..180).
// Tune using your UART stepping commands in main.c:
//   W/S = BASE_CH (0), A/D = ARM_CH (1), J/K = PEN_CH (2)

#define BASE_LEFT_ANGLE     60
#define BASE_RIGHT_ANGLE    120
#define ARM_TOP_ANGLE       0
#define ARM_BOTTOM_ANGLE    180

#define PEN_UP_ANGLE        180
#define PEN_DOWN_ANGLE      50

// Flip these if letters are mirrored/upside-down:
#define BASE_INVERT         0
#define ARM_INVERT          0

// Per-servo min/max pulse width (microseconds). Tune as needed
static uint16_t s_min_us[4] = {500, 500, 600, 600};
static uint16_t s_max_us[4] = {2500, 2500, 2400, 2400};

static uint8_t s_addr = WRITING_ARM_PCA_ADDR;

// Runtime state
static int s_pen_down = 0;      // 0 = up, 1 = down
// Left-handed mode mirrors X (base) so writing looks like a left arm.
// s_left_handed controls glyph mirroring and user preference for a left arm.
// Page-space mirroring is controlled separately via `s_mirror_page` so grid
// coordinates can remain natural: (0,0) = paper top-left by default.
static int s_left_handed = 1;   // 0 = right-handed, 1 = left-handed (glyph mirroring)
static int s_mirror_page = 0;   // 0 = page coords normal (top-left origin), 1 = mirror X


// Physical dimensions (user-provided): base link and forearm lengths
#define INCH_TO_MM 25.4f
#define ARM_L1_IN 6.0f   // base link length in inches (user)
#define ARM_L2_IN 5.5f   // forearm length in inches (user)
static const float ARM_L1_MM = ARM_L1_IN * INCH_TO_MM;
static const float ARM_L2_MM = ARM_L2_IN * INCH_TO_MM;

// Paper dimensions: 8.5 x 11" in LANDSCAPE (width = 11in, height = 8.5in)
#define PAGE_WIDTH_IN  11.0f
#define PAGE_HEIGHT_IN 8.5f
#define PAGE_WIDTH_MM  (PAGE_WIDTH_IN  * INCH_TO_MM)
#define PAGE_HEIGHT_MM (PAGE_HEIGHT_IN * INCH_TO_MM)
// Optional margins on the left/top edges (mm). Keep 0.0f for exact corner alignment.
#define PAGE_MARGIN_LEFT_MM 4.0f
#define PAGE_MARGIN_TOP_MM  4.0f

// Per-servo maximum angle in degrees (allows arm to exceed 180 deg)
static const int s_max_angle_deg[4] = {180, 180, 180, 180};

// Track last commanded joint angles so we can interpolate safely.
static int s_cur_base = 90;
static int s_cur_arm  = 0;
// Track current grid position corresponding to last commanded angles.
static int s_cur_gx = 0;
static int s_cur_gy = 0;

// Forward declarations for helpers defined later in the file
static void anglesFromGrid(int gx, int gy, int *out_base, int *out_arm);
static void gridFromAngles(int base, int arm, int *out_gx, int *out_gy);

// Prototypes for kinematics helpers (defined later). Added to avoid
// implicit function declarations when these are called earlier.
static void fkFromAngles(int base_deg, int arm_deg, float *out_x_mm, float *out_y_mm);
static int ikInverse(float x, float y, int *out_base_deg, int *out_arm_deg);

// Forward declaration for internal glyph draw helper used by WriteBuffer
void WritingArm_DrawCharAt(char c, int x0, int y0);

// Helper: map a grid cell to paper XY (mm) using the same logic as
// anglesFromGrid (including mirroring and autoscaling).
static void gridToXY(int gx, int gy, float *out_x_mm, float *out_y_mm);

// Text buffer (queue) used to remember input strings so they can be
// later written starting at the paper top-left and proceeding to
// bottom-right across lines.
#define TEXT_BUF_MAX 1024
static char s_text_buf[TEXT_BUF_MAX];
static int  s_text_len = 0;

// Public API to manipulate the text buffer and write its contents
void WritingArm_ClearTextBuffer(void);
int  WritingArm_QueueText(const char *s);
void WritingArm_WriteBuffer(void);

// Diagnostic: dump mapping samples over UART (corners + center)
void WritingArm_DumpMappingSamples(void);

static void delay_ms(uint32_t ms){
		while(ms--){
				SysTick_Wait(T1ms);
		}
}

// Text buffer & diagnostics implementations moved later in the file to
// ensure GRID_* and FONT_* macros are defined before they are used.
void WritingArm_Init(uint8_t pca_addr){
		s_addr = pca_addr;
		SysTick_Init();
		I2C1_Init();
		PCA9685_Init(s_addr);
		PCA9685_SetPWMFreq(s_addr, 50);
}

void WritingArm_SetServoUs(uint8_t channel, uint16_t pulse_us){
		PCA9685_SetChannelPulseUs(s_addr, channel, pulse_us);
}

void WritingArm_SetServoAngle(uint8_t channel, int16_t angle_deg){
		uint8_t idx = (channel < 4) ? channel : 0;
		int max_angle = s_max_angle_deg[idx];
		uint16_t min_us = s_min_us[idx];
		uint16_t max_us = s_max_us[idx];

		if(angle_deg < 0) angle_deg = 0;
		if(angle_deg > max_angle) angle_deg = max_angle;

		// Compute pulse (linear mapping over 0..max_angle)
		uint32_t span = (uint32_t)(max_us - min_us);
		uint32_t pulse = (uint32_t)min_us + ((uint32_t)angle_deg * span) / (uint32_t)max_angle;
		PCA9685_SetChannelPulseUs(s_addr, channel, (uint16_t)pulse);
} 

void WritingArm_PenUp(void){
		WritingArm_SetServoAngle(SERVO_PEN_CH, PEN_UP_ANGLE);
		delay_ms(250);
		s_pen_down = 0;
}

void WritingArm_PenDown(void){
		WritingArm_SetServoAngle(SERVO_PEN_CH, PEN_DOWN_ANGLE);
		delay_ms(250);
		s_pen_down = 1;
}

void WritingArm_EraserUp(void){
		WritingArm_SetServoAngle(SERVO_ERASER_CH, 0);
		delay_ms(300);
}

void WritingArm_EraserDown(void){
		WritingArm_SetServoAngle(SERVO_ERASER_CH, 90);
		delay_ms(300);
}

void WritingArm_Home(void){
		WritingArm_PenUp();
		WritingArm_EraserUp();
		WritingArm_SetServoAngle(SERVO_BASE_CH, 90);
		WritingArm_SetServoAngle(SERVO_ARM_CH, 0);
		delay_ms(500);
		s_cur_base = 90;
		s_cur_arm  = 0;

		// Update grid coordinates corresponding to home angles
		gridFromAngles(s_cur_base, s_cur_arm, &s_cur_gx, &s_cur_gy);
}

void WritingArm_MoveToAngles(int16_t base, int16_t arm){
		WritingArm_SetServoAngle(SERVO_BASE_CH, base);
		WritingArm_SetServoAngle(SERVO_ARM_CH, arm);
		s_cur_base = base;
		s_cur_arm  = arm;

		// Keep grid position in sync with explicit angle moves
		gridFromAngles(s_cur_base, s_cur_arm, &s_cur_gx, &s_cur_gy);
}

// Optional quick self-test for all 4 channels
void WritingArm_ServoSweepTest(void){
		for(uint8_t ch = 0; ch < 4; ++ch){
				for(int a=0; a<=180; a+=30){
						PCA9685_SetServoAngle(s_addr, ch, a, s_min_us[ch], s_max_us[ch]);
						delay_ms(150);
				}
				for(int a=180; a>=0; a-=30){
						PCA9685_SetServoAngle(s_addr, ch, a, s_min_us[ch], s_max_us[ch]);
						delay_ms(150);
				}
		}
}

// --- Font + drawing ---------------------------------------------------------
//


// --- New 5x7 ASCII plotter font (single-stroke, all printable ASCII 32-126) ---
#define FONT_W 5
#define FONT_H 7
#define ASCII_FONT_FIRST 32
#define ASCII_FONT_LAST 126
#define ASCII_FONT_COUNT (ASCII_FONT_LAST-ASCII_FONT_FIRST+1)

// Each glyph: 7 rows, 5 bits per row (LSB=left pixel)
// Simple visible 5x7 font for all printable ASCII (32-126)
// All characters have at least a border or a mark, so nothing is blank
static const uint8_t ascii_font_5x7[ASCII_FONT_COUNT][FONT_H] = {
    // 32-47: space and punctuation
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x04,0x04,0x04,0x04,0x00,0x00,0x04}, // !
    {0x0A,0x0A,0x00,0x00,0x00,0x00,0x00}, // "
    {0x0A,0x1F,0x0A,0x0A,0x1F,0x0A,0x00}, // #
    {0x04,0x1E,0x05,0x0E,0x14,0x0F,0x04}, // $
    {0x19,0x19,0x02,0x04,0x08,0x13,0x13}, // %
    {0x0C,0x12,0x14,0x08,0x15,0x12,0x0D}, // &
    {0x06,0x04,0x08,0x00,0x00,0x00,0x00}, // '
    {0x08,0x04,0x02,0x02,0x02,0x04,0x08}, // (
    {0x02,0x04,0x08,0x08,0x08,0x04,0x02}, // )
    {0x00,0x0A,0x04,0x1F,0x04,0x0A,0x00}, // *
    {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}, // +
    {0x00,0x00,0x00,0x00,0x06,0x04,0x08}, // ,
    {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}, // -
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}, // .
    {0x10,0x08,0x04,0x02,0x01,0x00,0x00}, // /
    // 48-57: 0-9
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, // 0
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, // 1
    {0x0E,0x11,0x10,0x08,0x04,0x02,0x1F}, // 2
    {0x0E,0x11,0x10,0x0C,0x10,0x11,0x0E}, // 3
    {0x08,0x0C,0x0A,0x09,0x1F,0x08,0x08}, // 4
    {0x1F,0x01,0x0F,0x10,0x10,0x11,0x0E}, // 5
    {0x0C,0x02,0x01,0x0F,0x11,0x11,0x0E}, // 6
    {0x1F,0x10,0x08,0x04,0x02,0x02,0x02}, // 7
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, // 8
    {0x0E,0x11,0x11,0x1E,0x10,0x08,0x06}, // 9
    // 58-64: : ; < = > ? @
    {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00}, // :
    {0x00,0x0C,0x0C,0x00,0x0C,0x04,0x08}, // ;
    {0x08,0x04,0x02,0x01,0x02,0x04,0x08}, // <
    {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00}, // =
    {0x02,0x04,0x08,0x10,0x08,0x04,0x02}, // >
    {0x0E,0x11,0x10,0x08,0x04,0x00,0x04}, // ?
    {0x0E,0x11,0x15,0x15,0x0D,0x01,0x0E}, // @
    // 65-90: A-Z
    {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}, // A
    {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, // B
    {0x0E,0x11,0x01,0x01,0x01,0x11,0x0E}, // C
    {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}, // D
    {0x1F,0x01,0x01,0x1F,0x01,0x01,0x1F}, // E
    {0x1F,0x01,0x01,0x1F,0x01,0x01,0x01}, // F
    {0x0E,0x11,0x01,0x1D,0x11,0x11,0x0E}, // G
    {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}, // H
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}, // I
    {0x1C,0x08,0x08,0x08,0x09,0x09,0x06}, // J
    {0x11,0x09,0x05,0x03,0x05,0x09,0x11}, // K
    {0x01,0x01,0x01,0x01,0x01,0x01,0x1F}, // L
    {0x11,0x1B,0x15,0x11,0x11,0x11,0x11}, // M
    {0x11,0x13,0x15,0x19,0x11,0x11,0x11}, // N
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}, // O
    {0x1E,0x11,0x11,0x1E,0x01,0x01,0x01}, // P
    {0x0E,0x11,0x11,0x11,0x15,0x09,0x16}, // Q
    {0x1E,0x11,0x11,0x1E,0x05,0x09,0x11}, // R
    {0x0E,0x11,0x01,0x0E,0x10,0x11,0x0E}, // S
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}, // T
    {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, // U
    {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}, // V
    {0x11,0x11,0x11,0x11,0x15,0x1B,0x11}, // W
    {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}, // X
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}, // Y
    {0x1F,0x10,0x08,0x04,0x02,0x01,0x1F}, // Z
    // 91-96: [ \ ] ^ _ `
    {0x0E,0x02,0x02,0x02,0x02,0x02,0x0E}, // [
    {0x01,0x02,0x04,0x08,0x10,0x00,0x00}, // \
    {0x0E,0x08,0x08,0x08,0x08,0x08,0x0E}, // ]
    {0x04,0x0A,0x11,0x00,0x00,0x00,0x00}, // ^
    {0x00,0x00,0x00,0x00,0x00,0x1F,0x00}, // _
    {0x04,0x08,0x00,0x00,0x00,0x00,0x00}, // `
    // 97-122: a-z
    {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}, // a
    {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, // b
    {0x0E,0x11,0x01,0x01,0x01,0x11,0x0E}, // c
    {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}, // d
    {0x1F,0x01,0x01,0x1F,0x01,0x01,0x1F}, // e
    {0x1F,0x01,0x01,0x1F,0x01,0x01,0x01}, // f
    {0x0E,0x11,0x01,0x1D,0x11,0x11,0x0E}, // g
    {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}, // h
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}, // i
    {0x1C,0x08,0x08,0x08,0x09,0x09,0x06}, // j
    {0x11,0x09,0x05,0x03,0x05,0x09,0x11}, // k
    {0x01,0x01,0x01,0x01,0x01,0x01,0x1F}, // l
    {0x11,0x1B,0x15,0x11,0x11,0x11,0x11}, // m
    {0x11,0x13,0x15,0x19,0x11,0x11,0x11}, // n
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}, // o
    {0x1E,0x11,0x11,0x1E,0x01,0x01,0x01}, // p
    {0x0E,0x11,0x11,0x11,0x15,0x09,0x16}, // q
    {0x1E,0x11,0x11,0x1E,0x05,0x09,0x11}, // r
    {0x0E,0x11,0x01,0x0E,0x10,0x11,0x0E}, // s
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}, // t
    {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, // u
    {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}, // v
    {0x11,0x11,0x11,0x11,0x15,0x1B,0x11}, // w
    {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}, // x
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}, // y
    {0x1F,0x10,0x08,0x04,0x02,0x01,0x1F}, // z
    // 123-126: { | } ~
    {0x08,0x04,0x04,0x02,0x04,0x04,0x08}, // {
    {0x04,0x04,0x04,0x00,0x04,0x04,0x04}, // |
    {0x02,0x04,0x04,0x08,0x04,0x04,0x02}, // }
    {0x00,0x0A,0x15,0x00,0x00,0x00,0x00}, // ~
};

#define FONT_SCALE 2
#define GRID_COLS_PER_CHAR ((FONT_W * FONT_SCALE) + FONT_SCALE)
#define GRID_ROWS_PER_LINE ((FONT_H * FONT_SCALE) + (2*FONT_SCALE))

// Grid bounds. (Think of this as your “paper space” in grid units.)
#define GRID_MAX_X 90
#define GRID_MAX_Y 60

// Servo-space bounds used by the grid mapping.
#define GRID_BASE_MIN BASE_LEFT_ANGLE
#define GRID_BASE_MAX BASE_RIGHT_ANGLE
#define GRID_ARM_MIN  ARM_TOP_ANGLE
#define GRID_ARM_MAX  ARM_BOTTOM_ANGLE


static int lerp_int(int a, int b, int t, int tmax){
		if(t < 0) t = 0;
		if(t > tmax) t = tmax;
		return a + (t * (b - a)) / tmax;
}

static int clamp_int(int v, int lo, int hi){
		if(v < lo) return lo;
		if(v > hi) return hi;
		return v;
}

// Safe margins (degrees) from mechanical end-stops while the pen is DOWN.
// These are *much* gentler than the old “quarter-range clamp”.
#define PEN_DOWN_BASE_MARGIN_DEG  3
#define PEN_DOWN_ARM_MARGIN_DEG   3

// Convert grid coords (gx,gy) -> servo angles (base, arm) by mapping
// grid -> paper (mm) coordinates and using inverse kinematics. The grid
// origin (0,0) corresponds to the paper's top-left and GRID_MAX maps to
// the paper's bottom-right corner (which lines up with the arm base at 0,0).
static void anglesFromGrid(int gx, int gy, int *out_base, int *out_arm){
	if(gx < 0) gx = 0;
	if(gy < 0) gy = 0;
	if(gx > GRID_MAX_X) gx = GRID_MAX_X;
	if(gy > GRID_MAX_Y) gy = GRID_MAX_Y;

	// Normalized positions across the paper (0.0 = left/top, 1.0 = right/bottom)
	float fx = (float)gx / (float)GRID_MAX_X;
	float fy = (float)gy / (float)GRID_MAX_Y;

	// Compute usable paper extents (top-left is (0,0), bottom-right is (usable_w, usable_h))
	float usable_w = PAGE_WIDTH_MM - PAGE_MARGIN_LEFT_MM;
	float usable_h = PAGE_HEIGHT_MM - PAGE_MARGIN_TOP_MM;

	// Auto-scale the mapping down if full page diagonal is outside the arm's reach.
	{
		float diag = sqrtf(usable_w*usable_w + usable_h*usable_h);
		float max_reach = (ARM_L1_MM + ARM_L2_MM) - 5.0f;
		if(diag > max_reach && diag > 0.0f){
			float fit = max_reach / diag;
			usable_w *= fit;
			usable_h *= fit;
		}
	}

	// Map normalized grid to Cartesian coordinates relative to arm base (bottom-right corner)
	// x: -usable_w (left edge) .. 0 (right edge)
	// y: usable_h (top edge) .. 0 (bottom edge)
	float x_mm = -usable_w + fx * usable_w;
	float y_mm = usable_h - fy * usable_h;

	// Use inverse kinematics to get joint angles for this XY point
	int base_deg = 0, arm_deg = 0;
	if(ikInverse(x_mm, y_mm, &base_deg, &arm_deg) != 0){
		// Fall back to safe center if IK failed
		base_deg = (GRID_BASE_MIN + GRID_BASE_MAX) / 2;
		arm_deg = (GRID_ARM_MIN + GRID_ARM_MAX) / 2;
	}

	// Clamp to servo-safe ranges
	base_deg = clamp_int(base_deg, GRID_BASE_MIN, GRID_BASE_MAX);
	arm_deg  = clamp_int(arm_deg,  GRID_ARM_MIN,  GRID_ARM_MAX);

	// Apply optional invert flags (keeps legacy behavior if needed)
	if(BASE_INVERT) base_deg = GRID_BASE_MIN + GRID_BASE_MAX - base_deg;
	if(ARM_INVERT)  arm_deg  = GRID_ARM_MIN  + GRID_ARM_MAX  - arm_deg;

	*out_base = base_deg;
	*out_arm  = arm_deg;
} 

// Convert servo angles back to grid coords (inverse of anglesFromGrid).
// This computes the Cartesian (x,y) via FK then maps back into normalized
// paper coordinates and finally into grid units.
static void gridFromAngles(int base, int arm, int *out_gx, int *out_gy){
	// Restore any legacy inverts before using FK
	int b = base;
	int a = arm;
	if(BASE_INVERT) b = GRID_BASE_MIN + GRID_BASE_MAX - b;
	if(ARM_INVERT)  a = GRID_ARM_MIN  + GRID_ARM_MAX  - a;

	// Compute Cartesian position relative to base
	float x_mm, y_mm;
	fkFromAngles(b, a, &x_mm, &y_mm);

	// Compute usable extents (must match anglesFromGrid)
	float usable_w = PAGE_WIDTH_MM - PAGE_MARGIN_LEFT_MM;
	float usable_h = PAGE_HEIGHT_MM - PAGE_MARGIN_TOP_MM;

	{
		float diag = sqrtf(usable_w*usable_w + usable_h*usable_h);
		float max_reach = (ARM_L1_MM + ARM_L2_MM) - 5.0f;
		if(diag > max_reach && diag > 0.0f){
			float fit = max_reach / diag;
			usable_w *= fit;
			usable_h *= fit;
		}
	}

	if(usable_w <= 0.0f) usable_w = PAGE_WIDTH_MM;
	if(usable_h <= 0.0f) usable_h = PAGE_HEIGHT_MM;

	// Invert mapping to normalized fractions
	float fx = (x_mm + usable_w) / usable_w;
	float fy = (usable_h - y_mm) / usable_h;

	if(fx < 0.0f) fx = 0.0f; if(fx > 1.0f) fx = 1.0f;
	if(fy < 0.0f) fy = 0.0f; if(fy > 1.0f) fy = 1.0f;

	int gx = (int)roundf(fx * (float)GRID_MAX_X);
	int gy = (int)roundf(fy * (float)GRID_MAX_Y);

	// Clamp again just in case
	if(gx < 0) gx = 0; if(gx > GRID_MAX_X) gx = GRID_MAX_X;
	if(gy < 0) gy = 0; if(gy > GRID_MAX_Y) gy = GRID_MAX_Y;

	*out_gx = gx;
	*out_gy = gy;
}

// Forward kinematics: servo angles (deg) -> Cartesian (mm)
static void fkFromAngles(int base_deg, int arm_deg, float *out_x_mm, float *out_y_mm){
		float theta1 = (base_deg - 90.0f) * (float)M_PI / 180.0f;
		float theta2 = (arm_deg  - 90.0f) * (float)M_PI / 180.0f;
		float x = ARM_L1_MM * cosf(theta1) + ARM_L2_MM * cosf(theta1 + theta2);
		float y = ARM_L1_MM * sinf(theta1) + ARM_L2_MM * sinf(theta1 + theta2);
		*out_x_mm = x;
		*out_y_mm = y;
}

// Inverse kinematics: (x,y)->servo angles (deg). Returns 0=ok
static int ikInverse(float x, float y, int *out_base_deg, int *out_arm_deg){
		float L1 = ARM_L1_MM;
		float L2 = ARM_L2_MM;
		float r2 = x*x + y*y;
		float cos_q2 = (r2 - L1*L1 - L2*L2) / (2.0f * L1 * L2);
		if(cos_q2 > 1.0f) cos_q2 = 1.0f;
		if(cos_q2 < -1.0f) cos_q2 = -1.0f;
		float sin_q2_pos = sqrtf(1.0f - cos_q2 * cos_q2);
		float sin_q2_neg = -sin_q2_pos;
		float q2_options[2] = { atan2f(sin_q2_pos, cos_q2), atan2f(sin_q2_neg, cos_q2) };
		int best_base = 0, best_arm = 0;
		float best_score = 1e12f;
		for(int i=0;i<2;i++){
			float q2 = q2_options[i];
			float q1 = atan2f(y, x) - atan2f(L2 * sinf(q2), L1 + L2 * cosf(q2));
			int base_deg = (int)roundf((q1 * 180.0f / (float)M_PI) + 90.0f);
			int arm_deg = (int)roundf((q2 * 180.0f / (float)M_PI) + 90.0f);
			float score = fabsf((float)arm_deg - (float)s_cur_arm);
			if(score < best_score){
				best_score = score;
				best_base = base_deg;
				best_arm = arm_deg;
			}
		}
		*out_base_deg = best_base;
		*out_arm_deg = best_arm;
		return 0;
}

// --- Text buffer API implementations (moved here) -------------------------

// Clear the queued text buffer
void WritingArm_ClearTextBuffer(void){
		s_text_len = 0;
		if(TEXT_BUF_MAX > 0) s_text_buf[0] = '\0';
}

// Append a NUL-terminated string to the text buffer. Returns number of
// characters appended (0 on failure or if buffer is full).
int WritingArm_QueueText(const char *s){
		if(!s) return 0;
		int added = 0;
		while(*s && s_text_len < (TEXT_BUF_MAX - 1)){
			s_text_buf[s_text_len++] = *s++;
			added++;
		}
		s_text_buf[s_text_len] = '\0';
		return added;
}

// Write the queued buffer starting from the top-left of the page and
// proceeding left->right, top->bottom until the buffer or page runs out.
// Clears the buffer when finished.
void WritingArm_WriteBuffer(void){
		int x = 0;
		int y = 0;

		for(int i = 0; i < s_text_len; ++i){
			char c = s_text_buf[i];

			if(c == '\n' || c == '\r'){
				x = 0;
				y += GRID_ROWS_PER_LINE;
				if(y + (FONT_H * FONT_SCALE) > GRID_MAX_Y) break; // no more room
				continue;
			}

			if(x + GRID_COLS_PER_CHAR > GRID_MAX_X){
				x = 0;
				y += GRID_ROWS_PER_LINE;
				if(y + (FONT_H * FONT_SCALE) > GRID_MAX_Y) break; // page full
			}

			// Draw the character at (x,y)
			WritingArm_DrawCharAt(c, x, y);
			x += GRID_COLS_PER_CHAR;
		}

		// Clear buffer after writing
		WritingArm_ClearTextBuffer();
}

// Map grid cell to paper XY using the same rules as anglesFromGrid
static void gridToXY(int gx, int gy, float *out_x_mm, float *out_y_mm){
		if(gx < 0) gx = 0;
		if(gy < 0) gy = 0;
		if(gx > GRID_MAX_X) gx = GRID_MAX_X;
		if(gy > GRID_MAX_Y) gy = GRID_MAX_Y;

		float fx = (float)gx / (float)GRID_MAX_X;
		float fy = (float)gy / (float)GRID_MAX_Y;

		float usable_w = PAGE_WIDTH_MM - PAGE_MARGIN_LEFT_MM;
		float usable_h = PAGE_HEIGHT_MM - PAGE_MARGIN_TOP_MM;

		{
			float diag = sqrtf(usable_w*usable_w + usable_h*usable_h);
			float max_reach = (ARM_L1_MM + ARM_L2_MM) - 5.0f;
			if(diag > max_reach && diag > 0.0f){
				float fit = max_reach / diag;
				usable_w *= fit;
				usable_h *= fit;
			}
		}

		// Map to coordinates relative to base at paper bottom-right
		float x_mm = -usable_w + fx * usable_w;
		float y_mm = usable_h - fy * usable_h;

		*out_x_mm = x_mm;
		*out_y_mm = y_mm;
}

// Diagnostic: Dump mapping samples (corners + center) over UART
void WritingArm_DumpMappingSamples(void){
		// Ensure UART initialized
		UART0_Init();

		const int samples[5][2] = {
			{0, 0},
			{GRID_MAX_X, GRID_MAX_Y},
			{0, GRID_MAX_Y},
			{GRID_MAX_X, 0},
			{GRID_MAX_X/2, GRID_MAX_Y/2}
		};

		for(int i=0;i<5;i++){
			int gx = samples[i][0];
			int gy = samples[i][1];

			float x_mm, y_mm;
			gridToXY(gx, gy, &x_mm, &y_mm);

			int base_deg = 0, arm_deg = 0;
			ikInverse(x_mm, y_mm, &base_deg, &arm_deg);

			float fx2, fy2;
			fkFromAngles(base_deg, arm_deg, &fx2, &fy2);

			int gx2, gy2;
			gridFromAngles(base_deg, arm_deg, &gx2, &gy2);

			UART0_OutString((uint8_t*)"Sample: grid ");
			UART0_OutSDec(gx); UART0_OutString((uint8_t*)","); UART0_OutSDec(gy);
			UART0_OutString((uint8_t*)" -> XY(mm) "); UART0_OutSDec((int)x_mm); UART0_OutString((uint8_t*)","); UART0_OutSDec((int)y_mm);
			UART0_OutString((uint8_t*)" -> IK angles "); UART0_OutSDec(base_deg); UART0_OutString((uint8_t*)","); UART0_OutSDec(arm_deg);
			UART0_OutString((uint8_t*)" -> FK XY(mm) "); UART0_OutSDec((int)fx2); UART0_OutString((uint8_t*)","); UART0_OutSDec((int)fy2);
			UART0_OutString((uint8_t*)" -> grid "); UART0_OutSDec(gx2); UART0_OutString((uint8_t*)","); UART0_OutSDec(gy2);
			UART0_NextLine();
		}
}

static void moveToGrid(int gx, int gy){
		if(gx < 0) gx = 0;
		if(gy < 0) gy = 0;
		if(gx > GRID_MAX_X) gx = GRID_MAX_X;
		if(gy > GRID_MAX_Y) gy = GRID_MAX_Y;

		// Compute target angles from grid
		int target_base, target_arm;
		anglesFromGrid(gx, gy, &target_base, &target_arm);

		// Clamp when pen is down
		if(s_pen_down){
			target_base = clamp_int(target_base, GRID_BASE_MIN + PEN_DOWN_BASE_MARGIN_DEG,
						 GRID_BASE_MAX - PEN_DOWN_BASE_MARGIN_DEG);
			target_arm  = clamp_int(target_arm,  GRID_ARM_MIN  + PEN_DOWN_ARM_MARGIN_DEG,
						 GRID_ARM_MAX  - PEN_DOWN_ARM_MARGIN_DEG);
		}

		// Current/target Cartesian positions
		float x0, y0, x1, y1;
		fkFromAngles(s_cur_base, s_cur_arm, &x0, &y0);
		fkFromAngles(target_base, target_arm, &x1, &y1);

		float dx = x1 - x0;
		float dy = y1 - y0;
		float dist = sqrtf(dx*dx + dy*dy);
		float step_mm = s_pen_down ? 1.0f : 4.0f;
		int steps = (int)(dist / step_mm) + 1;
		if(steps < 1) steps = 1;

		for(int t=1; t<=steps; t++){
			float fx = x0 + (dx * t) / (float)steps;
			float fy = y0 + (dy * t) / (float)steps;

			int ibase = target_base;
			int iarm  = target_arm;
			if(ikInverse(fx, fy, &ibase, &iarm) != 0){
				// fallback to angle interpolation
				ibase = lerp_int(s_cur_base, target_base, t, steps);
				iarm  = lerp_int(s_cur_arm, target_arm, t, steps);
			}

			if(s_pen_down){
				ibase = clamp_int(ibase, GRID_BASE_MIN + PEN_DOWN_BASE_MARGIN_DEG,
						 GRID_BASE_MAX - PEN_DOWN_BASE_MARGIN_DEG);
				iarm  = clamp_int(iarm,  GRID_ARM_MIN  + PEN_DOWN_ARM_MARGIN_DEG,
						 GRID_ARM_MAX  - PEN_DOWN_ARM_MARGIN_DEG);
			}

			WritingArm_SetServoAngle(SERVO_BASE_CH, ibase);
			WritingArm_SetServoAngle(SERVO_ARM_CH,  iarm);
			delay_ms((uint32_t)(s_pen_down ? 45 : 22));
		}

		// Update current states to target
		s_cur_base = target_base;
		s_cur_arm  = target_arm;
		s_cur_gx = gx;
		s_cur_gy = gy;
	}

// Lookup for new ASCII font (returns pointer to 7 bytes for printable ASCII, or blank for others)
static const uint8_t *ascii_glyph_rows(char c) {
    if (c < ASCII_FONT_FIRST || c > ASCII_FONT_LAST) {
        static const uint8_t blank[FONT_H] = {0,0,0,0,0,0,0};
        return blank;
    }
    return ascii_font_5x7[c - ASCII_FONT_FIRST];
}

static void penDotAt(int gx, int gy){
		moveToGrid(gx, gy);
		WritingArm_PenDown();
		delay_ms(120);
		WritingArm_PenUp();
}



// Glyph mapping tunables: rotation in degrees {0,90,180,270} and optional flips
#ifndef FONT_ROTATION_DEG
#define FONT_ROTATION_DEG 0
#endif
#ifndef FONT_FLIP_X
#define FONT_FLIP_X 0
#endif
#ifndef FONT_FLIP_Y
#define FONT_FLIP_Y 0
#endif

// Map glyph cell (col,row) to grid coords (gx,gy).
static void mapGlyphCell(int x0, int y0, int col, int row, int *out_gx, int *out_gy){
	int lx = 0, ly = 0; // local coordinates (in grid units)
	switch(FONT_ROTATION_DEG){
		case 0:
			lx = col * FONT_SCALE;
			ly = row * FONT_SCALE;
			break;
		case 90:
			lx = row * FONT_SCALE;
			ly = (FONT_W - 1 - col) * FONT_SCALE;
			break;
		case 180:
			lx = (FONT_W - 1 - col) * FONT_SCALE;
			ly = (FONT_H - 1 - row) * FONT_SCALE;
			break;
		case 270:
			lx = (FONT_H - 1 - row) * FONT_SCALE;
			ly = col * FONT_SCALE;
			break;
		default:
			lx = col * FONT_SCALE;
			ly = row * FONT_SCALE;
			break;
	}

	// Apply flips (relative to glyph box extent)
	if(FONT_FLIP_X){
		lx = (FONT_W - 1) * FONT_SCALE - lx;
	}
	if(FONT_FLIP_Y){
		ly = (FONT_H - 1) * FONT_SCALE - ly;
	}

	int gx = x0 + lx;
	int gy = y0 + ly;
	// clamp
	if(gx < 0) gx = 0;
	if(gx > GRID_MAX_X) gx = GRID_MAX_X;
	if(gy < 0) gy = 0;
	if(gy > GRID_MAX_Y) gy = GRID_MAX_Y;
	*out_gx = gx;
	*out_gy = gy;
}

// Draw a character using stroke extraction from the 5x5 bitmap.
// Strategy:
//  1) Draw long vertical runs (len >= 2) as one stroke each.
//  2) Draw long horizontal runs (len >= 2) as one stroke each.
//  3) Remaining single pixels become dots.
void WritingArm_DrawCharAt(char c, int x0, int y0){
	s_left_handed = 1;
	const uint8_t *rows = ascii_glyph_rows(c);
	WritingArm_PenUp();

	// Track which pixels have already been covered by strokes.
	uint8_t covered[FONT_H] = {0};

	// Pass 1: vertical runs
	for(int col = 0; col < FONT_W; col++){
		int r = 0;
		while(r < FONT_H){
			while(r < FONT_H && (((rows[r] >> col) & 1U) == 0)) r++;
			if(r >= FONT_H) break;
			int start = r;
			while(r < FONT_H && (((rows[r] >> col) & 1U) != 0)) r++;
			int end = r - 1;
			int len = end - start + 1;

			if(len >= 2){
				int gx1, gy1, gx2, gy2;
				mapGlyphCell(x0, y0, col, start, &gx1, &gy1);
				mapGlyphCell(x0, y0, col, end,   &gx2, &gy2);

				moveToGrid(gx1, gy1);
				WritingArm_PenDown();
				moveToGrid(gx2, gy2);
				WritingArm_PenUp();

				for(int rr = start; rr <= end; rr++){
					covered[rr] |= (1U << col);
				}
			}
		}
	}

	// Pass 2: horizontal runs
	for(int row = 0; row < FONT_H; row++){
		uint8_t bits = rows[row] & 0x1F;
		bits &= (uint8_t)(~covered[row]) & 0x1F;

		int col = 0;
		while(col < FONT_W){
			while(col < FONT_W && ((bits >> col) & 1U) == 0) col++;
			if(col >= FONT_W) break;
			int start = col;
			while(col < FONT_W && ((bits >> col) & 1U) != 0) col++;
			int end = col - 1;
			int len = end - start + 1;

			if(len >= 2){
				int gx1, gy1, gx2, gy2;
				mapGlyphCell(x0, y0, start, row, &gx1, &gy1);
				mapGlyphCell(x0, y0, end,   row, &gx2, &gy2);

				moveToGrid(gx1, gy1);
				WritingArm_PenDown();
				moveToGrid(gx2, gy2);
				WritingArm_PenUp();

				for(int cc = start; cc <= end; cc++){
					covered[row] |= (1U << cc);
				}
			}
		}
	}

	// Pass 3: dots for remaining pixels
	for(int row = 0; row < FONT_H; row++){
		uint8_t bits = rows[row] & 0x1F;
		bits &= (uint8_t)(~covered[row]) & 0x1F;

		for(int col = 0; col < FONT_W; col++){
			if(((bits >> col) & 1U) != 0){
				int gx, gy;
				mapGlyphCell(x0, y0, col, row, &gx, &gy);
				penDotAt(gx, gy);
			}
		}
	}
}

void WritingArm_DrawChar(char c, int x_origin){
	WritingArm_DrawCharAt(c, x_origin, 0);
	moveToGrid(x_origin + GRID_COLS_PER_CHAR, 0);
}

void WritingArm_DrawString(const char *s, int x_origin){
	int x = x_origin;
	int y = 0;

	while(s && *s){
		char c = *s++;

		if(c == '\n' || c == '\r'){
			x = x_origin;
			y += GRID_ROWS_PER_LINE;
			continue;
		}

		if(x + GRID_COLS_PER_CHAR > GRID_MAX_X){
			x = x_origin;
			y += GRID_ROWS_PER_LINE;
		}
		if(y + (FONT_H * FONT_SCALE) > GRID_MAX_Y) break;

		WritingArm_DrawCharAt(c, x, y);
		x += GRID_COLS_PER_CHAR;
	}
}
