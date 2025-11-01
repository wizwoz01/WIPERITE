/*
 * main.c - Example project for UT.6.02x Embedded Systems - Shape the World
 * Jonathan Valvano and Ramesh Yerraballi
 * October 27, 2015
 * Hardware requirements 
     TM4C123 LaunchPad, optional Nokia5110
     CC3100 wifi booster and 
     an internet access point with OPEN, WPA, or WEP security
 
 * derived from TI's getweather example
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/*
 * Application Name     -   Get weather
 * Application Overview -   This is a sample application demonstrating how to
                            connect to openweathermap.org server and request for
              weather details of a city.
 * Application Details  -   http://processors.wiki.ti.com/index.php/CC31xx_SLS_Get_Weather_Application
 *                          doc\examples\sls_get_weather.pdf
 */
 /* CC3100 booster pack connections (unused pins can be used by user application)
Pin  Signal        Direction      Pin   Signal     Direction
P1.1  3.3 VCC         IN          P2.1  Gnd   GND      IN
P1.2  PB5 UNUSED      NA          P2.2  PB2   IRQ      OUT
P1.3  PB0 UART1_TX    OUT         P2.3  PE0   SSI2_CS  IN
P1.4  PB1 UART1_RX    IN          P2.4  PF0   UNUSED   NA
P1.5  PE4 nHIB        IN          P2.5  Reset nRESET   IN
P1.6  PE5 UNUSED      NA          P2.6  PB7  SSI2_MOSI IN
P1.7  PB4 SSI2_CLK    IN          P2.7  PB6  SSI2_MISO OUT
P1.8  PA5 UNUSED      NA          P2.8  PA4   UNUSED   NA
P1.9  PA6 UNUSED      NA          P2.9  PA3   UNUSED   NA
P1.10 PA7 UNUSED      NA          P2.10 PA2   UNUSED   NA

Pin  Signal        Direction      Pin   Signal      Direction
P3.1  +5  +5 V       IN           P4.1  PF2 UNUSED      OUT
P3.2  Gnd GND        IN           P4.2  PF3 UNUSED      OUT
P3.3  PD0 M1PWM0     OUT          P4.3  PB3 UNUSED      NA
P3.4  PD1 M1PWM1     OUT          P4.4  PC4 UART1_CTS   IN
P3.5  PD2 UNUSED     NA           P4.5  PC5 UART1_RTS   OUT
P3.6  PD3 UNUSED     NA           P4.6  PC6 UNUSED      NA
P3.7  PE1 UNUSED     NA           P4.7  PC7 NWP_LOG_TX  OUT
P3.8  PE2 UNUSED     NA           P4.8  PD6 WLAN_LOG_TX OUT
P3.9  PE3 UNUSED     NA           P4.9  PD7 UNUSED      IN (see R74)
P3.10 PF1 UNUSED     NA           P4.10 PF4 UNUSED      OUT(see R75)

UART0 (PA1, PA0) sends data to the PC via the USB debug cable, 115200 baud rate
Port A, SSI0 (PA2, PA3, PA5, PA6, PA7) sends data to Nokia5110 LCD

*/
#include <stdint.h>
#include "..\cc3100\simplelink\include\simplelink.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "utils/cmdline.h"
#include "application_commands.h"
#include "LED.h"
#include <string.h>
#include <stdlib.h> // for atof
#include "ST7735.h"
#include "../inc/tm4c123gh6pm.h"
#include "bmps.h"
#include "UART0.h"
#include "motor.h"
#include "PWM.h"
#include "SysTick.h"
#ifndef SL_EAGAIN
#define SL_EAGAIN (-11)
#endif

// Optional: small cooperative delay helper
static inline void delay_ms(uint32_t ms){
  // SysTick is configured for busy-wait in this project
  // T1ms is defined as 50,000 for 1ms at 50MHz
  for(uint32_t i=0;i<ms;i++){
    SysTick_Wait(T1ms);
    // Keep the SimpleLink host driver serviced even during small delays
    _SlNonOsMainLoopTask();
  }
}

// To Do: replace the following three lines with your access point information
#define SSID_NAME  "test00" /* Access point name to connect to */
#define SEC_TYPE   SL_SEC_TYPE_WPA
#define PASSKEY    "011101110111"  /* Password in case of secure AP */ 
#define MAXLEN 100


#define MAX_RECV_BUFF_SIZE  1024
#define MAX_SEND_BUFF_SIZE  512
#define MAX_HOSTNAME_SIZE   40
#define MAX_PASSKEY_SIZE    32
#define MAX_SSID_SIZE       32


#define SUCCESS             0

#define CONNECTION_STATUS_BIT   0
#define IP_AQUIRED_STATUS_BIT   1

// Reduce TX power a bit to lower burst currents and RF self-interference
// 0 = max power; higher numbers are dB offset from max (0..15)
#ifndef WIFI_TX_POWER_DB_OFFSET
#define WIFI_TX_POWER_DB_OFFSET 4
#endif

/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,/* Choosing this number to avoid overlap w/ host-driver's error codes */

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;


/* Status bits - These are used to set/reset the corresponding bits in 'g_Status' */
typedef enum{
    STATUS_BIT_CONNECTION =  0, /* If this bit is:
                                 *      1 in 'g_Status', the device is connected to the AP
                                 *      0 in 'g_Status', the device is not connected to the AP
                                 */

    STATUS_BIT_IP_AQUIRED,       /* If this bit is:
                                 *      1 in 'g_Status', the device has acquired an IP
                                 *      0 in 'g_Status', the device has not acquired an IP
                                 */

}e_StatusBits;


#define SET_STATUS_BIT(status_variable, bit)    status_variable |= (1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit)    status_variable &= ~(1<<(bit))
#define GET_STATUS_BIT(status_variable, bit)    (0 != (status_variable & (1<<(bit))))
#define IS_CONNECTED(status_variable)           GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_CONNECTION)
#define IS_IP_AQUIRED(status_variable)          GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_IP_AQUIRED)

typedef struct{
    UINT8 SSID[MAX_SSID_SIZE];
    INT32 encryption;
    UINT8 password[MAX_PASSKEY_SIZE];
}UserInfo;

/*
 * GLOBAL VARIABLES -- Start
 */

char Recvbuff[MAX_RECV_BUFF_SIZE];
char SendBuff[MAX_SEND_BUFF_SIZE];
char HostName[MAX_HOSTNAME_SIZE];
unsigned long DestinationIP;
int SockID;


typedef enum{
    CONNECTED = 0x01,
    IP_AQUIRED = 0x02,
    IP_LEASED = 0x04,
    PING_DONE = 0x08

}e_Status;
UINT32  g_Status = 0;
// Reconnect management flags
static volatile uint8_t g_wifi_disconnect_flag = 0; // set by WLAN disconnect event
static volatile uint8_t g_wifi_connecting = 0;      // guard concurrent attempts
static uint32_t g_reconnect_backoff_ms = 0;         // simple backoff (0, 1000, 2000, max 8000)
static uint32_t g_reconnect_timer_ms = 0;           // countdown in main loop
static uint8_t g_reconnect_attempts = 0;            // attempts since last success
static volatile uint8_t g_ui_dirty = 1;              // request a UI refresh
static volatile uint8_t g_ip_b1=0,g_ip_b2=0,g_ip_b3=0,g_ip_b4=0; // last acquired IP
static volatile uint32_t g_last_disc_reason = 0;     // last disconnect reason code
static volatile uint8_t g_gw_b1=0,g_gw_b2=0,g_gw_b3=0,g_gw_b4=0; // last gateway IP
static volatile char g_last_cmd = 0;                 // last processed command for UI
static volatile uint32_t g_rx_count = 0;             // processed command byte counter
/*
 * GLOBAL VARIABLES -- End
 */


 /*
 * STATIC FUNCTION DEFINITIONS  -- Start
 */

static int32_t configureSimpleLinkToDefaultState(char *);
static void wifi_attempt_connect(void);
static void sockets_close_if_open(int *pListenSock, int *pClientSock);
static int sockets_ensure_listening(int *pListenSock);
static void socket_set_nonblocking(int sd);
static void ui_render(int listenSock, int clientSock);
// Soft safety: auto-stop motors if no motion command arrives for a short time
static volatile uint32_t g_motion_timeout_ms = 0; // countdown; when reaches 0 -> Stop
// Define tighter motion watchdog timeouts to make manual commands momentary
#define MANUAL_MOTION_TIMEOUT_MS 120u   // ~120 ms pulse for F/B/L/R commands
#define SPEED_HOLD_TIMEOUT_MS    250u   // keep lift/lower active briefly
// Short blue LED pulse on command receive
static volatile uint32_t g_led_blue_pulse_ms = 0;
// Small command queue to decouple network RX from motor actions
#define CMD_Q_SIZE 32u
#define CMD_Q_MASK (CMD_Q_SIZE-1u)
static volatile uint8_t g_cmd_q[CMD_Q_SIZE];
static volatile uint8_t g_cmd_head = 0; // next write (masked index)
static volatile uint8_t g_cmd_tail = 0; // next read  (masked index)
static inline int cmdq_is_full(void){ return (((g_cmd_head + 1u) & CMD_Q_MASK) == g_cmd_tail); }
static inline int cmdq_is_empty(void){ return (g_cmd_head == g_cmd_tail); }
static inline void cmdq_push(uint8_t c){ if(!cmdq_is_full()){ g_cmd_q[g_cmd_head] = c; g_cmd_head = (g_cmd_head + 1u) & CMD_Q_MASK; } }
static inline uint8_t cmdq_pop(void){ uint8_t c = 0; if(!cmdq_is_empty()){ c = g_cmd_q[g_cmd_tail]; g_cmd_tail = (g_cmd_tail + 1u) & CMD_Q_MASK; } return c; }
static uint8_t g_rx_escape_state = 0; // 0: idle, 1: saw ESC, 2: saw ESC[


/*
 * STATIC FUNCTION DEFINITIONS -- End
 */


void Crash(uint32_t time){
  while(1){
    for(int i=time;i;i--){};
    LED_RedToggle();
  }
}

// Hard fault handler: flash red LED rapidly so we can distinguish a crash
void HardFault_Handler(void){
  // Turn off other LEDs for clarity
  LED_GreenOff();
  LED_BlueOff();
  while(1){
    for(volatile int i=0;i<200000;i++){} // ~short delay
    LED_RedToggle();
  }
}


/*
 * Application's entry point
 */
// TCP command server port
#define LISTEN_PORT 5000
int main(void){
	int32_t retVal;  
	SlSecParams_t secParams;
  char *pConfig = NULL; 
  INT32 ASize = 0; 
  SlSockAddrIn_t  Addr;
    
    // For server
    SlSockAddrIn_t  LocalAddr;
    SlSockAddrIn_t  ClientAddr;
    SlSocklen_t     ClientLen;
  int ListenSock = -1;
  int ClientSock = -1;
  int16_t n;
  char cmdBuf[64];

  initClk();        // PLL 50 MHz
  UART_Init();      // Send data to PC, 115200 bps
  LED_Init();       // initialize LaunchPad I/O 
  ST7735_InitR(INITR_REDTAB);
  UART_OutString((uint8_t*)"WiFi WIPERITE Control\n\r");
  ST7735_OutString("WiFi WIPERITE Ctrl\n\r");

  // Initialize car control hardware 
  MotorControl_Init();
  SysTick_Init(); // enable busy-wait timing for cooperative scheduling
  // Hold PWM pins in a quiet state handled elsewhere

  // Latch a WiFi-only debug mode if PF4 held at boot (negative logic)
  // Requires PortF already initialized by MotorControl_Init
  uint8_t motors_locked = ((GPIO_PORTF_DATA_R & 0x10) == 0) ? 1 : 0;
  if(motors_locked){
    UART_OutString((uint8_t*)"Motor Debug Mode: Motors locked OFF (PF4 held)\r\n");
    ST7735_OutString("MotorDbg: OFF\r\n");
  }
  // Prevent PWM enabling inside command handler when locked
  MotorControl_SetEnableAllowed(!motors_locked);
  uint8_t motors_enabled = 0; // enable motors only when a client connects
	
  retVal = configureSimpleLinkToDefaultState(pConfig); // set policies
  if(retVal < 0)Crash(4000000);
  retVal = sl_Start(0, pConfig, 0);
  if((retVal < 0) || (ROLE_STA != retVal) ) Crash(8000000);
  secParams.Key = PASSKEY;
  secParams.KeyLen = strlen(PASSKEY);
  secParams.Type = SEC_TYPE; // OPEN, WPA, or WEP
  sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
  g_wifi_connecting = 1;
  while((0 == (g_Status&CONNECTED)) || (0 == (g_Status&IP_AQUIRED))){
    _SlNonOsMainLoopTask();
  }
  g_wifi_connecting = 0;
  UART_OutString((uint8_t*)"Connected\n\r");
  ST7735_OutString("Connected\n\r");
  g_ui_dirty = 1;
  // Keep motors silent until a client connects
	
  // Set up TCP server socket
  LocalAddr.sin_family = SL_AF_INET;
  LocalAddr.sin_port = sl_Htons(LISTEN_PORT);
  LocalAddr.sin_addr.s_addr = 0; // bind to all interfaces
  ASize = sizeof(SlSockAddrIn_t);

  ListenSock = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, 0);
  if(ListenSock < 0){ Crash(4000000); }
  if(sl_Bind(ListenSock, (SlSockAddr_t *)&LocalAddr, ASize) < 0){ Crash(4000000); }
  if(sl_Listen(ListenSock, 1) < 0){ Crash(4000000); }
  // Make listen socket non-blocking so we can service the SimpleLink driver and reconnect
  socket_set_nonblocking(ListenSock);

  UART_OutString((uint8_t*)"Listening on TCP port ");
  UART_OutUDec(LISTEN_PORT);
  UART_OutString((uint8_t*)"\r\nCommands: F,B,L,R,H/space(stop),U,D,8,C,S(square),Z. WASD/arrow keys supported. 'Q' to close.\r\n");
  g_ui_dirty = 1;

  // Cooperative main loop: poll sockets and service driver; auto-reconnect on drop
  while(1){
    // Always service the SimpleLink driver in non-OS mode
    _SlNonOsMainLoopTask();

    // If a disconnect was signaled, close sockets and schedule reconnect attempts
    if(g_wifi_disconnect_flag){
      sockets_close_if_open(&ListenSock, &ClientSock);
    g_wifi_disconnect_flag = 0; // handled
  // On WiFi loss, ensure motors are silenced
  Car_ProcessCommand('s');
  PWM_DisableOutputs();
  PWM_PinsToHiZ();
  motors_enabled = 0;
  g_motion_timeout_ms = 0;
  LED_BlueOff();
  g_led_blue_pulse_ms = 0;
      // start or increase backoff
      if(g_reconnect_backoff_ms == 0) g_reconnect_backoff_ms = 1000;
      else if(g_reconnect_backoff_ms < 8000) g_reconnect_backoff_ms <<= 1; // up to 8s
      g_reconnect_timer_ms = g_reconnect_backoff_ms;
      g_wifi_connecting = 0; // allow new attempt
    }

    // Attempt reconnect when not connected and timer expired
    if(!IS_CONNECTED(g_Status) || !IS_IP_AQUIRED(g_Status)){
      if(g_reconnect_timer_ms == 0 && !g_wifi_connecting){
        wifi_attempt_connect();
      }
    } else {
      // Connected: ensure we have a listening socket
      if(ListenSock < 0){
        if(sockets_ensure_listening(&ListenSock) < 0){
          // try again later
        } else {
          UART_OutString((uint8_t*)"Listening on TCP port ");
          UART_OutUDec(LISTEN_PORT);
          UART_OutString((uint8_t*)"\r\n");
          g_ui_dirty = 1;
        }
      }
    }

    // Accept new client if none; non-blocking
  if(IS_CONNECTED(g_Status) && IS_IP_AQUIRED(g_Status) && ListenSock >= 0 && ClientSock < 0){
      ClientLen = sizeof(ClientAddr);
      ClientSock = sl_Accept(ListenSock, (SlSockAddr_t *)&ClientAddr, &ClientLen);
      if(ClientSock >= 0){
        LED_GreenOn();
        socket_set_nonblocking(ClientSock);
        // Enable TCP keepalive to survive NAT/idle networks
        int ka = 1;
        sl_SetSockOpt(ClientSock, SL_SOL_SOCKET, SL_SO_KEEPALIVE, &ka, sizeof(ka));
        const char *hello = "Connected to TM4C WiFi Robot\r\n";
        sl_Send(ClientSock, hello, (int)strlen(hello), 0);
        // Enable motors only when a client is connected and not locked
        if(!motors_locked && !motors_enabled){
          PWM_PinsToPWM();
          MotorPWM_Enable();
          motors_enabled = 1;
          UART_OutString((uint8_t*)"Motors enabled\r\n");
          MotorControl_SetEnableAllowed(1);
        }
        g_ui_dirty = 1;
      }
    }

    // If client connected, try to recv non-blocking
    if(ClientSock >= 0){
      n = (int16_t)sl_Recv(ClientSock, cmdBuf, sizeof(cmdBuf), 0);
      if(n > 0){
        for(int idx=0; idx<n; idx++){
          char c = cmdBuf[idx];
          if(c=='\r' || c=='\n') continue;
          if(c == '\x1b'){
            g_rx_escape_state = 1;
            continue;
          }
          if(g_rx_escape_state == 1){
            g_rx_escape_state = (c == '[') ? 2 : 0;
            continue;
          }
          if(g_rx_escape_state == 2){
            char mapped = 0;
            switch(c){
              case 'A': mapped = 'F'; break; // Up arrow -> Forward
              case 'B': mapped = 'B'; break; // Down arrow -> Reverse
              case 'C': mapped = 'R'; break; // Right arrow -> Right turn
              case 'D': mapped = 'L'; break; // Left arrow -> Left turn
              default: mapped = 0; break;
            }
            g_rx_escape_state = 0;
            if(mapped == 0){
              continue; // ignore unknown escape sequence
            }
            c = mapped;
          }
          switch(c){
            case 'w':
            case 'W':
              c = 'F';
              break;
            case 'a':
            case 'A':
              c = 'L';
              break;
            case 'd':
              c = 'R';
              break;
            default:
              break;
          }
          // Convert Halt and Space to lowercase 's' (Stop), but preserve uppercase 'S' (Square)
          if(c=='H' || c==' '){ c = 's'; }
          if(c=='Q' || c=='q'){
            const char *bye = "Bye\r\n";
            sl_Send(ClientSock, bye, (int)strlen(bye), 0);
            sl_Close(ClientSock);
            ClientSock = -1;
            LED_GreenOff();
            // Safely stop and silence motors on client quit
            Car_ProcessCommand('s');
            PWM_DisableOutputs();
            PWM_PinsToHiZ();
            motors_enabled = 0;
            // Clear last command on quit so UI doesn't show stale input
            g_last_cmd = 0;
            g_rx_count = 0;
            g_motion_timeout_ms = 0;
            g_led_blue_pulse_ms = 0;
            LED_BlueOff();
            g_rx_escape_state = 0;
            g_ui_dirty = 1;
            n = -1; // Signal to stop processing this buffer
            continue;
          }
          // Queue the command; process outside the RX path to avoid any blocking here
          cmdq_push((uint8_t)c);
          g_rx_count++;
          g_last_cmd = c;
          g_ui_dirty = 1;
        }
      } else if(n == SL_EAGAIN){
        // no data yet
      } else if(n <= 0){
        // error or closed
        if(ClientSock >= 0){ sl_Close(ClientSock); }
        ClientSock = -1;
        LED_GreenOff();
  // Ensure motors are silenced on drop
  Car_ProcessCommand('s');
  PWM_DisableOutputs();
  PWM_PinsToHiZ();
  motors_enabled = 0;
  // Do not allow PWM to be re-enabled by commands until next connection
  MotorControl_SetEnableAllowed(0);
  g_motion_timeout_ms = 0;
  g_led_blue_pulse_ms = 0;
  LED_BlueOff();
  g_rx_escape_state = 0;
        g_ui_dirty = 1;
      }
    }

    // cooperative tiny sleep and tick down timers
    if(g_reconnect_timer_ms){
      uint32_t step = (g_reconnect_timer_ms > 10) ? 10 : g_reconnect_timer_ms;
      delay_ms(step);
      g_reconnect_timer_ms -= step;
    } else {
      delay_ms(10);
    }

    // Consume and execute queued commands (decoupled from RX)
    if(!cmdq_is_empty()){
      // Process a few per tick to keep loop responsive
      for(int i=0; i<4 && !cmdq_is_empty(); i++){
        char c = (char)cmdq_pop();
        // Visual heartbeat on command: blue LED pulse ~50ms
        LED_BlueOn();
        g_led_blue_pulse_ms = 50;
        // Execute motor action
        Car_ProcessCommand((unsigned char)c);
        // Refresh motion safety timeout for active motion commands
        switch(c){
          case 'F': case 'f': case 'B': case 'b':
          case 'L': case 'l': case 'R': case 'r':
            g_motion_timeout_ms = MANUAL_MOTION_TIMEOUT_MS;
            break;
          case 'U': case 'u': case 'D': case 'd':
            g_motion_timeout_ms = SPEED_HOLD_TIMEOUT_MS;
            break;
          case 's':
            g_motion_timeout_ms = 0; // explicit stop clears watchdog
            break;
          default:
            break;
        }
      }
    }

    // Motion watchdog: stop motors if no motion command within timeout
    if(g_motion_timeout_ms){
      uint32_t dec = (g_motion_timeout_ms > 10) ? 10 : g_motion_timeout_ms;
      g_motion_timeout_ms -= dec;
      if(g_motion_timeout_ms == 0){
        Car_ProcessCommand('s');
        g_last_cmd = 's';
        LED_BlueOff();
        g_led_blue_pulse_ms = 0;
        g_ui_dirty = 1;
      }
    }

    // Blue LED pulse timing
    if(g_led_blue_pulse_ms){
      uint32_t dec2 = (g_led_blue_pulse_ms > 10) ? 10 : g_led_blue_pulse_ms;
      g_led_blue_pulse_ms -= dec2;
      if(g_led_blue_pulse_ms == 0){
        LED_BlueOff();
      }
    }

    // Update UI when needed
    if(g_ui_dirty){
      ui_render(ListenSock, ClientSock);
      g_ui_dirty = 0;
    }
  }
}

/*!
    \brief This function puts the device in its default state. It:
           - Set the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregister mDNS services

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
*/
static int32_t configureSimpleLinkToDefaultState(char *pConfig){
  SlVersionFull   ver = {0};
  UINT8           val = 1;
  UINT8           configOpt = 0;
  UINT8           configLen = 0;
  UINT8           power = 0;

  INT32           retVal = -1;
  INT32           mode = -1;

  mode = sl_Start(0, pConfig, 0);


    /* If the device is not in station-mode, try putting it in station-mode */
  if (ROLE_STA != mode){
    if (ROLE_AP == mode){
            /* If the device is in AP mode, we need to wait for this event before doing anything */
      while(!IS_IP_AQUIRED(g_Status));
    }

        /* Switch to STA role and restart */
    retVal = sl_WlanSetMode(ROLE_STA);

    retVal = sl_Stop(0xFF);

    retVal = sl_Start(0, pConfig, 0);

        /* Check if the device is in station again */
    if (ROLE_STA != retVal){
            /* We don't want to proceed if the device is not coming up in station-mode */
      return DEVICE_NOT_IN_STATION_MODE;
    }
  }
    /* Get the device's version-information */
  configOpt = SL_DEVICE_GENERAL_VERSION;
  configLen = sizeof(ver);
  retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (unsigned char *)(&ver));

    /* Set connection policy to Auto + SmartConfig (no Fast) to match earlier defaults */
  retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);

    /* Remove all profiles */
  retVal = sl_WlanProfileDel(0xFF);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
  retVal = sl_WlanDisconnect();
  if(0 == retVal){
        /* Wait */
     while(IS_CONNECTED(g_Status));
  }

    /* Enable DHCP client*/
  retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);

    /* Disable scan */
  configOpt = SL_SCAN_POLICY(0);
  retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
  power = WIFI_TX_POWER_DB_OFFSET;
  retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&power);

    /* Set PM policy to normal */
  retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);

    /* TBD - Unregister mDNS services */
  retVal = sl_NetAppMDNSUnRegisterService(0, 0);


  retVal = sl_Stop(0xFF);


  g_Status = 0;
  memset(&Recvbuff,0,MAX_RECV_BUFF_SIZE);
  memset(&SendBuff,0,MAX_SEND_BUFF_SIZE);
  memset(&HostName,0,MAX_HOSTNAME_SIZE);
  DestinationIP = 0;;
  SockID = 0;


  return retVal; /* Success */
}




/*
 * * ASYNCHRONOUS EVENT HANDLERS -- Start
 */

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent){
  switch(pWlanEvent->Event){
    case SL_WLAN_CONNECT_EVENT:
    {
      SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
      g_wifi_connecting = 0;
      g_reconnect_backoff_ms = 0;
      g_reconnect_timer_ms = 0;
      g_reconnect_attempts = 0;
  g_ui_dirty = 1;

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'sl_protocol_wlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * sl_protocol_wlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
    }
    break;

    case SL_WLAN_DISCONNECT_EVENT:
    {
      sl_protocol_wlanConnectAsyncResponse_t*  pEventData = NULL;

      CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
      CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_AQUIRED);
      g_wifi_disconnect_flag = 1; // notify main loop to tear down sockets and reconnect

      pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
      if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code){
        UART_OutString((uint8_t*)" Device disconnected (user request) \r\n");
      }
      else{
        UART_OutString((uint8_t*)" Device disconnected (ERROR) rc=");
        UART_OutUDec((uint32_t)pEventData->reason_code);
        UART_OutString((uint8_t*)"\r\n");
      }
      g_last_disc_reason = pEventData->reason_code;
      g_ui_dirty = 1;
    }
    break;

    default:
    {
      UART_OutString((uint8_t*)" [WLAN EVENT] Unexpected event \r\n");
    }
    break;
  }
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent){
  switch(pNetAppEvent->Event)
  {
    case SL_NETAPP_IPV4_ACQUIRED:
    {

      SET_STATUS_BIT(g_Status, STATUS_BIT_IP_AQUIRED);
      g_wifi_connecting = 0;
      g_reconnect_backoff_ms = 0; // reset backoff on successful IP acquire
  // capture IP bytes for UI
  SlIpV4AcquiredAsync_t *pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
  uint32_t ip = pEventData->ip;
  g_ip_b4 = (ip) & 0xFF;
  g_ip_b3 = (ip >> 8) & 0xFF;
  g_ip_b2 = (ip >> 16) & 0xFF;
  g_ip_b1 = (ip >> 24) & 0xFF;
  uint32_t gw = pEventData->gateway;
  g_gw_b4 = (gw) & 0xFF;
  g_gw_b3 = (gw >> 8) & 0xFF;
  g_gw_b2 = (gw >> 16) & 0xFF;
  g_gw_b1 = (gw >> 24) & 0xFF;
  g_ui_dirty = 1;
        // Print the acquired IP address
        uint8_t b4 = (ip) & 0xFF;
        uint8_t b3 = (ip >> 8) & 0xFF;
        uint8_t b2 = (ip >> 16) & 0xFF;
        uint8_t b1 = (ip >> 24) & 0xFF;
        // Note: SimpleLink reports IP in little-endian; above produces a.b.c.d correctly for CC3100
        // Format string locally (UART0 has no sprintf helpers)
        // Build as decimal numbers
        UART_OutString((uint8_t*)"IP: ");
        UART_OutUDec(b1); UART_OutChar('.');
        UART_OutUDec(b2); UART_OutChar('.');
        UART_OutUDec(b3); UART_OutChar('.');
        UART_OutUDec(b4); UART_OutString((uint8_t*)"\r\n");
        ST7735_OutString("IP: ");
		ST7735_OutUDec(b1); ST7735_OutChar('.');
        ST7735_OutUDec(b2); ST7735_OutChar('.');
        ST7735_OutUDec(b3); ST7735_OutChar('.');
        ST7735_OutUDec(b4); ST7735_OutString("\r\n");
        
        /*
             * Information about the connected AP's ip, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */

    }
    break;

    default:
    {
            UART_OutString((uint8_t*)" [NETAPP EVENT] Unexpected event \r\n");
    }
    break;
  }
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pServerEvent - Contains the relevant event information
    \param[in]      pServerResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse){
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
  UART_OutString((uint8_t*)" [HTTP EVENT] Unexpected event \r\n");
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent){
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
  UART_OutString((uint8_t*)" [GENERAL EVENT] \r\n");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock){
  switch( pSock->Event )
  {
    case SL_NETAPP_SOCKET_TX_FAILED:
    {
            /*
            * TX Failed
            *
            * Information about the socket descriptor and status will be
            * available in 'SlSockEventData_t' - Applications can use it if
            * required
            *
            * SlSockEventData_t *pEventData = NULL;
            * pEventData = & pSock->EventData;
            */
      switch( pSock->EventData.status )
      {
        case SL_ECLOSE:
          UART_OutString((uint8_t*)" [SOCK EVENT] Close socket operation failed to transmit all queued packets\r\n");
          break;


        default:
          UART_OutString((uint8_t*)" [SOCK EVENT] Unexpected event \r\n");
          break;
      }
    }
    break;

    default:
      UART_OutString((uint8_t*)" [SOCK EVENT] Unexpected event \r\n");
    break;
  }
}
/*
 * * ASYNCHRONOUS EVENT HANDLERS -- End
 */

// -------- helper: start a connect attempt now --------
static void wifi_attempt_connect(void){
  if(g_wifi_connecting){ return; }
  SlSecParams_t secParams;
  secParams.Key = PASSKEY;
  secParams.KeyLen = strlen(PASSKEY);
  secParams.Type = SEC_TYPE;
  UART_OutString((uint8_t*)"WiFi: attempting reconnect...\r\n");
  sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
  g_wifi_connecting = 1;
  // if this attempt fails, next attempt after current backoff (or 1s if zero)
  if(g_reconnect_backoff_ms == 0) g_reconnect_backoff_ms = 1000;
  g_reconnect_timer_ms = g_reconnect_backoff_ms;
  // bump attempts; after several tries, restart NWP cleanly
  if(g_reconnect_attempts < 255) g_reconnect_attempts++;
  if(g_reconnect_attempts >= 5){
    UART_OutString((uint8_t*)"WiFi: restarting NWP\r\n");
    sl_Stop(0xFF);
    char *pConfig = NULL;
    int32_t ret = sl_Start(0, pConfig, 0);
    if(ret != ROLE_STA){
      // try to force back to STA
      sl_WlanSetMode(ROLE_STA);
      sl_Stop(0xFF);
      sl_Start(0, pConfig, 0);
    }
    // Reapply policies similar to default state
    UINT8 configOpt = SL_SCAN_POLICY(0);
    sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 1, 0, 0, 1), NULL, 0);
    // Relauch connection immediately after restart
    sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    g_reconnect_attempts = 0; // reset after restart
  }
  g_ui_dirty = 1;
}

// -------- UI renderer: clear screen and display connection/server status --------
static void ui_render(int listenSock, int clientSock){
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetTextColor(ST7735_WHITE);
  ST7735_SetCursor(0,0);
  ST7735_OutString("WIPERITE WiFi");

  // WiFi status line
  ST7735_SetCursor(0,2);
  if(IS_CONNECTED(g_Status)){
    ST7735_OutString("WiFi: Connected");
  } else {
    ST7735_OutString("WiFi: Disconnected");
    ST7735_SetCursor(0,3);
    ST7735_OutString("rc="); ST7735_OutUDec(g_last_disc_reason);
  }

  // IP line
  ST7735_SetCursor(0,4);
  if(IS_IP_AQUIRED(g_Status)){
    ST7735_OutString("IP: ");
    ST7735_OutUDec(g_ip_b1); ST7735_OutChar('.');
    ST7735_OutUDec(g_ip_b2); ST7735_OutChar('.');
    ST7735_OutUDec(g_ip_b3); ST7735_OutChar('.');
    ST7735_OutUDec(g_ip_b4);
  } else {
    ST7735_OutString("IP: -");
  }

  // Server/listen state
  ST7735_SetCursor(0,6);
  if(listenSock >= 0){ ST7735_OutString("Server: Listening"); }
  else { ST7735_OutString("Server: Idle"); }

  ST7735_SetCursor(0,7);
  if(clientSock >= 0){ ST7735_OutString("Client: Connected"); }
  else { ST7735_OutString("Client: None"); }

  // Gateway info
  ST7735_SetCursor(0,8);
  ST7735_OutString("GW: ");
  if(IS_IP_AQUIRED(g_Status)){
    ST7735_OutUDec(g_gw_b1); ST7735_OutChar('.');
    ST7735_OutUDec(g_gw_b2); ST7735_OutChar('.');
    ST7735_OutUDec(g_gw_b3); ST7735_OutChar('.');
    ST7735_OutUDec(g_gw_b4);
		ST7735_OutString("\r\n");
  } else {
    ST7735_OutString("-");
		ST7735_OutString("\r\n");
  }

  // Reconnect/backoff info when not connected
  if(!IS_CONNECTED(g_Status) || !IS_IP_AQUIRED(g_Status)){
    ST7735_SetCursor(0,10);
    ST7735_OutString("Retry in: ");
    ST7735_OutUDec(g_reconnect_timer_ms);
    ST7735_OutString(" ms");
    ST7735_SetCursor(0,11);
    ST7735_OutString("Attempts: ");
    ST7735_OutUDec(g_reconnect_attempts);
  }

  // Last command line
  ST7735_SetCursor(0,13);
  ST7735_OutString("Last CMD: ");
  if(clientSock >= 0 && g_last_cmd){
    ST7735_OutChar(g_last_cmd);
  } else {
    ST7735_OutString("-");
  }

  // RX counter line
  ST7735_SetCursor(0,14);
  ST7735_OutString("RX: ");
  ST7735_OutUDec(g_rx_count);
}

// -------- helper: set socket non-blocking --------
static void socket_set_nonblocking(int sd){
  if(sd < 0) return;
  unsigned long enableOption = 1; // non-zero enables non-blocking
  sl_SetSockOpt(sd, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &enableOption, sizeof(enableOption));
}

// -------- helper: close sockets if open --------
static void sockets_close_if_open(int *pListenSock, int *pClientSock){
  if(pClientSock && *pClientSock >= 0){
    sl_Close(*pClientSock);
    *pClientSock = -1;
  }
  if(pListenSock && *pListenSock >= 0){
    sl_Close(*pListenSock);
    *pListenSock = -1;
  }
}

// -------- helper: ensure listening socket exists (non-blocking) --------
static int sockets_ensure_listening(int *pListenSock){
  if(!pListenSock) return -1;
  if(*pListenSock >= 0) return 0;
  SlSockAddrIn_t LocalAddr;
  INT32 ASize = sizeof(SlSockAddrIn_t);
  LocalAddr.sin_family = SL_AF_INET;
  LocalAddr.sin_port = sl_Htons(LISTEN_PORT);
  LocalAddr.sin_addr.s_addr = 0;
  int sd = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, 0);
  if(sd < 0) return -1;
  if(sl_Bind(sd, (SlSockAddr_t *)&LocalAddr, ASize) < 0){ sl_Close(sd); return -1; }
  if(sl_Listen(sd, 1) < 0){ sl_Close(sd); return -1; }
  socket_set_nonblocking(sd);
  *pListenSock = sd;
  return 0;
}


