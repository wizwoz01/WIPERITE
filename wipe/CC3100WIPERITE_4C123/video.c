#include <stdint.h>
#include <string.h>
#include "ST7735.h"
#include "video.h"
#include "UART0.h"
#include "LED.h"
#include "../cc3100/simplelink/include/simplelink.h"

#ifndef SL_EAGAIN
#define SL_EAGAIN (-11)
#endif

// Protocol constants
// Header: 'V','F', w_hi, w_lo, h_hi, h_lo
#define VIDEO_HDR0 'V'
#define VIDEO_HDR1 'F'

// Internal server state
static uint16_t g_vport = 5001;
static int g_vListenSock = -1;
static int g_vClientSock = -1;

// Frame parsing state
static uint8_t hdr[6];
static int hdr_idx = 0;
static uint16_t frame_w = 0;
static uint16_t frame_h = 0;
static uint32_t pixels_remaining = 0; // bytes remaining
static uint16_t row_y = 0;
static uint16_t row_bytes_expected = 0;
static uint16_t row_bytes_read = 0;
static uint8_t rowbuf[ST7735_TFTWIDTH * 2]; // max 128*2 bytes per row

static void socket_set_nonblocking(int sd){
  if(sd < 0) return;
  unsigned long enableOption = 1;
  sl_SetSockOpt(sd, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &enableOption, sizeof(enableOption));
}

void Video_Init(uint16_t vport){
  g_vport = vport ? vport : 5001;
  // Create non-blocking listen socket
  SlSockAddrIn_t LocalAddr;
  INT32 ASize = sizeof(SlSockAddrIn_t);
  LocalAddr.sin_family = SL_AF_INET;
  LocalAddr.sin_port = sl_Htons(g_vport);
  LocalAddr.sin_addr.s_addr = 0; // any
  g_vListenSock = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, 0);
  if(g_vListenSock < 0){ UART_OutString((uint8_t*)"Video: socket create failed\r\n"); return; }
  if(sl_Bind(g_vListenSock, (SlSockAddr_t *)&LocalAddr, ASize) < 0){ UART_OutString((uint8_t*)"Video: bind failed\r\n"); sl_Close(g_vListenSock); g_vListenSock = -1; return; }
  if(sl_Listen(g_vListenSock, 1) < 0){ UART_OutString((uint8_t*)"Video: listen failed\r\n"); sl_Close(g_vListenSock); g_vListenSock = -1; return; }
  socket_set_nonblocking(g_vListenSock);
  UART_OutString((uint8_t*)"Video: listening on TCP port "); UART_OutUDec(g_vport); UART_OutString((uint8_t*)"\r\n");
}

// Reset frame state
static void video_reset_frame(void){
  hdr_idx = 0; frame_w = 0; frame_h = 0; pixels_remaining = 0; row_y = 0;
  row_bytes_expected = 0; row_bytes_read = 0;
}

void Video_Poll(void){
  // Accept client if none
  if(g_vListenSock >= 0 && g_vClientSock < 0){
    SlSockAddrIn_t ClientAddr; SlSocklen_t ClientLen = sizeof(ClientAddr);
    int sd = sl_Accept(g_vListenSock, (SlSockAddr_t *)&ClientAddr, &ClientLen);
    if(sd >= 0){
      g_vClientSock = sd;
      socket_set_nonblocking(g_vClientSock);
      int ka = 1; sl_SetSockOpt(g_vClientSock, SL_SOL_SOCKET, SL_SO_KEEPALIVE, &ka, sizeof(ka));
      UART_OutString((uint8_t*)"Video: client connected\r\n");
      video_reset_frame();
    }
  }

  // Process client data if connected
  if(g_vClientSock >= 0){
    // Read header first
    if(hdr_idx < 6){
      int n = sl_Recv(g_vClientSock, (char*)hdr + hdr_idx, 6 - hdr_idx, 0);
      if(n > 0){
        hdr_idx += n;
        if(hdr_idx == 6){
          if(hdr[0] != VIDEO_HDR0 || hdr[1] != VIDEO_HDR1){
            UART_OutString((uint8_t*)"Video: bad header, closing\r\n");
            sl_Close(g_vClientSock); g_vClientSock = -1; return;
          }
          frame_w = ((uint16_t)hdr[2] << 8) | hdr[3];
          frame_h = ((uint16_t)hdr[4] << 8) | hdr[5];
          if(frame_w == 0 || frame_h == 0 || frame_w > ST7735_TFTWIDTH || frame_h > ST7735_TFTHEIGHT){
            UART_OutString((uint8_t*)"Video: invalid WxH\r\n");
            sl_Close(g_vClientSock); g_vClientSock = -1; return;
          }
          pixels_remaining = (uint32_t)frame_w * (uint32_t)frame_h * 2u; // bytes
          row_y = 0;
          row_bytes_expected = frame_w * 2u;
          row_bytes_read = 0;
        }
      } else if(n == SL_EAGAIN){
        // no data
      } else {
        sl_Close(g_vClientSock); g_vClientSock = -1; UART_OutString((uint8_t*)"Video: client closed\r\n");
        return;
      }
      return; // wait for full header
    }

    // Stream rows
    while(pixels_remaining && g_vClientSock >= 0){
      // Read remaining bytes for current row
      int need = (int)(row_bytes_expected - row_bytes_read);
      if(need > 0){
        int n = sl_Recv(g_vClientSock, (char*)rowbuf + row_bytes_read, need, 0);
        if(n > 0){
          row_bytes_read += (uint16_t)n;
          pixels_remaining -= (uint32_t)n;
        } else if(n == SL_EAGAIN){
          break; // wait for more
        } else {
          sl_Close(g_vClientSock); g_vClientSock = -1; UART_OutString((uint8_t*)"Video: recv error\r\n");
          return;
        }
      }
      // If row complete, blit it
      if(row_bytes_read == row_bytes_expected){
        ST7735_BlitRGB565(0, row_y, frame_w, 1, rowbuf);
        row_y++;
        row_bytes_read = 0;
        // If finished all rows, reset for next frame
        if(row_y >= frame_h){
          video_reset_frame();
          break;
        }
      } else {
        break; // partial row; wait for more
      }
    }
  }
}
