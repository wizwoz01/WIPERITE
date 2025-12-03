#ifndef VIDEO_H
#define VIDEO_H
#include <stdint.h>

// Initialize video server on given port (default 5001)
void Video_Init(uint16_t vport);
// Poll the video server state; non-blocking processing
void Video_Poll(void);

#endif // VIDEO_H
