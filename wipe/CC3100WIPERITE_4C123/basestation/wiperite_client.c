// WIPERITE base-station client for Zynq-7000 (PetaLinux)
// Connects to TM4C/CC3100 TCP server on port 5000 and sends single-letter commands.
// Interactive mode supports WASD/arrow keys and specific letters; 'q' quits.
// One-shot mode supports sending a string via -c "...".
//
// Enhancements:
// - ncurses TUI with status, last command, server messages (disable with NO_CURSES=1)
// - Auto-reconnect on socket drop with exponential backoff, preserves UI

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#define DEFAULT_PORT 5000

#ifndef NO_CURSES
#include <ncurses.h>
#endif

#ifdef NO_CURSES
static struct termios orig_termios;
static bool raw_enabled = false;
#endif
static volatile sig_atomic_t g_stop = 0;

typedef struct {
    const char *ip;
    uint16_t port;
    int sock;
    int reconnects;
    char last_cmd;
    char last_key; // last raw key pressed (to distinguish Space vs b/B when both map to 'S')
    char last_server_msg[128];
    bool connected;
} AppState;

typedef struct {
    bool enabled;
    char dev_path[128];
    int vfd;
    int width;
    int height;
    int frame_bytes;
    int y_threshold;   // Y luma threshold for dark-on-white tracking
    int sample_step;   // process every Nth pixel horizontally and vertically
    int min_count;     // minimum qualifying pixel count to consider valid
    int margin;        // deadband (pixels) around center
    bool auto_send;    // if true, send guidance commands automatically
    int last_cx;
    int last_cy;
    int last_count;
    char last_auto_cmd;
    struct timespec last_send_ts;
    int send_interval_ms; // min interval between auto sends
    unsigned char *framebuf; // allocated once
    // ST7735 streaming
    bool stream_st7735;     // enable streaming to TM4C ST7735 over a second TCP port
    uint16_t video_port;    // default 5001
    int vsock;              // video socket
    int out_w;              // output width (e.g., 128)
    int out_h;              // output height (e.g., 160)
    bool rotate90;          // rotate output 90 degrees
    int vsend_interval_ms;  // min interval between video frame sends
    struct timespec last_vsend_ts;
    uint16_t *vidbuf;       // RGB565 buffer for one output frame
} TrackState;

// Configurable key mapping: ASCII key -> command letter (F,B,L,R,S,U,D,8,C,Z,Q)
typedef struct {
    unsigned char map[256];
} KeyMap;

static void keymap_init_defaults(KeyMap *km) {
    memset(km->map, 0, sizeof(km->map));
    // Default bindings
    km->map[(unsigned char)'w'] = 'F'; km->map[(unsigned char)'W'] = 'F';
    km->map[(unsigned char)'s'] = 'B'; km->map[(unsigned char)'S'] = 'B';
    km->map[(unsigned char)'a'] = 'L'; km->map[(unsigned char)'A'] = 'L';
    km->map[(unsigned char)'d'] = 'R'; km->map[(unsigned char)'D'] = 'R';
    // Space = Stop using 'S' to match legacy server behavior
    km->map[(unsigned char)' '] = 'S';
    // Square on b/B by default, sending 'S' (server interprets this accordingly)
    km->map[(unsigned char)'b'] = 'S'; km->map[(unsigned char)'B'] = 'S';
    km->map[(unsigned char)'u'] = 'U'; km->map[(unsigned char)'U'] = 'U';
    km->map[(unsigned char)'j'] = 'D'; km->map[(unsigned char)'J'] = 'D';
    km->map[(unsigned char)'8'] = '8';
    km->map[(unsigned char)'c'] = 'C'; km->map[(unsigned char)'C'] = 'C';
    km->map[(unsigned char)'z'] = 'Z'; km->map[(unsigned char)'Z'] = 'Z';
    km->map[(unsigned char)'q'] = 'Q'; km->map[(unsigned char)'Q'] = 'Q';
}

static int keymap_set(KeyMap *km, const char *spec) {
    // spec format: "k:X" where k is a single character key, X is one-letter command
    const char *colon = strchr(spec, ':');
    if (!colon || colon == spec || colon[1] == '\0') return -1;
    unsigned char key = (unsigned char)spec[0];
    unsigned char cmd = (unsigned char)colon[1];
    const char *valid = "FBLRSUD8CZQH"; // include 'H' (Halt/Stop)
    if (!strchr(valid, (cmd >= 'a' && cmd <= 'z') ? (cmd - 32) : cmd)) return -2;
    if (cmd >= 'a' && cmd <= 'z') cmd -= 32; // uppercase
    km->map[key] = cmd;
    return 0;
}

#ifdef NO_CURSES
static void die(const char *msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}
#endif

#ifdef NO_CURSES
static void disable_raw_mode(void) {
    if (raw_enabled) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
        raw_enabled = false;
    }
}
#endif

#ifdef NO_CURSES
static void enable_raw_mode(void) {
    if (tcgetattr(STDIN_FILENO, &orig_termios) == -1) die("tcgetattr");
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    raw.c_iflag &= ~(IXON | ICRNL | BRKINT | INPCK | ISTRIP);
    raw.c_cflag |= (CS8);
    raw.c_oflag &= ~(OPOST);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1; // 100ms read timeout
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) == -1) die("tcsetattr");
    raw_enabled = true;
}
#endif

static void on_signal(int sig) {
    (void)sig;
    g_stop = 1;
}

static void usage(const char *prog) {
    fprintf(stderr, "Usage: %s [<ip> [port]] [-c COMMANDS] [--track [options]] [--video [options]]\n", prog);
    fputs("  ip       Target TM4C IP (as shown on LCD/UART)\n", stderr);
    fprintf(stderr, "  port     TCP port (default %d)\n", DEFAULT_PORT);
    fputs("  -c str   Send command string then exit (non-interactive)\n\n", stderr);
    fputs("Tracking options (OV7670 via UYVY stream):\n", stderr);
    fputs("  --track                 Enable whiteboard tracking (dark-on-white)\n", stderr);
    fputs("  --dev PATH              Video device path (default /dev/xillybus_read_32)\n", stderr);
    fputs("  --size WxH              Frame size, default 640x480\n", stderr);
    fputs("  --ythr N                Y luma threshold for dark (default 60)\n", stderr);
    fputs("  --step N                Sampling step (default 2)\n", stderr);
    fputs("  --min N                 Min pixel hits to validate (default 500)\n", stderr);
    fputs("  --margin N              Deadband margin in pixels (default 24)\n", stderr);
    fputs("  --auto                  Autopilot: send guidance cmds automatically\n\n", stderr);
    fputs("ST7735 video stream (RGB565 over second TCP port):\n", stderr);
    fputs("  --video                 Enable streaming to TM4C ST7735 (requires <ip>)\n", stderr);
    fputs("  --vport N               Video TCP port (default 5001)\n", stderr);
    fputs("  --vsize WxH             ST7735 target size (default 128x160)\n", stderr);
    fputs("  --vfps N                Max video FPS (default 10)\n", stderr);
    fputs("  --rotate                Rotate video 90 degrees\n\n", stderr);
    fputs("Interactive controls (no -c):\n", stderr);
    fputs("  w or Up Arrow     -> F (Forward)\n", stderr);
    fputs("  s or Down Arrow   -> B (Back)\n", stderr);
    fputs("  a or Left Arrow   -> L (Left)\n", stderr);
    fputs("  d or Right Arrow  -> R (Right)\n", stderr);
    fputs("  space             -> S (Stop)\n", stderr);
    fputs("  u                 -> U (Speed Up / lift)\n", stderr);
    fputs("  j                 -> D (Slow Down / lower)\n", stderr);
    fputs("  8                 -> 8 (custom)\n", stderr);
    fputs("  c                 -> C (custom)\n", stderr);
    fputs("  z                 -> Z (custom)\n", stderr);
    fputs("  q                 -> Q (quit)\n\n", stderr);
    fputs("Runtime toggles:\n", stderr);
    fputs("  t                 Toggle Auto (tracking guidance) ON/OFF\n", stderr);
    fputs("  v                 Toggle Video streaming ON/OFF\n\n", stderr);
    fputs("UI: ncurses TUI enabled by default (disable with NO_CURSES=1 at build).\n", stderr);
    fputs("Notes:\n", stderr);
    fputs("  - Without <ip>, runs tracking-only (no CC3100 connection).\n", stderr);
    fputs("  - With <ip>, combines tracking with CC3100 control.\n", stderr);
}

static int connect_with_timeout(const char *ip, uint16_t port, int timeout_sec) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) { perror("socket"); return -1; }

    // Set non-blocking for connect timeout
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0) { perror("fcntl get"); close(sock); return -1; }
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) { perror("fcntl set"); close(sock); return -1; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
        fprintf(stderr, "Invalid IP address: %s\n", ip);
        close(sock);
        return -1;
    }

    int rc = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    if (rc < 0 && errno != EINPROGRESS) { perror("connect"); close(sock); return -1; }

    if (rc != 0) {
        fd_set wfds; FD_ZERO(&wfds); FD_SET(sock, &wfds);
        struct timeval tv = { .tv_sec = timeout_sec, .tv_usec = 0 };
        rc = select(sock + 1, NULL, &wfds, NULL, &tv);
        if (rc <= 0) {
            fprintf(stderr, "Connect timeout or error\n");
            close(sock);
            return -1;
        }
        int so_error = 0; socklen_t len = sizeof(so_error);
        if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0 || so_error != 0) {
            errno = so_error;
            perror("connect (post-select)");
            close(sock);
            return -1;
        }
    }

    // Restore blocking mode
    if (fcntl(sock, F_SETFL, flags) < 0) { perror("fcntl restore"); close(sock); return -1; }

    // small recv timeout
    struct timeval rcv_to = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcv_to, sizeof(rcv_to));

    return sock;
}

static char map_key(const KeyMap *km, unsigned char k, const unsigned char *esc_seq, size_t esc_len) {
    // Arrow keys produce ESC [ A/B/C/D
    if (k == '\x1b' && esc_len >= 2 && esc_seq[0] == '[') {
        unsigned char code = esc_seq[1];
        if (code == 'A') return 'F'; // Up
        if (code == 'B') return 'B'; // Down
        if (code == 'C') return 'R'; // Right
        if (code == 'D') return 'L'; // Left
    }
    // Configured direct mapping
    return km->map[k]; // 0 if unmapped -> ignore
}

#ifndef NO_CURSES
// -------- ncurses UI helpers --------
static bool ui_inited = false;
static void ui_init(void) {
    if (ui_inited) return;
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    ui_inited = true;
}
static void ui_shutdown(void) {
    if (ui_inited) { endwin(); ui_inited = false; }
}
static void ui_draw(const AppState *st) {
    int rows, cols; getmaxyx(stdscr, rows, cols);
    erase();
    mvprintw(0, 0, "WIPERITE Base Station  |  Target %s:%u  |  Status: %s  |  Reconnects: %d",
             st->ip, st->port, st->connected ? "CONNECTED" : "OFFLINE", st->reconnects);
    mvhline(1, 0, '-', cols);
    mvprintw(2, 0, "Controls: WASD/Arrows move, Space=Stop, B/b=Square, U/J speed up/down, 8/C/Z custom, Q quit");
    const char *label = "-";
    switch (st->last_cmd) {
        case 'F': label = "Forward"; break;
        case 'B': label = "Back"; break;
        case 'L': label = "Left"; break;
        case 'R': label = "Right"; break;
        case 's': label = "Stop"; break; // backward compatibility if sent
        case 'S':
            if (st->last_key == ' ') label = "Stop"; // space sent 'S'
            else if (st->last_key == 'b' || st->last_key == 'B') label = "Square"; // b/B sent 'S'
            else label = "S";
            break;
        case 'U': label = "SpeedUp"; break;
        case 'D': label = "SlowDown"; break;
        case '8': label = "Eight"; break;
        case 'C': label = "Custom C"; break;
        case 'Z': label = "Custom Z"; break;
        case 'Q': label = "Quit"; break;
        default: label = "-"; break;
    }
    if (st->last_cmd) {
    mvprintw(4, 0, "Last command: %s (%c)", label, st->last_cmd);
    } else {
        mvprintw(4, 0, "Last command: -");
    }
    mvprintw(6, 0, "Server: %s", st->last_server_msg[0] ? st->last_server_msg : "<none>");
    mvprintw(rows - 1, 0, "Press Q to quit");
    refresh();
}
static int ui_get_key(void) {
    int ch = getch();
    return ch; // ERR if none
}
#endif

// -------- non-curses raw input helpers --------
#ifdef NO_CURSES
static int kbd_read_raw(unsigned char *out) {
    unsigned char ch; ssize_t n = read(STDIN_FILENO, &ch, 1);
    if (n == 1) { *out = ch; return 1; }
    return 0;
}
#endif

// Returns: 0 continue, 1 user-quit, -1 socket dropped
static int interactive_session(AppState *st, const KeyMap *km) {
#ifndef NO_CURSES
    ui_draw(st);
#else
    printf("Connected. Interactive control ready. Press 'q' to quit.\n");
#endif

    // Try to read greeting without blocking the loop
    char gbuf[256];
    ssize_t gr = recv(st->sock, gbuf, sizeof(gbuf)-1, MSG_DONTWAIT);
    if (gr > 0) {
        gbuf[gr] = '\0';
#ifndef NO_CURSES
        size_t cplen = (size_t)gr;
        if (cplen > sizeof(st->last_server_msg)-1) cplen = sizeof(st->last_server_msg)-1;
        memcpy(st->last_server_msg, gbuf, cplen);
        st->last_server_msg[cplen] = '\0';
        ui_draw(st);
#else
        printf("Server: %s", gbuf);
#endif
    }

#ifdef NO_CURSES
    enable_raw_mode();
    atexit(disable_raw_mode);
    unsigned char esc_buf[8] = {0};
    size_t esc_len = 0; bool in_escape = false;
#endif

    while (!g_stop) {
        // Socket readability wait with small timeout to pace the loop
        fd_set rfds; FD_ZERO(&rfds); FD_SET(st->sock, &rfds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 }; // 100ms
        int rc = select(st->sock + 1, &rfds, NULL, NULL, &tv);
        if (rc < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (FD_ISSET(st->sock, &rfds)) {
            char rbuf[256];
            ssize_t n = recv(st->sock, rbuf, sizeof(rbuf)-1, 0);
            if (n <= 0) {
#ifndef NO_CURSES
                snprintf(st->last_server_msg, sizeof(st->last_server_msg), "<connection dropped>");
                ui_draw(st);
#else
                printf("\nServer closed connection.\n");
#endif
                return -1; // dropped
            }
            rbuf[n] = '\0';
#ifndef NO_CURSES
            // Keep tail that fits
            size_t len = (size_t)n;
            const char *start = rbuf;
            if (len > sizeof(st->last_server_msg)-1) {
                start = rbuf + (len - (sizeof(st->last_server_msg)-1));
                len = sizeof(st->last_server_msg)-1;
            }
            memcpy(st->last_server_msg, start, len);
            st->last_server_msg[len] = '\0';
            ui_draw(st);
#else
            printf("%s", rbuf); fflush(stdout);
#endif
    }

        // Process keyboard
#ifndef NO_CURSES
        for (;;) {
            int ch = ui_get_key();
            if (ch == ERR) break;
            char cmd = 0;
            int arrow = 0;
            switch (ch) {
                case KEY_UP: cmd = 'F'; arrow = 1; break;
                case KEY_DOWN: cmd = 'B'; arrow = 1; break;
                case KEY_LEFT: cmd = 'L'; arrow = 1; break;
                case KEY_RIGHT: cmd = 'R'; arrow = 1; break;
                default:
                    if (ch >= 0 && ch <= 255) cmd = map_key(km, (unsigned char)ch, NULL, 0);
                    break;
            }
            if (cmd) {
                st->last_key = arrow ? 0 : (char)((ch >= 0 && ch <= 255) ? ch : 0);
                st->last_cmd = cmd;
                ui_draw(st);
                if (send(st->sock, &cmd, 1, 0) != 1) return -1; // treat as drop
                if (cmd == 'Q') return 1; // user quit
            }
        }
#else
        if (1) {
            unsigned char ch;
            while (kbd_read_raw(&ch)) {
                static unsigned char esc_buf2[8]; static size_t esc_len2 = 0; static bool in_esc2 = false;
                if (!in_esc2) {
                    if (ch == '\x1b') { in_esc2 = true; esc_len2 = 0; continue; }
                    char cmd = map_key(km, ch, NULL, 0);
                    if (cmd) {
                        st->last_key = (char)ch;
                        st->last_cmd = cmd;
                        if (send(st->sock, &cmd, 1, 0) != 1) return -1;
                        if (cmd == 'Q') return 1;
                    }
                } else {
                    if (esc_len2 < sizeof(esc_buf2)) esc_buf2[esc_len2++] = ch;
                    if (esc_len2 >= 2) {
                        char cmd = map_key(km, '\x1b', esc_buf2, esc_len2);
                        if (cmd) {
                            st->last_key = 0; // arrow/escape-originated
                            st->last_cmd = cmd;
                            if (send(st->sock, &cmd, 1, 0) != 1) return -1;
                        }
                        in_esc2 = false; esc_len2 = 0;
                    }
                }
            }
        }
#endif
    }
    return 1; // stop requested
}

static ssize_t read_full(int fd, void *buf, size_t len) {
    size_t got = 0;
    while (got < len) {
        ssize_t n = read(fd, (char*)buf + got, len - got);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) break;
        got += (size_t)n;
    }
    return (ssize_t)got;
}

static void trackstate_init_defaults(TrackState *ts) {
    memset(ts, 0, sizeof(*ts));
    ts->enabled = false;
    snprintf(ts->dev_path, sizeof(ts->dev_path), "%s", "/dev/xillybus_read_32");
    ts->vfd = -1;
    ts->width = 640;
    ts->height = 480;
    ts->frame_bytes = ts->width * ts->height * 2; // UYVY 16bpp
    ts->y_threshold = 60;
    ts->sample_step = 2;
    ts->min_count = 500;
    ts->margin = 24;
    ts->auto_send = false;
    ts->last_cx = ts->last_cy = -1;
    ts->last_count = 0;
    ts->last_auto_cmd = 0;
    ts->send_interval_ms = 100; // 10Hz max
    ts->framebuf = NULL;
    ts->stream_st7735 = false;
    ts->video_port = 5001;
    ts->vsock = -1;
    ts->out_w = 128;
    ts->out_h = 160;
    ts->rotate90 = false;
    ts->vsend_interval_ms = 100; // ~10 fps
    ts->vidbuf = NULL;
    ts->last_vsend_ts.tv_sec = 0; ts->last_vsend_ts.tv_nsec = 0;
}

static int cmp_timespec_ms(struct timespec *a, struct timespec *b) {
    long da = (long)(a->tv_sec - b->tv_sec) * 1000L + (long)((a->tv_nsec - b->tv_nsec) / 1000000L);
    return (int)da;
}

static void timespec_now(struct timespec *ts) {
    clock_gettime(CLOCK_MONOTONIC, ts);
}

static int connect_with_timeout_port(const char *ip, uint16_t port, int timeout_sec) {
    return connect_with_timeout(ip, port, timeout_sec);
}

static char guidance_from_centroid(const TrackState *ts, int cx, int cy) {
    if (cx < 0 || cy < 0) return 0;
    int ex = cx - ts->width / 2;
    int ey = cy - ts->height / 2;
    if (ex < -ts->margin) return 'L';
    if (ex >  ts->margin) return 'R';
    if (ey < -ts->margin) return 'F';
    if (ey >  ts->margin) return 'B';
    return 'S'; // inside deadband -> stop/hold (legacy stop)
}

static void process_frame_dark_on_white(TrackState *ts) {
    // UYVY: bytes per two pixels: U Y0 V Y1
    const int w = ts->width, h = ts->height, step = ts->sample_step;
    unsigned char *p = ts->framebuf;
    long sumx = 0, sumy = 0; int count = 0;
    for (int y = 0; y < h; y += step) {
        int row_off = y * w * 2;
        for (int x = 0; x < w; x += 2*step) {
            // Two pixels share U,V; examine both Y's
            unsigned char Y0 = p[row_off + x*2 + 1];
            unsigned char Y1 = p[row_off + x*2 + 3];
            if (Y0 < ts->y_threshold) { sumx += x; sumy += y; count++; }
            if (x + 1 < w && Y1 < ts->y_threshold) { sumx += (x+1); sumy += y; count++; }
        }
    }
    ts->last_count = count * (step*step); // approx scaled population
    if (count >= 1 && count >= ts->min_count / (step*step)) {
        int cx = (int)(sumx / count);
        int cy = (int)(sumy / count);
        ts->last_cx = cx;
        ts->last_cy = cy;
    } else {
        ts->last_cx = ts->last_cy = -1;
    }
}

static int open_video_if_needed(TrackState *ts) {
    if (!ts->enabled && !ts->stream_st7735) return 0;
    if (ts->vfd >= 0) return 0;
    ts->frame_bytes = ts->width * ts->height * 2;
    ts->vfd = open(ts->dev_path, O_RDONLY);
    if (ts->vfd < 0) { perror("open video device"); return -1; }
    if (!ts->framebuf) {
        ts->framebuf = (unsigned char*)malloc((size_t)ts->frame_bytes);
        if (!ts->framebuf) { perror("malloc frame"); close(ts->vfd); ts->vfd = -1; return -1; }
    }
    timespec_now(&ts->last_send_ts);
    return 0;
}

static void close_video(TrackState *ts) {
    if (ts->vfd >= 0) { close(ts->vfd); ts->vfd = -1; }
    if (ts->framebuf) { free(ts->framebuf); ts->framebuf = NULL; }
    if (ts->vidbuf) { free(ts->vidbuf); ts->vidbuf = NULL; }
    if (ts->vsock >= 0) { close(ts->vsock); ts->vsock = -1; }
}

// Build a color RGB565 frame from UYVY source, optionally rotated 90 degrees
static void uyvy_to_rgb565_color_downscale(const TrackState *ts, uint16_t *out, int outw, int outh, bool rotate90) {
    const int srcw = ts->width, srch = ts->height;
    const unsigned char *src = ts->framebuf;
    for (int oy = 0; oy < outh; ++oy) {
        for (int ox = 0; ox < outw; ++ox) {
            int sx = (ox * srcw) / outw;
            int sy = (oy * srch) / outh;
            int idx = sy * srcw * 2 + (sx & ~1) * 2; // two pixels per 4 bytes
            int U = src[idx + 0] - 128;
            int V = src[idx + 2] - 128;
            int Y = src[idx + ((sx & 1) ? 3 : 1)];
            // BT.601 approximate conversion to RGB
            int C = Y - 16; if (C < 0) C = 0;
            int R = (298 * C + 409 * V + 128) >> 8;
            int G = (298 * C - 100 * U - 208 * V + 128) >> 8;
            int B = (298 * C + 516 * U + 128) >> 8;
            if (R < 0) R = 0; if (R > 255) R = 255;
            if (G < 0) G = 0; if (G > 255) G = 255;
            if (B < 0) B = 0; if (B > 255) B = 255;
            uint16_t r5 = (uint16_t)(R >> 3);
            uint16_t g6 = (uint16_t)(G >> 2);
            uint16_t b5 = (uint16_t)(B >> 3);
            uint16_t pix = (uint16_t)((r5 << 11) | (g6 << 5) | b5);
            int out_index;
            if (rotate90) {
                // rotate 90 deg: (ox,oy) -> (oy, outw-1-ox)
                out_index = oy * outw + (outw - 1 - ox);
            } else {
                out_index = oy * outw + ox;
            }
            out[out_index] = pix;
        }
    }
}

static int ensure_video_socket(TrackState *ts, const char *ip) {
    if (!ts->stream_st7735) return 0;
    if (ts->vsock >= 0) return 0;
    ts->vsock = connect_with_timeout_port(ip, ts->video_port, 5);
    if (ts->vsock < 0) return -1;
    return 0;
}

static int interactive_session_trk(AppState *st, const KeyMap *km, TrackState *ts) {
#ifndef NO_CURSES
    ui_draw(st);
#else
    printf("Connected. Interactive control ready. Press 'q' to quit.\n");
#endif

    char gbuf[256];
    ssize_t gr = recv(st->sock, gbuf, sizeof(gbuf)-1, MSG_DONTWAIT);
    if (gr > 0) {
        gbuf[gr] = '\0';
#ifndef NO_CURSES
        size_t cplen = (size_t)gr;
        if (cplen > sizeof(st->last_server_msg)-1) cplen = sizeof(st->last_server_msg)-1;
        memcpy(st->last_server_msg, gbuf, cplen);
        st->last_server_msg[cplen] = '\0';
        ui_draw(st);
#else
        printf("Server: %s", gbuf);
#endif
    }

#ifdef NO_CURSES
    enable_raw_mode();
    atexit(disable_raw_mode);
#endif

    if (open_video_if_needed(ts) != 0) {
        // proceed without tracking if open failed
        ts->enabled = false;
    }

    // Ensure video stream socket if requested
    if (ts->stream_st7735) {
        (void)ensure_video_socket(ts, st->ip);
        if (!ts->vidbuf) {
            ts->vidbuf = (uint16_t*)malloc((size_t)(ts->out_w * ts->out_h * 2));
            if (!ts->vidbuf) ts->stream_st7735 = false;
        }
        timespec_now(&ts->last_vsend_ts);
    }

    while (!g_stop) {
        fd_set rfds; FD_ZERO(&rfds);
        FD_SET(st->sock, &rfds);
        int maxfd = st->sock;
        if ((ts->enabled || ts->stream_st7735) && ts->vfd >= 0) { FD_SET(ts->vfd, &rfds); if (ts->vfd > maxfd) maxfd = ts->vfd; }
        struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 }; // 100ms
        int rc = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if (rc < 0) { if (errno == EINTR) continue; close_video(ts); return -1; }

        if (FD_ISSET(st->sock, &rfds)) {
            char rbuf[256];
            ssize_t n = recv(st->sock, rbuf, sizeof(rbuf)-1, 0);
            if (n <= 0) {
#ifndef NO_CURSES
                snprintf(st->last_server_msg, sizeof(st->last_server_msg), "<connection dropped>");
                ui_draw(st);
#else
                printf("\nServer closed connection.\n");
#endif
                close_video(ts);
                return -1;
            }
            rbuf[n] = '\0';
#ifndef NO_CURSES
            size_t len = (size_t)n;
            const char *start = rbuf;
            if (len > sizeof(st->last_server_msg)-1) { start = rbuf + (len - (sizeof(st->last_server_msg)-1)); len = sizeof(st->last_server_msg)-1; }
            memcpy(st->last_server_msg, start, len);
            st->last_server_msg[len] = '\0';
            ui_draw(st);
#else
            printf("%s", rbuf); fflush(stdout);
#endif
        }

        if ((ts->enabled || ts->stream_st7735) && ts->vfd >= 0 && FD_ISSET(ts->vfd, &rfds)) {
            ssize_t n = read_full(ts->vfd, ts->framebuf, (size_t)ts->frame_bytes);
            if (n != ts->frame_bytes) {
#ifndef NO_CURSES
                snprintf(st->last_server_msg, sizeof(st->last_server_msg), "Video read error");
                ui_draw(st);
#else
                fprintf(stderr, "Video read error or EOF\n");
#endif
                close_video(ts);
            } else {
                if (ts->enabled) {
                    process_frame_dark_on_white(ts);
                }
                char auto_cmd = 0;
                if (ts->enabled && ts->auto_send) {
                    auto_cmd = guidance_from_centroid(ts, ts->last_cx, ts->last_cy);
                }
#ifndef NO_CURSES
                // Render basic tracking HUD on the status line
                mvprintw(7, 0, "Tracking: %s  CX:%d CY:%d Count:%d  Auto:%s Cmd:%c        ",
                         ts->enabled ? "ON" : "OFF", ts->last_cx, ts->last_cy, ts->last_count,
                         ts->auto_send ? "ON" : "OFF", auto_cmd ? auto_cmd : '-');
                refresh();
#else
                static int tick = 0; if ((tick++ % 10) == 0) {
                    printf("\n[Track] CX=%d CY=%d Count=%d Cmd=%c\n", ts->last_cx, ts->last_cy, ts->last_count, auto_cmd ? auto_cmd : '-');
                }
#endif
                if (ts->auto_send && auto_cmd) {
                    struct timespec now; timespec_now(&now);
                    if (cmp_timespec_ms(&now, &ts->last_send_ts) >= ts->send_interval_ms) {
                        if (auto_cmd != st->last_cmd || auto_cmd == 'S') {
                            if (send(st->sock, &auto_cmd, 1, 0) == 1) {
                                st->last_cmd = auto_cmd;
                                ts->last_auto_cmd = auto_cmd;
                                ts->last_send_ts = now;
                            }
                        }
                    }
                }

                // ST7735 streaming: downscale (color) and send over video socket
                if (ts->stream_st7735 && ts->vsock >= 0 && ts->vidbuf) {
                    struct timespec now; timespec_now(&now);
                    if (cmp_timespec_ms(&now, &ts->last_vsend_ts) >= ts->vsend_interval_ms) {
                        uyvy_to_rgb565_color_downscale(ts, ts->vidbuf, ts->out_w, ts->out_h, ts->rotate90);
                        uint16_t w = (uint16_t)ts->out_w, h = (uint16_t)ts->out_h;
                        uint16_t payload = (uint16_t)(w * h * 2);
                        unsigned char hdr[6];
                        hdr[0] = 'V'; hdr[1] = 'F';
                        hdr[2] = (unsigned char)(w >> 8); hdr[3] = (unsigned char)(w & 0xFF);
                        hdr[4] = (unsigned char)(h >> 8); hdr[5] = (unsigned char)(h & 0xFF);
                        ssize_t hn = send(ts->vsock, hdr, sizeof(hdr), 0);
                        if (hn == (ssize_t)sizeof(hdr)) {
                            ssize_t pn = send(ts->vsock, (const char*)ts->vidbuf, payload, 0);
                            if (pn != payload) {
                                // drop and try reconnect later
                                close(ts->vsock); ts->vsock = -1;
                            } else {
                                ts->last_vsend_ts = now;
                            }
                        } else {
                            close(ts->vsock); ts->vsock = -1;
                        }
                    }
                    // attempt lazy reconnect if needed
                    if (ts->vsock < 0) (void)ensure_video_socket(ts, st->ip);
                }
            }
        }

        // Keyboard handling (same as before)
#ifndef NO_CURSES
        for (;;) {
            int ch = ui_get_key();
            if (ch == ERR) break;
            char cmd = 0; int arrow = 0;
            switch (ch) {
                case KEY_UP: cmd = 'F'; arrow = 1; break;
                case KEY_DOWN: cmd = 'B'; arrow = 1; break;
                case KEY_LEFT: cmd = 'L'; arrow = 1; break;
                case KEY_RIGHT: cmd = 'R'; arrow = 1; break;
                // Runtime toggles
                case 't': case 'T':
                    ts->auto_send = !ts->auto_send;
                    mvprintw(8, 0, "Auto: %s        ", ts->auto_send ? "ON" : "OFF");
                    refresh();
                    continue;
                case 'v': case 'V':
                    ts->stream_st7735 = !ts->stream_st7735;
                    if (ts->stream_st7735 && ts->vsock < 0) (void)ensure_video_socket(ts, st->ip);
                    mvprintw(9, 0, "Video: %s        ", ts->stream_st7735 ? "ON" : "OFF");
                    refresh();
                    continue;
                default:
                    if (ch >= 0 && ch <= 255) cmd = map_key(km, (unsigned char)ch, NULL, 0);
                    break;
            }
            if (cmd) {
                st->last_key = arrow ? 0 : (char)((ch >= 0 && ch <= 255) ? ch : 0);
                st->last_cmd = cmd;
                ui_draw(st);
                if (send(st->sock, &cmd, 1, 0) != 1) { close_video(ts); return -1; }
                if (cmd == 'Q') { close_video(ts); return 1; }
            }
        }
#else
        if (1) {
            unsigned char ch;
            while (kbd_read_raw(&ch)) {
                static unsigned char esc_buf2[8]; static size_t esc_len2 = 0; static bool in_esc2 = false;
                if (!in_esc2) {
                    if (ch == '\x1b') { in_esc2 = true; esc_len2 = 0; continue; }
                    // Runtime toggles for NO_CURSES mode
                    if (ch == 't' || ch == 'T') { ts->auto_send = !ts->auto_send; printf("\n[Auto %s]\n", ts->auto_send?"ON":"OFF"); continue; }
                    if (ch == 'v' || ch == 'V') { ts->stream_st7735 = !ts->stream_st7735; if (ts->stream_st7735 && ts->vsock < 0) (void)ensure_video_socket(ts, st->ip); printf("\n[Video %s]\n", ts->stream_st7735?"ON":"OFF"); continue; }
                    char cmd = map_key(km, ch, NULL, 0);
                    if (cmd) {
                        st->last_key = (char)ch;
                        st->last_cmd = cmd;
                        if (send(st->sock, &cmd, 1, 0) != 1) { close_video(ts); return -1; }
                        if (cmd == 'Q') { close_video(ts); return 1; }
                    }
                } else {
                    if (esc_len2 < sizeof(esc_buf2)) esc_buf2[esc_len2++] = ch;
                    if (esc_len2 >= 2) {
                        char cmd = map_key(km, '\x1b', esc_buf2, esc_len2);
                        if (cmd) {
                            st->last_key = 0; st->last_cmd = cmd;
                            if (send(st->sock, &cmd, 1, 0) != 1) { close_video(ts); return -1; }
                        }
                        in_esc2 = false; esc_len2 = 0;
                    }
                }
            }
        }
#endif
    }
    close_video(ts);
    return 1;
}

// Tracking-only session: no socket, just video + HUD + keyboard quit
static int track_only_session(TrackState *ts) {
#ifndef NO_CURSES
    ui_init();
    atexit(ui_shutdown);
    erase();
    mvprintw(0, 0, "WIPERITE Tracking-Only | Video: %s | Size: %dx%d", ts->dev_path, ts->width, ts->height);
    mvhline(1, 0, '-', 80);
    mvprintw(2, 0, "Press Q to quit. HUD shows centroid and proposed cmd.");
    refresh();
#else
    printf("Tracking-only mode. Press 'q' to quit.\n");
#endif
    if (open_video_if_needed(ts) != 0) return -1;
    while (!g_stop) {
        fd_set rfds; FD_ZERO(&rfds);
        FD_SET(ts->vfd, &rfds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
        int rc = select(ts->vfd + 1, &rfds, NULL, NULL, &tv);
        if (rc < 0) { if (errno == EINTR) continue; break; }
        if (FD_ISSET(ts->vfd, &rfds)) {
            ssize_t n = read_full(ts->vfd, ts->framebuf, (size_t)ts->frame_bytes);
            if (n != ts->frame_bytes) { fprintf(stderr, "Video read error or EOF\n"); break; }
            process_frame_dark_on_white(ts);
            char auto_cmd = guidance_from_centroid(ts, ts->last_cx, ts->last_cy);
#ifndef NO_CURSES
            mvprintw(4, 0, "Tracking: ON  CX:%d CY:%d Count:%d  Proposed Cmd:%c        ", ts->last_cx, ts->last_cy, ts->last_count, auto_cmd ? auto_cmd : '-');
            refresh();
            int ch = ui_get_key();
            if (ch == 'q' || ch == 'Q') { g_stop = 1; break; }
#else
            static int tick = 0; if ((tick++ % 10) == 0) {
                printf("[Track] CX=%d CY=%d Count=%d Cmd=%c\n", ts->last_cx, ts->last_cy, ts->last_count, auto_cmd ? auto_cmd : '-');
            }
            unsigned char ch; while (kbd_read_raw(&ch)) { if (ch=='q'||ch=='Q') { g_stop=1; break; } }
#endif
        } else {
#ifndef NO_CURSES
            int ch = ui_get_key(); if (ch == 'q' || ch == 'Q') { g_stop = 1; break; }
#else
            unsigned char ch; while (kbd_read_raw(&ch)) { if (ch=='q'||ch=='Q') { g_stop=1; break; } }
#endif
        }
    }
    close_video(ts);
    return 1;
}

int main(int argc, char **argv) {
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    const char *ip = NULL;
    uint16_t port = DEFAULT_PORT;
    const char *cmdstr = NULL;
    TrackState ts; trackstate_init_defaults(&ts);

    bool ip_provided = false;
    if (argc >= 2 && argv[1][0] != '-') { ip = argv[1]; ip_provided = true; }
    if (!ip_provided && argc < 2) { usage(argv[0]); return 1; }
    if (ip_provided && argc >= 3 && argv[2][0] != '-') {
        long p = strtol(argv[2], NULL, 10);
        if (p <= 0 || p > 65535) { fprintf(stderr, "Bad port\n"); return 1; }
        port = (uint16_t)p;
    }

    // Parse optional -c and tracking/video flags; start index depends on whether IP/port present
    int arg_start = ip_provided ? ((argc >= 3 && argv[2][0] != '-') ? 3 : 2) : 1;
    for (int i = arg_start; i < argc; ++i) {
        if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            cmdstr = argv[i + 1];
            ++i;
        } else if (strcmp(argv[i], "--map") == 0 && i + 1 < argc) {
            // Defer applying in the appropriate mode below
            ++i; // skip value here; we'll re-parse below for each mode
        } else if (strcmp(argv[i], "--track") == 0) {
            ts.enabled = true;
        } else if (strcmp(argv[i], "--dev") == 0 && i + 1 < argc) {
            snprintf(ts.dev_path, sizeof(ts.dev_path), "%s", argv[++i]);
        } else if (strcmp(argv[i], "--size") == 0 && i + 1 < argc) {
            int w = 0, h = 0; if (sscanf(argv[++i], "%dx%d", &w, &h) == 2 && w > 0 && h > 0) { ts.width = w; ts.height = h; ts.frame_bytes = w*h*2; }
        } else if (strcmp(argv[i], "--ythr") == 0 && i + 1 < argc) {
            ts.y_threshold = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--step") == 0 && i + 1 < argc) {
            ts.sample_step = atoi(argv[++i]); if (ts.sample_step < 1) ts.sample_step = 1;
        } else if (strcmp(argv[i], "--min") == 0 && i + 1 < argc) {
            ts.min_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--margin") == 0 && i + 1 < argc) {
            ts.margin = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--auto") == 0) {
            ts.auto_send = true;
        } else if (strcmp(argv[i], "--video") == 0) {
            ts.stream_st7735 = true;
        } else if (strcmp(argv[i], "--vport") == 0 && i + 1 < argc) {
            int vp = atoi(argv[++i]); if (vp > 0 && vp <= 65535) ts.video_port = (uint16_t)vp;
        } else if (strcmp(argv[i], "--vsize") == 0 && i + 1 < argc) {
            int vw = 0, vh = 0; if (sscanf(argv[++i], "%dx%d", &vw, &vh) == 2 && vw > 0 && vh > 0) { ts.out_w = vw; ts.out_h = vh; }
        } else if (strcmp(argv[i], "--vfps") == 0 && i + 1 < argc) {
            int fps = atoi(argv[++i]); if (fps > 0 && fps <= 60) ts.vsend_interval_ms = 1000 / fps;
        } else if (strcmp(argv[i], "--rotate") == 0) {
            ts.rotate90 = true;
        }
    }

    // If no IP and tracking enabled, jump directly into tracking-only mode before any socket logic
    if (!ip_provided) {
        if (ts.enabled) {
            return track_only_session(&ts);
        } else if (ts.stream_st7735) {
            fprintf(stderr, "--video requires an IP address (separate TCP port).\n");
            usage(argv[0]);
            return 1;
        } else if (cmdstr) {
            fprintf(stderr, "One-shot (-c) requires an IP address.\n");
            usage(argv[0]);
            return 1;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    // One-shot mode: simple connect/send/close; minimal reconnect (one retry)
    if (cmdstr) {
        KeyMap km; keymap_init_defaults(&km);
        // Apply any --map arguments
        for (int i = 2; i < argc; ++i) {
            if (strcmp(argv[i], "--map") == 0 && i + 1 < argc) {
                (void)keymap_set(&km, argv[i + 1]);
                ++i;
            }
        }
        int attempts = 0;
        size_t idx = 0, len = strlen(cmdstr);
        int sock = -1;
        while (idx < len && attempts < 2) {
            if (sock < 0) {
                sock = connect_with_timeout(ip, port, 5);
            }
            while (idx < len) {
                char cmd = map_key(&km, (unsigned char)cmdstr[idx], NULL, 0);
                if (!cmd) cmd = cmdstr[idx];
                ssize_t n = send(sock, &cmd, 1, 0);
                if (n != 1) { close(sock); sock = -1; attempts++; break; }
                idx++;
            }
        }
        if (sock >= 0) {
            char buf[256]; ssize_t n = recv(sock, buf, sizeof(buf)-1, 0);
            if (n > 0) { buf[n] = '\0'; printf("%s", buf); }
            close(sock);
        }
        return (idx == len) ? 0 : 1;
    }

    // Interactive mode with optional tracking + auto-reconnect
    AppState st = { .ip = ip, .port = port, .sock = -1, .reconnects = 0, .last_cmd = 0, .last_key = 0, .connected = false };
    KeyMap km; keymap_init_defaults(&km);
    // Apply --map arguments for interactive mode
    int map_start = ip_provided ? ((argc >= 3 && argv[2][0] != '-') ? 3 : 2) : 1;
    for (int i = map_start; i < argc; ++i) {
        if (strcmp(argv[i], "--map") == 0 && i + 1 < argc) {
            (void)keymap_set(&km, argv[i + 1]);
            ++i;
        }
    }
#ifndef NO_CURSES
    ui_init();
    atexit(ui_shutdown);
#endif

    int backoff = 1;
    while (!g_stop) {
        // Connect
        if (st.sock >= 0) { close(st.sock); st.sock = -1; }
        st.connected = false;
        // Attempt (and re-attempt) connection with backoff until success or stop
        int s = -1; int local_backoff = 1;
        for (;;) {
#ifndef NO_CURSES
            snprintf(st.last_server_msg, sizeof(st.last_server_msg), "Connecting to %s:%u...", st.ip, st.port);
            ui_draw(&st);
#else
            fprintf(stderr, "Connecting to %s:%u...\n", st.ip, st.port);
#endif
            s = connect_with_timeout(ip, port, 5);
            if (s >= 0 || g_stop) break;
#ifndef NO_CURSES
            snprintf(st.last_server_msg, sizeof(st.last_server_msg), "Connect failed. Retrying in %ds...", local_backoff);
            ui_draw(&st);
#else
            fprintf(stderr, "Connect failed. Retrying in %ds...\n", local_backoff);
#endif
            for (int i = 0; i < local_backoff * 10 && !g_stop; ++i) {
                struct timespec ts = { .tv_sec = 0, .tv_nsec = 100000000L };
                nanosleep(&ts, NULL);
            }
            if (local_backoff < 5) local_backoff *= 2;
        }
        if (g_stop) break;
        st.sock = s;
        st.connected = true;
        st.reconnects++;
#ifndef NO_CURSES
        snprintf(st.last_server_msg, sizeof(st.last_server_msg), "Connected.");
        ui_draw(&st);
#else
        printf("Connected.\n");
#endif

        int res;
        if (ts.enabled || ts.stream_st7735) {
            res = interactive_session_trk(&st, &km, &ts);
        } else {
            res = interactive_session(&st, &km);
        }
        if (res == 1) { // user quit
            if (st.sock >= 0) {
                // Try to send polite quit
                char q = 'Q'; send(st.sock, &q, 1, 0);
                close(st.sock); st.sock = -1;
            }
            break;
        }
        // dropped
        if (st.sock >= 0) { close(st.sock); st.sock = -1; }
        st.connected = false;
#ifndef NO_CURSES
        snprintf(st.last_server_msg, sizeof(st.last_server_msg), "Disconnected. Reconnecting in %ds...", backoff);
        ui_draw(&st);
#else
        fprintf(stderr, "Disconnected. Reconnecting in %ds...\n", backoff);
#endif
        for (int i = 0; i < backoff * 10 && !g_stop; ++i) { // sleep with 100ms steps
#ifndef NO_CURSES
            // Keep UI responsive during wait
            int ch = ui_get_key();
            if (ch == 'q' || ch == 'Q') { g_stop = 1; break; }
#endif
            struct timespec ts = { .tv_sec = 0, .tv_nsec = 100000000L };
            nanosleep(&ts, NULL);
        }
        if (backoff < 5) backoff *= 2; // exponential backoff up to 5s
    }

#ifdef NO_CURSES
    disable_raw_mode();
#endif
    return 0;
}
