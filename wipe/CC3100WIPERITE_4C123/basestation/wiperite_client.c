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
    char last_server_msg[128];
    bool connected;
} AppState;

static void die(const char *msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}

static void disable_raw_mode(void) {
#ifdef NO_CURSES
    if (raw_enabled) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
        raw_enabled = false;
    }
#endif
}

static void enable_raw_mode(void) {
#ifdef NO_CURSES
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
#endif
}

static void on_signal(int sig) {
    (void)sig;
    g_stop = 1;
}

static void usage(const char *prog) {
    fprintf(stderr, "Usage: %s <ip> [port] [-c COMMANDS]\n", prog);
    fputs("  ip       Target TM4C IP (as shown on LCD/UART)\n", stderr);
    fprintf(stderr, "  port     TCP port (default %d)\n", DEFAULT_PORT);
    fputs("  -c str   Send command string then exit (non-interactive)\n\n", stderr);
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
    fputs("UI: ncurses TUI enabled by default (disable with NO_CURSES=1 at build).\n", stderr);
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

    // Optional: small recv timeout
    struct timeval rcv_to = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcv_to, sizeof(rcv_to));

    return sock;
}

static char map_key(unsigned char k, const unsigned char *esc_seq, size_t esc_len) {
    // Arrow keys produce ESC [ A/B/C/D
    if (k == '\x1b' && esc_len >= 2 && esc_seq[0] == '[') {
        unsigned char code = esc_seq[1];
        if (code == 'A') return 'F'; // Up
        if (code == 'B') return 'B'; // Down
        if (code == 'C') return 'R'; // Right
        if (code == 'D') return 'L'; // Left
    }
    // Direct mappings
    switch (k) {
        case 'w': case 'W': return 'F';
        case 's': case 'S': return 'B';
        case 'a': case 'A': return 'L';
        case 'd': case 'D': return 'R';
        case ' ':           return 'S';
        case 'u': case 'U': return 'U';
        case 'j': case 'J': return 'D';
        case '8':           return '8';
        case 'c': case 'C': return 'C';
        case 'z': case 'Z': return 'Z';
        case 'q': case 'Q': return 'Q';
        default:            return 0; // ignore
    }
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
    mvprintw(2, 0, "Controls: WASD/Arrows move, Space=Stop, U/J speed up/down, 8/C/Z custom, Q quit");
    mvprintw(4, 0, "Last command: %c", st->last_cmd ? st->last_cmd : '-');
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
static int kbd_read_raw(unsigned char *out) {
    unsigned char ch; ssize_t n = read(STDIN_FILENO, &ch, 1);
    if (n == 1) { *out = ch; return 1; }
    return 0;
}

// Returns: 0 continue, 1 user-quit, -1 socket dropped
static int interactive_session(AppState *st) {
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
            switch (ch) {
                case KEY_UP: cmd = 'F'; break;
                case KEY_DOWN: cmd = 'B'; break;
                case KEY_LEFT: cmd = 'L'; break;
                case KEY_RIGHT: cmd = 'R'; break;
                case 'w': case 'W': cmd = 'F'; break;
                case 's': case 'S': cmd = 'B'; break;
                case 'a': case 'A': cmd = 'L'; break;
                case 'd': case 'D': cmd = 'R'; break;
                case ' ': cmd = 'S'; break;
                case 'u': case 'U': cmd = 'U'; break;
                case 'j': case 'J': cmd = 'D'; break;
                case '8': cmd = '8'; break;
                case 'c': case 'C': cmd = 'C'; break;
                case 'z': case 'Z': cmd = 'Z'; break;
                case 'q': case 'Q': cmd = 'Q'; break;
                default: break;
            }
            if (cmd) {
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
                    char cmd = map_key(ch, NULL, 0);
                    if (cmd) {
                        st->last_cmd = cmd;
                        if (send(st->sock, &cmd, 1, 0) != 1) return -1;
                        if (cmd == 'Q') return 1;
                    }
                } else {
                    if (esc_len2 < sizeof(esc_buf2)) esc_buf2[esc_len2++] = ch;
                    if (esc_len2 >= 2) {
                        char cmd = map_key('\x1b', esc_buf2, esc_len2);
                        if (cmd) {
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

int main(int argc, char **argv) {
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (argc < 2) { usage(argv[0]); return 1; }

    const char *ip = argv[1];
    uint16_t port = DEFAULT_PORT;
    const char *cmdstr = NULL;

    if (argc >= 3 && argv[2][0] != '-') {
        long p = strtol(argv[2], NULL, 10);
        if (p <= 0 || p > 65535) { fprintf(stderr, "Bad port\n"); return 1; }
        port = (uint16_t)p;
    }

    // Parse optional -c
    for (int i = 2; i < argc; ++i) {
        if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            cmdstr = argv[i + 1];
            ++i;
        }
    }

    // One-shot mode: simple connect/send/close; minimal reconnect (one retry)
    if (cmdstr) {
        int attempts = 0;
        size_t idx = 0, len = strlen(cmdstr);
        int sock = -1;
        while (idx < len && attempts < 2) {
            if (sock < 0) {
                sock = connect_with_timeout(ip, port, 5);
            }
            while (idx < len) {
                char cmd = map_key((unsigned char)cmdstr[idx], NULL, 0);
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

    // Interactive mode with auto-reconnect
    AppState st = { .ip = ip, .port = port, .sock = -1, .reconnects = 0, .last_cmd = 0, .connected = false };
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

        int res = interactive_session(&st);
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
