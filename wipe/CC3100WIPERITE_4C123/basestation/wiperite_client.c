// WIPERITE base-station client for Zynq-7000 (PetaLinux)
// Connects to TM4C/CC3100 TCP server on port 5000 and sends single-letter commands.
// Interactive mode supports WASD/arrow keys and specific letters; 'q' quits.
// One-shot mode supports sending a string via -c "...".

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

static struct termios orig_termios;
static bool raw_enabled = false;

static void die(const char *msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}

static void disable_raw_mode(void) {
    if (raw_enabled) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
        raw_enabled = false;
    }
}

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

static void on_signal(int sig) {
    (void)sig;
    disable_raw_mode();
    // Don't terminate aggressively; let main handle
    exit(0);
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s <ip> [port] [-c COMMANDS]\n"
            "  ip       Target TM4C IP (as shown on LCD/UART)\n"
            "  port     TCP port (default %d)\n"
            "  -c str   Send command string then exit (non-interactive)\n"
            "\nInteractive controls (no -c):\n"
            "  w or Up Arrow     -> F (Forward)\n"
            "  s or Down Arrow   -> B (Back)\n"
            "  a or Left Arrow   -> L (Left)\n"
            "  d or Right Arrow  -> R (Right)\n"
            "  space             -> S (Stop)\n"
            "  u                 -> U (Speed Up / lift)\n"
            "  j                 -> D (Slow Down / lower)\n"
            "  8                 -> 8 (custom)\n"
            "  c                 -> C (custom)\n"
            "  z                 -> Z (custom)\n"
            "  q                 -> Q (quit)\n",
            prog, DEFAULT_PORT);
}

static int connect_with_timeout(const char *ip, uint16_t port, int timeout_sec) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) die("socket");

    // Set non-blocking for connect timeout
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0) die("fcntl get");
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) die("fcntl set");

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
        fprintf(stderr, "Invalid IP address: %s\n", ip);
        exit(EXIT_FAILURE);
    }

    int rc = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    if (rc < 0 && errno != EINPROGRESS) die("connect");

    if (rc != 0) {
        fd_set wfds; FD_ZERO(&wfds); FD_SET(sock, &wfds);
        struct timeval tv = { .tv_sec = timeout_sec, .tv_usec = 0 };
        rc = select(sock + 1, NULL, &wfds, NULL, &tv);
        if (rc <= 0) {
            fprintf(stderr, "Connect timeout or error\n");
            close(sock);
            exit(EXIT_FAILURE);
        }
        int so_error = 0; socklen_t len = sizeof(so_error);
        if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0 || so_error != 0) {
            errno = so_error;
            die("connect (post-select)");
        }
    }

    // Restore blocking mode
    if (fcntl(sock, F_SETFL, flags) < 0) die("fcntl restore");

    // Optional: small recv timeout
    struct timeval rcv_to = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcv_to, sizeof(rcv_to));

    return sock;
}

static void send_char(int sock, char c) {
    ssize_t n = send(sock, &c, 1, 0);
    if (n != 1) die("send");
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

static void interactive_loop(int sock) {
    printf("Connected. Interactive control ready. Press 'q' to quit.\n");

    // Try to read greeting without blocking the loop
    char buf[256];
    ssize_t gr = recv(sock, buf, sizeof(buf)-1, MSG_DONTWAIT);
    if (gr > 0) {
        buf[gr] = '\0';
        printf("Server: %s", buf);
    }

    enable_raw_mode();
    atexit(disable_raw_mode);

    unsigned char esc_buf[8] = {0};
    size_t esc_len = 0;
    bool in_escape = false;

    for (;;) {
        fd_set rfds; FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        FD_SET(sock, &rfds);
        int maxfd = (STDIN_FILENO > sock ? STDIN_FILENO : sock) + 1;
        struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 }; // 200ms
        int rc = select(maxfd, &rfds, NULL, NULL, &tv);
        if (rc < 0) {
            if (errno == EINTR) continue;
            die("select");
        }

        if (FD_ISSET(sock, &rfds)) {
            char rbuf[256];
            ssize_t n = recv(sock, rbuf, sizeof(rbuf)-1, 0);
            if (n <= 0) {
                printf("\nServer closed connection.\n");
                break;
            }
            rbuf[n] = '\0';
            printf("%s", rbuf);
            fflush(stdout);
        }

        if (FD_ISSET(STDIN_FILENO, &rfds)) {
            unsigned char ch;
            ssize_t n = read(STDIN_FILENO, &ch, 1);
            if (n == 1) {
                if (!in_escape) {
                    if (ch == '\x1b') { // ESC
                        in_escape = true;
                        esc_len = 0;
                        continue;
                    }
                    char cmd = map_key(ch, NULL, 0);
                    if (cmd) {
                        send_char(sock, cmd);
                        if (cmd == 'Q') break;
                    }
                } else {
                    // reading escape sequence
                    if (esc_len < sizeof(esc_buf)) esc_buf[esc_len++] = ch;
                    if (esc_len >= 2) { // we have at least "[" + code
                        char cmd = map_key('\x1b', esc_buf, esc_len);
                        if (cmd) send_char(sock, cmd);
                        in_escape = false;
                        esc_len = 0;
                    }
                }
            }
        }
    }
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

    int sock = connect_with_timeout(ip, port, 5);

    if (cmdstr) {
        size_t len = strlen(cmdstr);
        for (size_t i = 0; i < len; ++i) {
            char cmd = map_key((unsigned char)cmdstr[i], NULL, 0);
            if (!cmd) cmd = cmdstr[i]; // allow raw letters like 'F'
            send_char(sock, cmd);
        }
        // Try to read any immediate response
        char buf[256];
        ssize_t n = recv(sock, buf, sizeof(buf)-1, 0);
        if (n > 0) { buf[n] = '\0'; printf("%s", buf); }
        close(sock);
        return 0;
    }

    interactive_loop(sock);

    // Send polite quit if still open
    send_char(sock, 'Q');
    close(sock);
    disable_raw_mode();
    return 0;
}
