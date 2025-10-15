#include <stdio.h>
#include "app.h"

int main(int argc, char** argv) {
    printf("hello from firmware; argc=%d\n", argc);
    for (int i = 0; i < argc; ++i) {
        printf(" arg[%d] = %s\n", i, argv[i]);
    }
    // place a handy line for a breakpoint:
    puts("ready to debug");  // set breakpoint here
    return 0;
}
