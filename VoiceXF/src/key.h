#ifndef KEY_H
#define KEY_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F "

int get_char();

#endif