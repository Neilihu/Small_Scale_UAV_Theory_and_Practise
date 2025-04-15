#include <stdio.h>
//#include <stdiolib.h>
#include "BOARD.h"
#include "serial.h"

int main(void) {
    BOARD_Init();
    printf("HELLO WORLD\n");
    BOARD_End();
    while (1);
}