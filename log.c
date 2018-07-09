#include "log.h"
#include <stdbool.h>
#include <string.h>

enum BASE_E {
  BASE_2,
  BASE_10,
  BASE_16
};

static int uint2str(uint32_t value, char *buf, enum BASE_E base) {
  uint32_t divider;
  switch (base) {
  case BASE_2:
    divider = 2u;
    break;
  case BASE_10:
    divider =10u;
    break;
  case BASE_16:
    divider = 16u;
    break;
  default:
    divider = 10u;
    break;
  }

  int charNb = 0;
  char digit;

  while (value != 0) {
    digit = value % divider;
    if (digit < 10)
      buf[charNb] = digit + '0';
    else if (digit < 16)
      buf[charNb] = digit - 10 + 'A';
    charNb++;
    value = value / divider;
  }

  if (charNb == 0) {
    buf[0] = '0';
    charNb++;
  } else {
    for (int i = 0; i < (charNb / 2); i++) {
      digit = buf[charNb - i - 1];
      buf[charNb - i - 1] = buf[i];
      buf[i] = digit;
    }
  }

  return charNb;
}

/*******************************/
/*        Public API           */
/*******************************/
void write(const char *message, uint32_t length) {
  uint32_t m[] = {2, (uint32_t)message, length};
   asm("mov r0, #0x05;"
       "mov r1, %[msg];"
       "bkpt #0xAB"
         :
         : [msg] "r" (m)
         : "r0", "r1", "memory");
}

int swd_printf(const char *string, ...) {
  int readIndex = 0;
  int writeIndex = 0;
  va_list arg;

  char msg[MAX_MSG_SIZE + 10];
  if (strlen(string) > MAX_MSG_SIZE) {
    return -1;
  }

  va_start(arg, string);

  while ((string[readIndex] != '\0') && (writeIndex < MAX_MSG_SIZE)) {
    if (string[readIndex] == '%') {
      switch(string[readIndex + 1]) {
      case 'd':
        writeIndex += uint2str((uint32_t)va_arg(arg, int), &msg[writeIndex], BASE_10);
        readIndex += 2;
        break;
      case 'x':
        writeIndex += uint2str((uint32_t)va_arg(arg, int), &msg[writeIndex], BASE_16);
        readIndex += 2;
        break;
      case 'b':
        writeIndex += uint2str((uint32_t)va_arg(arg, int), &msg[writeIndex], BASE_2);
        readIndex += 2;
        break;
      default:
        msg[writeIndex++] = string[readIndex++];
        msg[writeIndex++] = string[readIndex++];
      }
    } else {
      msg[writeIndex++] = string[readIndex++];
    }
  }

  write(msg, writeIndex);

  return 0;
}
