#ifndef LOG_H
#define LOG_H

#include <stdint.h>
#include <stdarg.h>

#define MAX_MSG_SIZE 255

void write(const char *message, uint32_t length);

void write_int(uint32_t value);

int swd_printf(const char *string, ...);

#endif /* LOG_H */
