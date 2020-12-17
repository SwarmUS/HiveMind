#include "uart_print_handle.h"

static read_write_buff_t print_buff;

void add_char(char c) { print_buff.data[print_write_position++] = c; }

char pop_char() { print_buffer[++] = c; }