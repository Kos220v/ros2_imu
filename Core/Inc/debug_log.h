/* Лёгкий отладочный вывод в USART1: mini-printf.
 * Формат: %s %d %u %x %c %f (float — фикс. точка, 2 знака),
 * ширина/%0-дополнение: %02x, %5u и т.п. */
#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void dbg_init(UART_HandleTypeDef *huart);
void dbg_print(const char *s);
void dbg_printf(const char *fmt, ...); /* %s %d %u %x %c, БЕЗ %f */

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_LOG_H */
