#include "debug_log.h"

#include <stdarg.h>
#include <string.h>

#include "imu_config.h"

static UART_HandleTypeDef *dbg_uart;

void dbg_init(UART_HandleTypeDef *huart)
{
    dbg_uart = huart;
}

void dbg_print(const char *s)
{
    if (!dbg_uart || !s) {
        return;
    }
    size_t n = strlen(s);
    if (n > 0) {
        (void)HAL_UART_Transmit(dbg_uart, (const uint8_t *)s, (uint16_t)n,
                                IMU_UART_TX_TIMEOUT_MS);
    }
}

/* Мини-printf без float (чтобы не тянуть тяжёлую поддержку %f). */
static void emit_int(char *buf, size_t cap, size_t *pos, int32_t v)
{
    char tmp[12];
    int len = 0;
    uint32_t u;
    if (v < 0) {
        if (*pos < cap) {
            buf[(*pos)++] = '-';
        }
        u = (uint32_t)(-(v + 1)) + 1u;
    } else {
        u = (uint32_t)v;
    }
    do {
        tmp[len++] = (char)('0' + (u % 10u));
        u /= 10u;
    } while (u > 0 && len < (int)sizeof(tmp));
    while (len > 0 && *pos < cap) {
        buf[(*pos)++] = tmp[--len];
    }
}

static void emit_uint(char *buf, size_t cap, size_t *pos, uint32_t v, int hex)
{
    char tmp[12];
    int len = 0;
    const char *dig = "0123456789abcdef";
    uint32_t base = hex ? 16u : 10u;
    do {
        tmp[len++] = dig[v % base];
        v /= base;
    } while (v > 0 && len < (int)sizeof(tmp));
    while (len > 0 && *pos < cap) {
        buf[(*pos)++] = tmp[--len];
    }
}

void dbg_printf(const char *fmt, ...)
{
    if (!dbg_uart || !fmt) {
        return;
    }
    char buf[160];
    size_t pos = 0;
    va_list ap;
    va_start(ap, fmt);
    for (const char *p = fmt; *p && pos < sizeof(buf) - 1; p++) {
        if (*p != '%') {
            buf[pos++] = *p;
            continue;
        }
        p++;
        switch (*p) {
        case 'd':
        case 'i':
            emit_int(buf, sizeof(buf) - 1, &pos, va_arg(ap, int));
            break;
        case 'u':
            emit_uint(buf, sizeof(buf) - 1, &pos, va_arg(ap, unsigned int), 0);
            break;
        case 'x':
            emit_uint(buf, sizeof(buf) - 1, &pos, va_arg(ap, unsigned int), 1);
            break;
        case 'c': {
            int c = va_arg(ap, int);
            buf[pos++] = (char)c;
            break;
        }
        case 's': {
            const char *s = va_arg(ap, const char *);
            if (!s) {
                s = "(null)";
            }
            while (*s && pos < sizeof(buf) - 1) {
                buf[pos++] = *s++;
            }
            break;
        }
        case '%':
            buf[pos++] = '%';
            break;
        default:
            break;
        }
    }
    va_end(ap);
    buf[pos] = '\0';
    dbg_print(buf);
}
