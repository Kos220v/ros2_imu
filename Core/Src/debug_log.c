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

static void emit_int(char *buf, size_t cap, size_t *pos, int32_t v,
                     int width, int zero_pad)
{
    char tmp[12];
    int len = 0;
    uint32_t u;
    if (v < 0) {
        if (*pos < cap) {
            buf[(*pos)++] = '-';
        }
        width -= 1;
        u = (uint32_t)(-(v + 1)) + 1u;
    } else {
        u = (uint32_t)v;
    }
    do {
        tmp[len++] = (char)('0' + (u % 10u));
        u /= 10u;
    } while (u > 0 && len < (int)sizeof(tmp));
    if (zero_pad && width > len) {
        for (int i = len; i < width && (int)*pos < (int)cap - 1; i++) {
            buf[(*pos)++] = '0';
        }
    }
    while (len > 0 && *pos < cap) {
        buf[(*pos)++] = tmp[--len];
    }
}

static void emit_uint(char *buf, size_t cap, size_t *pos, uint32_t v, int hex,
                      int width, int zero_pad)
{
    char tmp[12];
    int len = 0;
    const char *dig = "0123456789abcdef";
    uint32_t base = hex ? 16u : 10u;
    do {
        tmp[len++] = dig[v % base];
        v /= base;
    } while (v > 0 && len < (int)sizeof(tmp));
    if (zero_pad && width > len) {
        for (int i = len; i < width && (int)*pos < (int)cap - 1; i++) {
            buf[(*pos)++] = '0';
        }
    }
    while (len > 0 && *pos < cap) {
        buf[(*pos)++] = tmp[--len];
    }
}

/* Дробное: фиксированная десятичная точка, ровно 2 знака (%f). */
static void emit_float(char *buf, size_t cap, size_t *pos, double d, int width)
{
    char tmp[24];
    int len = 0;
    float f = (float)d;
    if (f < 0.0f) {
        tmp[len++] = '-';
        f = -f;
    }
    uint32_t ip = (uint32_t)f;
    uint32_t fp = (uint32_t)((f - (float)ip) * 100.0f + 0.5f);
    if (fp >= 100u) {
        ip++;
        fp -= 100u;
    }
    char num[12];
    int nlen = 0;
    do {
        num[nlen++] = (char)('0' + (ip % 10u));
        ip /= 10u;
    } while (ip > 0 && nlen < (int)sizeof(num));
    for (int i = nlen - 1; i >= 0; i--) {
        tmp[len++] = num[i];
    }
    tmp[len++] = '.';
    tmp[len++] = (char)('0' + (fp / 10u) % 10u);
    tmp[len++] = (char)('0' + (fp % 10u));
    for (int i = len; i < width && (int)*pos < (int)cap - 1; i++) {
        buf[(*pos)++] = ' ';
    }
    while (len > 0 && *pos < cap) {
        buf[(*pos)++] = tmp[--len];
    }
}

/* Формат: %s %d %u %x %X %c %f; ширина и нули: %02x, %-подобного нет
 * (width без знака = пробелы, с '0' = нули). */
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
        if (*p == '\0') {
            break;
        }
        int width = 0;
        int zero_pad = 0;
        if (*p == '0') {
            zero_pad = 1;
            p++;
        }
        while (*p >= '0' && *p <= '9' && width < 10) {
            width = width * 10 + (*p - '0');
            p++;
        }
        switch (*p) {
        case 'd':
        case 'i':
            emit_int(buf, sizeof(buf) - 1, &pos, va_arg(ap, int), width,
                     zero_pad);
            break;
        case 'u':
            emit_uint(buf, sizeof(buf) - 1, &pos, va_arg(ap, unsigned int), 0,
                      width, zero_pad);
            break;
        case 'x':
        case 'X':
            emit_uint(buf, sizeof(buf) - 1, &pos, va_arg(ap, unsigned int), 1,
                      width, zero_pad);
            break;
        case 'f':
            emit_float(buf, sizeof(buf) - 1, &pos, va_arg(ap, double), width);
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
            buf[pos++] = '%';
            if (*p) {
                buf[pos++] = *p;
            }
            break;
        }
    }
    va_end(ap);
    buf[pos] = '\0';
    dbg_print(buf);
}
