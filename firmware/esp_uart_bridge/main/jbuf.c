#include "jbuf.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void jb_init(jbuf_t *jb, size_t initial_cap)
{
    *jb = (jbuf_t){ 0 };
    jb->buf = malloc(initial_cap);
    jb->cap = initial_cap;
    jb->failed = jb->buf == NULL;
}

static bool reserve(jbuf_t *jb, size_t extra)
{
    if (jb->failed) {
        return false;
    }
    if (jb->len + extra + 1 <= jb->cap) {
        return true;
    }
    size_t cap = jb->cap ? jb->cap : 64;
    while (jb->len + extra + 1 > cap) {
        cap *= 2;
    }
    char *p = realloc(jb->buf, cap);
    if (p == NULL) {
        jb->failed = true;
        return false;
    }
    jb->buf = p;
    jb->cap = cap;
    return true;
}

static void put(jbuf_t *jb, const char *s, size_t n)
{
    if (reserve(jb, n)) {
        memcpy(jb->buf + jb->len, s, n);
        jb->len += n;
    }
}

static void putc_(jbuf_t *jb, char c)
{
    put(jb, &c, 1);
}

/* Length of the valid UTF-8 sequence starting at s[0], or 0 if invalid
 * (overlong forms, surrogates and > U+10FFFF are rejected). */
static size_t utf8_len(const uint8_t *s, size_t n)
{
    const uint8_t c = s[0];
    size_t len;
    uint8_t lo = 0x80;
    uint8_t hi = 0xBF;
    if (c >= 0xC2 && c <= 0xDF) {
        len = 2;
    } else if (c >= 0xE0 && c <= 0xEF) {
        len = 3;
        if (c == 0xE0) lo = 0xA0;
        if (c == 0xED) hi = 0x9F;
    } else if (c >= 0xF0 && c <= 0xF4) {
        len = 4;
        if (c == 0xF0) lo = 0x90;
        if (c == 0xF4) hi = 0x8F;
    } else {
        return 0;
    }
    if (n < len || s[1] < lo || s[1] > hi) {
        return 0;
    }
    for (size_t i = 2; i < len; i++) {
        if ((s[i] & 0xC0) != 0x80) {
            return 0;
        }
    }
    return len;
}

/* Valid UTF-8 passes through (Vietnamese SSIDs, UTF-8 robot output); control
 * characters and stray bytes become \u00XX, i.e. the byte's Latin-1 code. */
static void put_escaped(jbuf_t *jb, const uint8_t *s, size_t n)
{
    putc_(jb, '"');
    for (size_t i = 0; i < n; i++) {
        const uint8_t c = s[i];
        if (c >= 0x80) {
            size_t u = utf8_len(s + i, n - i);
            if (u > 0) {
                put(jb, (const char *)s + i, u);
                i += u - 1;
                continue;
            }
        }
        switch (c) {
        case '"':  put(jb, "\\\"", 2); break;
        case '\\': put(jb, "\\\\", 2); break;
        case '\n': put(jb, "\\n", 2); break;
        case '\r': put(jb, "\\r", 2); break;
        case '\t': put(jb, "\\t", 2); break;
        default:
            if (c >= 0x20 && c < 0x7F) {
                putc_(jb, (char)c);
            } else {
                char esc[7];
                snprintf(esc, sizeof(esc), "\\u%04x", c);
                put(jb, esc, 6);
            }
        }
    }
    putc_(jb, '"');
}

static void key(jbuf_t *jb, const char *k)
{
    if (jb->comma) {
        putc_(jb, ',');
    }
    if (k != NULL) {
        put_escaped(jb, (const uint8_t *)k, strlen(k));
        putc_(jb, ':');
    }
    jb->comma = true;
}

void jb_obj_open(jbuf_t *jb, const char *k)
{
    key(jb, k);
    putc_(jb, '{');
    jb->comma = false;
}

void jb_obj_close(jbuf_t *jb)
{
    putc_(jb, '}');
    jb->comma = true;
}

void jb_arr_open(jbuf_t *jb, const char *k)
{
    key(jb, k);
    putc_(jb, '[');
    jb->comma = false;
}

void jb_arr_close(jbuf_t *jb)
{
    putc_(jb, ']');
    jb->comma = true;
}

void jb_str(jbuf_t *jb, const char *k, const char *value)
{
    key(jb, k);
    put_escaped(jb, (const uint8_t *)value, strlen(value));
}

void jb_bytes(jbuf_t *jb, const char *k, const uint8_t *data, size_t len)
{
    key(jb, k);
    put_escaped(jb, data, len);
}

void jb_hex(jbuf_t *jb, const char *k, const uint8_t *data, size_t len)
{
    static const char digits[] = "0123456789abcdef";
    key(jb, k);
    putc_(jb, '"');
    if (reserve(jb, len * 2)) {
        for (size_t i = 0; i < len; i++) {
            jb->buf[jb->len++] = digits[data[i] >> 4];
            jb->buf[jb->len++] = digits[data[i] & 0xF];
        }
    }
    putc_(jb, '"');
}

void jb_int(jbuf_t *jb, const char *k, long long value)
{
    char num[24];
    int n = snprintf(num, sizeof(num), "%lld", value);
    key(jb, k);
    put(jb, num, (size_t)n);
}

void jb_bool(jbuf_t *jb, const char *k, bool value)
{
    key(jb, k);
    if (value) {
        put(jb, "true", 4);
    } else {
        put(jb, "false", 5);
    }
}

char *jb_finish(jbuf_t *jb)
{
    if (jb->failed) {
        jb_free(jb);
        return NULL;
    }
    jb->buf[jb->len] = '\0';
    char *out = jb->buf;
    jb->buf = NULL;
    return out;
}

void jb_free(jbuf_t *jb)
{
    free(jb->buf);
    jb->buf = NULL;
    jb->failed = true;
}
