#pragma once

/* Minimal growable JSON writer. Commas are inserted automatically; a NULL key
 * means "array element". On allocation failure the buffer is marked failed
 * and later calls are ignored; jb_finish() then returns NULL. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    char *buf;
    size_t len;
    size_t cap;
    bool comma;
    bool failed;
} jbuf_t;

void jb_init(jbuf_t *jb, size_t initial_cap);
/* Grows the buffer once so `extra` more bytes fit without repeated doubling. */
void jb_reserve(jbuf_t *jb, size_t extra);
void jb_obj_open(jbuf_t *jb, const char *key);
void jb_obj_close(jbuf_t *jb);
void jb_arr_open(jbuf_t *jb, const char *key);
void jb_arr_close(jbuf_t *jb);
void jb_str(jbuf_t *jb, const char *key, const char *value);
/* Arbitrary bytes as a JSON string: printable ASCII as-is, the rest \u00XX. */
void jb_bytes(jbuf_t *jb, const char *key, const uint8_t *data, size_t len);
void jb_hex(jbuf_t *jb, const char *key, const uint8_t *data, size_t len);
void jb_int(jbuf_t *jb, const char *key, long long value);
void jb_bool(jbuf_t *jb, const char *key, bool value);
/* Returns the NUL-terminated string (caller frees) or NULL on failure. */
char *jb_finish(jbuf_t *jb);
void jb_free(jbuf_t *jb);
