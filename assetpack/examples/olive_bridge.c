// olive_bridge.c - olive.c compiled as C (it needs C99 designated
// initializers); this TU re-exports the static-inline functions with
// plain external linkage for the C++ viewer.
#include "olive_bridge.h"

#define OLIVEC_IMPLEMENTATION
#include <olive.c>
#include <string.h>

static Olivec_Canvas toC(Obv_Canvas c) {
    Olivec_Canvas o;
    memcpy(&o, &c, sizeof(o));
    return o;
}

Obv_Canvas obv_canvas(uint32_t* pixels, size_t width, size_t height, size_t stride) {
    Obv_Canvas c;
    Olivec_Canvas o = olivec_canvas(pixels, width, height, stride);
    memcpy(&c, &o, sizeof(c));
    return c;
}

void obv_fill(Obv_Canvas c, uint32_t color) { olivec_fill(toC(c), color); }

void obv_rect(Obv_Canvas c, int x, int y, int w, int h, uint32_t color) {
    olivec_rect(toC(c), x, y, w, h, color);
}

void obv_text(Obv_Canvas c, const char* text, int x, int y, Obv_Font font,
              size_t size, uint32_t color) {
    Olivec_Font f;
    f.glyphs = font.glyphs;
    f.width = font.width;
    f.height = font.height;
    olivec_text(toC(c), text, x, y, f, size, color);
}

bool obv_normalize_triangle(size_t width, size_t height, int x1, int y1,
                            int x2, int y2, int x3, int y3, int* lx, int* hx,
                            int* ly, int* hy) {
    return olivec_normalize_triangle(width, height, x1, y1, x2, y2, x3, y3,
                                     lx, hx, ly, hy);
}

bool obv_barycentric(int x1, int y1, int x2, int y2, int x3, int y3, int xp,
                     int yp, int* u1, int* u2, int* det) {
    return olivec_barycentric(x1, y1, x2, y2, x3, y3, xp, yp, u1, u2, det);
}

Obv_Font obv_default_font(void) {
    Obv_Font f;
    f.glyphs = olivec_default_font.glyphs;
    f.width = olivec_default_font.width;
    f.height = olivec_default_font.height;
    return f;
}
