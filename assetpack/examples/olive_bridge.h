// olive_bridge.h - C++-callable facade over olive.c (single-header
// software renderer). olive.c only compiles as C (C99 designated
// initializers in the font tables), so the bridge re-exports its
// static-inline functions as regular link symbols.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// Same layout as Olivec_Canvas in olive.c: { pixels, width, height, stride }.
typedef struct {
    uint32_t* pixels;
    size_t width;
    size_t height;
    size_t stride;
} Obv_Canvas;

typedef struct {
    size_t width;      // glyph width in pixels
    size_t height;     // glyph height in pixels
    const char* glyphs; // 6x6 bitmaps, one byte per pixel, 128 glyphs
} Obv_Font;

Obv_Canvas obv_canvas(uint32_t* pixels, size_t width, size_t height, size_t stride);
void obv_fill(Obv_Canvas c, uint32_t color);   // color: 0xAARRGGBB
void obv_rect(Obv_Canvas c, int x, int y, int w, int h, uint32_t color);
void obv_text(Obv_Canvas c, const char* text, int x, int y, Obv_Font font,
              size_t size, uint32_t color);

// Triangle rasterization helpers (used by the viewer's own z-buffered
// rasterizer; olivec_barycentric returns true when (xp,yp) is inside the
// triangle, with barycentrics u1/det, u2/det, 1 - (u1+u2)/det).
bool obv_normalize_triangle(size_t width, size_t height, int x1, int y1,
                            int x2, int y2, int x3, int y3, int* lx, int* hx,
                            int* ly, int* hy);
bool obv_barycentric(int x1, int y1, int x2, int y2, int x3, int y3, int xp,
                     int yp, int* u1, int* u2, int* det);
Obv_Font obv_default_font(void);

#ifdef __cplusplus
}
#endif
