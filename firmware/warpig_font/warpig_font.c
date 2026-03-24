// ============================================================================
// warpig_font — Bézier font rendering for MicroPython (ESP32-S3 / RP2350)
//
// Implements the Slug Algorithm for GPU-quality antialiased text on MCUs.
// Reads .slugfont binary files built by warpig_slug_v3.py on the Pi 5.
//
// Credit: Slug shader code Copyright 2017 by Eric Lengyel.
//         Patent dedicated to public domain. Reference shaders MIT licensed.
//
// This module is the fourth WarpPig native C module, joining:
//   warpig_http  — HTTPS client with SSE streaming
//   warpig_httpd — Native HTTP server on FreeRTOS task
//   warpig_rtos  — FreeRTOS primitives exposed to Python
//
// All rendering uses float (32-bit) — matches the original GPU shader and
// gives single-cycle multiply on both Xtensa LX7 and Cortex-M33 FPv5.
// ============================================================================

#include "py/runtime.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/binary.h"
#include "py/stream.h"
#include "py/builtin.h"

#include <math.h>
#include <string.h>

// ============================================================================
// SLUGFONT FORMAT CONSTANTS (must match warpig_slug_v3.py)
// ============================================================================

#define WARPSLUG_MAGIC       "WARPSLUG"
#define WARPSLUG_MAGIC_LEN   8
#define WARPSLUG_VERSION     3
#define WARPSLUG_HEADER_SIZE 32
#define GLYPH_FIXED_SIZE     20
#define BAND_ENTRY_SIZE      4
#define KERN_ENTRY_SIZE      8
#define CURVE_FLOATS         6   // x1,y1, x2,y2, x3,y3 per curve

// ============================================================================
// SLUGFONT ON-DISK STRUCTURES (packed, little-endian)
// ============================================================================

// We don't use structs for parsing — we read directly from the byte buffer
// with explicit offset calculations. This avoids alignment/packing issues
// across ARM and Xtensa.

// Header layout (32 bytes):
//   [0:8]   magic
//   [8]     version
//   [9]     n_bands
//   [10:12] glyph_count (u16)
//   [12:14] units_per_em (u16)
//   [14:16] ascender (u16)
//   [16:18] descender (u16)
//   [18:20] line_gap (u16)
//   [20:22] space_advance (u16)
//   [22:24] kern_count (u16)
//   [24:28] curve_pool_offset (u32)
//   [28:32] band_pool_offset (u32)

// ============================================================================
// INLINE HELPERS — read little-endian from byte buffer
// ============================================================================

static inline uint16_t rd_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline int16_t rd_i16(const uint8_t *p) {
    return (int16_t)rd_u16(p);
}

static inline uint32_t rd_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline float rd_f32(const uint8_t *p) {
    uint32_t bits = rd_u32(p);
    float f;
    memcpy(&f, &bits, 4);
    return f;
}

// ============================================================================
// SLUG RENDERING CORE — all float32, matches SlugPixelShader.hlsl
// ============================================================================

static inline float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

// Root eligibility from control point signs — Lengyel's 0x2E74 magic
static inline void calc_root_code(float y1, float y2, float y3,
                                   int *use_r1, int *use_r2) {
    int s = (y1 < 0.0f ? 1 : 0) |
            (y2 < 0.0f ? 2 : 0) |
            (y3 < 0.0f ? 4 : 0);
    int code = (0x2E74 >> s) & 0x0101;
    *use_r1 = code & 1;
    *use_r2 = code >> 8;
}

// Solve where curve crosses y=0, return x-coordinates
// Matches SolveHorizPoly in SlugPixelShader.hlsl
static void solve_horiz(float p1x, float p1y, float p2x, float p2y,
                         float p3x, float p3y, float *rx1, float *rx2) {
    float ay = p1y - 2.0f * p2y + p3y;
    float by = p1y - p2y;
    float ax = p1x - 2.0f * p2x + p3x;
    float bx = p1x - p2x;

    float t1, t2;

    if (fabsf(ay) < (1.0f / 65536.0f)) {
        // Nearly linear
        if (fabsf(by) < 1e-12f) {
            *rx1 = *rx2 = 0.0f;
            return;
        }
        t1 = t2 = p1y / (2.0f * by);
    } else {
        float disc = by * by - ay * p1y;
        if (disc < 0.0f) disc = 0.0f;
        float d = sqrtf(disc);
        float ra = 1.0f / ay;
        t1 = (by - d) * ra;
        t2 = (by + d) * ra;
    }

    // Evaluate x at both t values (Horner form)
    *rx1 = (ax * t1 - bx * 2.0f) * t1 + p1x;
    *rx2 = (ax * t2 - bx * 2.0f) * t2 + p1x;
}

// Solve where curve crosses x=0, return y-coordinates
static void solve_vert(float p1x, float p1y, float p2x, float p2y,
                        float p3x, float p3y, float *ry1, float *ry2) {
    float ax = p1x - 2.0f * p2x + p3x;
    float bx = p1x - p2x;
    float ay = p1y - 2.0f * p2y + p3y;
    float by = p1y - p2y;

    float t1, t2;

    if (fabsf(ax) < (1.0f / 65536.0f)) {
        if (fabsf(bx) < 1e-12f) {
            *ry1 = *ry2 = 0.0f;
            return;
        }
        t1 = t2 = p1x / (2.0f * bx);
    } else {
        float disc = bx * bx - ax * p1x;
        if (disc < 0.0f) disc = 0.0f;
        float d = sqrtf(disc);
        float ra = 1.0f / ax;
        t1 = (bx - d) * ra;
        t2 = (bx + d) * ra;
    }

    *ry1 = (ay * t1 - by * 2.0f) * t1 + p1y;
    *ry2 = (ay * t2 - by * 2.0f) * t2 + p1y;
}

// Trace horizontal ray through band curves
static void trace_hband(const uint8_t *curves, int curve_off,
                         int count, int band_idx, const uint8_t *band_pool,
                         float sx, float sy, float ppe,
                         float *out_cov, float *out_wgt) {
    float cov = 0.0f;
    float wgt = 0.0f;

    for (int i = 0; i < count; i++) {
        int ci = rd_u16(band_pool + (band_idx + i) * 2);
        const uint8_t *c = curves + (curve_off + ci) * CURVE_FLOATS * 4;

        float p1x = rd_f32(c)      - sx, p1y = rd_f32(c + 4)  - sy;
        float p2x = rd_f32(c + 8)  - sx, p2y = rd_f32(c + 12) - sy;
        float p3x = rd_f32(c + 16) - sx, p3y = rd_f32(c + 20) - sy;

        // Early exit: curves sorted by descending max-x
        float mx = p1x;
        if (p2x > mx) mx = p2x;
        if (p3x > mx) mx = p3x;
        if (mx * ppe < -0.5f) break;

        int use_r1, use_r2;
        calc_root_code(p1y, p2y, p3y, &use_r1, &use_r2);
        if (!use_r1 && !use_r2) continue;

        float rx1, rx2;
        solve_horiz(p1x, p1y, p2x, p2y, p3x, p3y, &rx1, &rx2);

        if (use_r1) {
            float r = rx1 * ppe;
            cov += clamp01(r + 0.5f);
            float w = clamp01(1.0f - fabsf(r) * 2.0f);
            if (w > wgt) wgt = w;
        }
        if (use_r2) {
            float r = rx2 * ppe;
            cov -= clamp01(r + 0.5f);
            float w = clamp01(1.0f - fabsf(r) * 2.0f);
            if (w > wgt) wgt = w;
        }
    }

    *out_cov = cov;
    *out_wgt = wgt;
}

// Trace vertical ray through band curves (sign-flipped vs horizontal)
static void trace_vband(const uint8_t *curves, int curve_off,
                         int count, int band_idx, const uint8_t *band_pool,
                         float sx, float sy, float ppe,
                         float *out_cov, float *out_wgt) {
    float cov = 0.0f;
    float wgt = 0.0f;

    for (int i = 0; i < count; i++) {
        int ci = rd_u16(band_pool + (band_idx + i) * 2);
        const uint8_t *c = curves + (curve_off + ci) * CURVE_FLOATS * 4;

        float p1x = rd_f32(c)      - sx, p1y = rd_f32(c + 4)  - sy;
        float p2x = rd_f32(c + 8)  - sx, p2y = rd_f32(c + 12) - sy;
        float p3x = rd_f32(c + 16) - sx, p3y = rd_f32(c + 20) - sy;

        float my = p1y;
        if (p2y > my) my = p2y;
        if (p3y > my) my = p3y;
        if (my * ppe < -0.5f) break;

        int use_r1, use_r2;
        calc_root_code(p1x, p2x, p3x, &use_r1, &use_r2);
        if (!use_r1 && !use_r2) continue;

        float ry1, ry2;
        solve_vert(p1x, p1y, p2x, p2y, p3x, p3y, &ry1, &ry2);

        if (use_r1) {
            float r = ry1 * ppe;
            cov -= clamp01(r + 0.5f);
            float w = clamp01(1.0f - fabsf(r) * 2.0f);
            if (w > wgt) wgt = w;
        }
        if (use_r2) {
            float r = ry2 * ppe;
            cov += clamp01(r + 0.5f);
            float w = clamp01(1.0f - fabsf(r) * 2.0f);
            if (w > wgt) wgt = w;
        }
    }

    *out_cov = cov;
    *out_wgt = wgt;
}

// Lengyel's weighted dual-ray coverage combination
// weight_boost: apply sqrt() to boost optical weight of thin features.
// Matches SLUG_WEIGHT define in SlugPixelShader.hlsl.
// Greatly improves legibility at small pixel sizes (8-20px).
static inline float calc_coverage(float xcov, float ycov,
                                    float xwgt, float ywgt,
                                    int weight_boost) {
    float wsum = xwgt + ywgt;
    if (wsum < (1.0f / 65536.0f)) wsum = (1.0f / 65536.0f);

    float weighted = fabsf(xcov * xwgt + ycov * ywgt) / wsum;
    float ax = fabsf(xcov);
    float ay = fabsf(ycov);
    float conservative = (ax < ay) ? ax : ay;
    float result = (weighted > conservative) ? weighted : conservative;
    result = clamp01(result);

    if (weight_boost) {
        result = sqrtf(result);
    }

    return result;
}

// ============================================================================
// MICROPYTHON SLUGFONT TYPE
// ============================================================================

typedef struct _slugfont_obj_t {
    mp_obj_base_t base;

    // Raw file data (kept in memory for zero-copy access)
    uint8_t *data;
    size_t data_len;

    // Parsed header
    uint8_t n_bands;
    uint16_t glyph_count;
    uint16_t units_per_em;
    uint16_t ascender;
    uint16_t descender;
    uint16_t line_gap;
    uint16_t space_advance;
    uint16_t kern_count;

    // Rendering options
    uint8_t weight_boost;              // 1 = apply sqrt() for bolder thin features
    // Pointers into data buffer (byte pointers to avoid alignment issues on ARM)
    const uint8_t *glyph_dir;      // start of glyph directory
    const uint8_t *kern_table;     // start of kern table
    const uint8_t *band_pool;      // band index pool (u16 entries, read via rd_u16)
    const uint8_t *curve_pool;     // curve data pool (f32 entries, read via rd_f32)
    size_t glyph_entry_size;       // bytes per glyph entry
} slugfont_obj_t;

// Forward declaration — MP_DEFINE_CONST_OBJ_TYPE expands to non-static
extern const mp_obj_type_t slugfont_type;

// ---- Glyph lookup by codepoint (linear scan — fine for <128 glyphs) ----

static const uint8_t *slugfont_find_glyph(slugfont_obj_t *self, uint32_t codepoint) {
    const uint8_t *p = self->glyph_dir;
    for (int i = 0; i < self->glyph_count; i++) {
        if (rd_u32(p) == codepoint) return p;
        p += self->glyph_entry_size;
    }
    return NULL;
}

// ---- Kern lookup (linear scan — fine for typical kern table sizes) ----

static int16_t slugfont_get_kern(slugfont_obj_t *self, uint16_t left, uint16_t right) {
    const uint8_t *p = self->kern_table;
    for (int i = 0; i < self->kern_count; i++) {
        if (rd_u16(p) == left && rd_u16(p + 2) == right) {
            return rd_i16(p + 4);
        }
        p += KERN_ENTRY_SIZE;
    }
    return 0;
}

// ============================================================================
// CONSTRUCTOR: warpig_font.SlugFont(path)
// ============================================================================

static mp_obj_t slugfont_make_new(const mp_obj_type_t *type,
                                   size_t n_args, size_t n_kw,
                                   const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    uint8_t *data;
    size_t file_size;

    if (mp_obj_is_str(args[0])) {
        // Path string — open file and call .read() method
        // Avoids low-level stream seek which crashes on some VFS implementations
        mp_obj_t open_args[2] = { args[0], mp_obj_new_str("rb", 2) };
        mp_obj_t fobj = mp_builtin_open(2, open_args, (mp_map_t *)&mp_const_empty_map);

        // Call .read() with no args to get all data
        mp_obj_t read_func = mp_load_attr(fobj, MP_QSTR_read);
        mp_obj_t data_obj = mp_call_function_n_kw(read_func, 0, 0, NULL);

        // Close
        mp_obj_t close_func = mp_load_attr(fobj, MP_QSTR_close);
        mp_call_function_n_kw(close_func, 0, 0, NULL);

        // Copy data from returned bytes object
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
        file_size = bufinfo.len;
        data = m_malloc(file_size);
        memcpy(data, bufinfo.buf, file_size);
    } else {
        // bytes/bytearray — use directly
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(args[0], &bufinfo, MP_BUFFER_READ);
        file_size = bufinfo.len;
        data = m_malloc(file_size);
        memcpy(data, bufinfo.buf, file_size);
    }

    // Validate magic
    if (file_size < WARPSLUG_HEADER_SIZE ||
        memcmp(data, WARPSLUG_MAGIC, WARPSLUG_MAGIC_LEN) != 0) {
        m_free(data);
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("not a WARPSLUG file"));
    }

    if (data[8] != WARPSLUG_VERSION) {
        m_free(data);
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("wrong WARPSLUG version"));
    }

    // Create object
    slugfont_obj_t *self = mp_obj_malloc(slugfont_obj_t, &slugfont_type);
    self->data = data;
    self->data_len = file_size;

    // Parse header
    self->n_bands       = data[9];
    self->glyph_count   = rd_u16(data + 10);
    self->units_per_em  = rd_u16(data + 12);
    self->ascender      = rd_u16(data + 14);
    self->descender     = rd_u16(data + 16);
    self->line_gap      = rd_u16(data + 18);
    self->space_advance = rd_u16(data + 20);
    self->kern_count    = rd_u16(data + 22);

    uint32_t curve_pool_off = rd_u32(data + 24);
    uint32_t band_pool_off  = rd_u32(data + 28);

    self->glyph_entry_size = GLYPH_FIXED_SIZE + self->n_bands * 2 * BAND_ENTRY_SIZE;
    self->glyph_dir = data + WARPSLUG_HEADER_SIZE;
    self->kern_table = self->glyph_dir + self->glyph_count * self->glyph_entry_size;
    self->band_pool = data + band_pool_off;
    self->curve_pool = data + curve_pool_off;

    // Weight boost ON by default — essential for legibility at small sizes
    self->weight_boost = 1;

    return MP_OBJ_FROM_PTR(self);
}

// ============================================================================
// METHOD: font.render_glyph(codepoint, px_size, buf, buf_w, x, y)
// Returns advance width in pixels
// ============================================================================

static mp_obj_t slugfont_render_glyph(size_t n_args, const mp_obj_t *args) {
    slugfont_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint32_t codepoint   = mp_obj_get_int(args[1]);
    float px_size        = mp_obj_get_float(args[2]);

    // Get buffer
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[3], &bufinfo, MP_BUFFER_WRITE);
    uint8_t *buf = bufinfo.buf;
    size_t buf_len = bufinfo.len;

    int buf_w = mp_obj_get_int(args[4]);
    int bx    = mp_obj_get_int(args[5]);
    int by    = mp_obj_get_int(args[6]);

    float scale = px_size / (float)(self->ascender + self->descender);

    // Find glyph
    const uint8_t *g = slugfont_find_glyph(self, codepoint);
    if (!g) {
        // Space or missing — return space advance
        return mp_obj_new_int((int)(self->space_advance * scale + 0.5f));
    }

    uint16_t advance = rd_u16(g + 4);
    int16_t lsb      = rd_i16(g + 6);
    int16_t bx1      = rd_i16(g + 8);
    int16_t by1      = rd_i16(g + 10);
    int16_t bx2      = rd_i16(g + 12);
    int16_t by2      = rd_i16(g + 14);
    uint16_t c_off   = rd_u16(g + 16);

    int adv_px = (int)(advance * scale + 0.5f);
    int gw = bx2 - bx1;
    int gh = by2 - by1;
    if (gw <= 0 || gh <= 0) return mp_obj_new_int(adv_px);

    float lsb_px = lsb * scale;
    int gw_px = (int)(gw * scale + 0.5f);
    int gh_px = (int)(gh * scale + 0.5f);
    if (gw_px < 1 || gh_px < 1) return mp_obj_new_int(adv_px);

    float fu_per_px_x = (float)gw / gw_px;
    float fu_per_px_y = (float)gh / gh_px;
    float ppe_x = 1.0f / fu_per_px_x;
    float ppe_y = 1.0f / fu_per_px_y;

    int nb = self->n_bands;
    float band_h = (float)gh / nb;
    float band_w = (float)gw / nb;

    int baseline_y = (int)(self->ascender * scale + 0.5f);
    int glyph_top = baseline_y - (int)(by2 * scale + 0.5f);
    int glyph_left = (int)(bx + lsb_px + 0.5f);

    // Parse band references for this glyph
    const uint8_t *band_ptr = g + GLYPH_FIXED_SIZE;

    for (int py = 0; py < gh_px; py++) {
        float ey = (gh_px - 1 - py) * fu_per_px_y;
        int row = by + glyph_top + py;
        if (row < 0) continue;

        for (int px = 0; px < gw_px; px++) {
            float ex = px * fu_per_px_x;

            // Sample at two offsets and take max — eliminates band boundary gaps
            // Tiny jitter (1/16 pixel) is enough to dodge exact boundary zeros
            float best = 0.0f;
            for (int s = 0; s < 2; s++) {
                float sx = ex + (s ? 0.0625f * fu_per_px_x : 0.0f);
                float sy = ey + (s ? 0.0625f * fu_per_px_y : 0.0f);

                int hbi2 = (int)(sy / band_h);
                if (hbi2 >= nb) hbi2 = nb - 1;
                if (hbi2 < 0) hbi2 = 0;
                uint16_t hc = rd_u16(band_ptr + hbi2 * BAND_ENTRY_SIZE);
                uint16_t hp = rd_u16(band_ptr + hbi2 * BAND_ENTRY_SIZE + 2);

                int vbi = (int)(sx / band_w);
                if (vbi >= nb) vbi = nb - 1;
                if (vbi < 0) vbi = 0;
                int voff = (nb + vbi) * BAND_ENTRY_SIZE;
                uint16_t vc = rd_u16(band_ptr + voff);
                uint16_t vp = rd_u16(band_ptr + voff + 2);

                float xcov, xwgt, ycov, ywgt;
                trace_hband(self->curve_pool, c_off, hc, hp,
                            self->band_pool, sx, sy, ppe_x, &xcov, &xwgt);
                trace_vband(self->curve_pool, c_off, vc, vp,
                            self->band_pool, sx, sy, ppe_y, &ycov, &ywgt);
                float c = calc_coverage(xcov, ycov, xwgt, ywgt,
                                        self->weight_boost);
                if (c > best) best = c;
            }

            int col = glyph_left + px;
            if (col >= 0 && col < buf_w) {
                size_t idx = (size_t)row * buf_w + col;
                if (idx < buf_len) {
                    int val = (int)(best * 255.0f + 0.5f);
                    if (val > 255) val = 255;
                    if (val > buf[idx]) buf[idx] = (uint8_t)val;
                }
            }
        }
    }

    // Patch isolated zero pixels inside glyph outlines.
    // A hole is a zero pixel with 5+ of 8 neighbors filled AND at least 2
    // strong (>96) cardinal neighbors. This catches interior gaps from the
    // Slug algorithm without filling counters or creating edge halos.
    for (int py = 1; py < gh_px - 1; py++) {
        int row = by + glyph_top + py;
        if (row < 1 || row >= (int)(buf_len / buf_w) - 1) continue;
        for (int px = 1; px < gw_px - 1; px++) {
            int col = glyph_left + px;
            if (col < 1 || col >= buf_w - 1) continue;
            size_t idx = (size_t)row * buf_w + col;
            if (idx >= buf_len || buf[idx] != 0) continue;

            uint8_t up    = buf[idx - buf_w];
            uint8_t down  = buf[idx + buf_w];
            uint8_t left  = buf[idx - 1];
            uint8_t right = buf[idx + 1];

            int n8 = (buf[idx - buf_w - 1] > 0) + (up > 0) +
                     (buf[idx - buf_w + 1] > 0) + (left > 0) +
                     (right > 0) + (buf[idx + buf_w - 1] > 0) +
                     (down > 0) + (buf[idx + buf_w + 1] > 0);
            int strong_card = (up > 96) + (down > 96) + (left > 96) + (right > 96);

            if (n8 >= 5 && strong_card >= 2) {
                int card_count = (up > 0) + (down > 0) + (left > 0) + (right > 0);
                int sum = up + down + left + right;
                buf[idx] = (card_count > 0) ? (uint8_t)(sum / card_count) : 128;
            }
        }
    }

    return mp_obj_new_int(adv_px);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(slugfont_render_glyph_obj, 7, 7, slugfont_render_glyph);

// ============================================================================
// METHOD: font.render(text, px_size, buf, buf_w, x=0, y=0)
// Renders a string with advance widths and kerning
// Returns total advance in pixels
// ============================================================================

static mp_obj_t slugfont_render(size_t n_args, const mp_obj_t *args) {
    slugfont_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    const char *text     = mp_obj_str_get_str(args[1]);
    float px_size        = mp_obj_get_float(args[2]);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[3], &bufinfo, MP_BUFFER_WRITE);

    int pen_x_start = (n_args > 5) ? mp_obj_get_int(args[5]) : 0;
    int pen_y       = (n_args > 6) ? mp_obj_get_int(args[6]) : 0;

    float scale = px_size / (float)(self->ascender + self->descender);
    float pen_x = (float)pen_x_start;

    // Iterate UTF-8 codepoints
    const uint8_t *p = (const uint8_t *)text;
    uint32_t prev_cp = 0;
    int first = 1;

    while (*p) {
        // Decode UTF-8
        uint32_t cp;
        if (*p < 0x80) {
            cp = *p++;
        } else if (*p < 0xE0) {
            cp = (*p++ & 0x1F) << 6;
            if (*p) cp |= (*p++ & 0x3F);
        } else if (*p < 0xF0) {
            cp = (*p++ & 0x0F) << 12;
            if (*p) cp |= (*p++ & 0x3F) << 6;
            if (*p) cp |= (*p++ & 0x3F);
        } else {
            cp = (*p++ & 0x07) << 18;
            if (*p) cp |= (*p++ & 0x3F) << 12;
            if (*p) cp |= (*p++ & 0x3F) << 6;
            if (*p) cp |= (*p++ & 0x3F);
        }

        // Apply kerning
        if (!first) {
            int16_t kern = slugfont_get_kern(self, (uint16_t)prev_cp, (uint16_t)cp);
            pen_x += kern * scale;
        }

        // Build args for render_glyph
        mp_obj_t glyph_args[7] = {
            args[0],                              // self
            mp_obj_new_int(cp),                   // codepoint
            args[2],                              // px_size
            args[3],                              // buf
            args[4],                              // buf_w
            mp_obj_new_int((int)(pen_x + 0.5f)),  // x
            mp_obj_new_int(pen_y),                // y
        };
        slugfont_render_glyph(7, glyph_args);

        // Advance pen by glyph's actual advance width (not pixel width)
        const uint8_t *g = slugfont_find_glyph(self, cp);
        float actual_adv;
        if (g) {
            actual_adv = rd_u16(g + 4) * scale;
        } else {
            actual_adv = self->space_advance * scale;
        }
        pen_x += actual_adv;

        prev_cp = cp;
        first = 0;
    }

    return mp_obj_new_int((int)(pen_x - pen_x_start + 0.5f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(slugfont_render_obj, 5, 7, slugfont_render);

// ============================================================================
// METHOD: font.metrics(px_size) -> (ascender, descender, line_height)
// ============================================================================

static mp_obj_t slugfont_metrics(mp_obj_t self_in, mp_obj_t px_size_in) {
    slugfont_obj_t *self = MP_OBJ_TO_PTR(self_in);
    float px_size = mp_obj_get_float(px_size_in);
    float scale = px_size / (float)(self->ascender + self->descender);

    mp_obj_t tuple[3] = {
        mp_obj_new_int((int)(self->ascender * scale + 0.5f)),
        mp_obj_new_int((int)(self->descender * scale + 0.5f)),
        mp_obj_new_int((int)((self->ascender + self->descender + self->line_gap) * scale + 0.5f)),
    };
    return mp_obj_new_tuple(3, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_2(slugfont_metrics_obj, slugfont_metrics);

// ============================================================================
// METHOD: font.text_width(text, px_size) -> int (pixels)
// ============================================================================

static mp_obj_t slugfont_text_width(mp_obj_t self_in, mp_obj_t text_in, mp_obj_t px_size_in) {
    slugfont_obj_t *self = MP_OBJ_TO_PTR(self_in);
    const char *text = mp_obj_str_get_str(text_in);
    float px_size = mp_obj_get_float(px_size_in);
    float scale = px_size / (float)(self->ascender + self->descender);

    float total = 0.0f;
    const uint8_t *p = (const uint8_t *)text;
    uint32_t prev_cp = 0;
    int first = 1;

    while (*p) {
        uint32_t cp;
        if (*p < 0x80) { cp = *p++; }
        else if (*p < 0xE0) { cp = (*p++ & 0x1F) << 6; if (*p) cp |= (*p++ & 0x3F); }
        else if (*p < 0xF0) { cp = (*p++ & 0x0F) << 12; if (*p) cp |= (*p++ & 0x3F) << 6; if (*p) cp |= (*p++ & 0x3F); }
        else { cp = (*p++ & 0x07) << 18; if (*p) cp |= (*p++ & 0x3F) << 12; if (*p) cp |= (*p++ & 0x3F) << 6; if (*p) cp |= (*p++ & 0x3F); }

        if (!first) {
            total += slugfont_get_kern(self, (uint16_t)prev_cp, (uint16_t)cp) * scale;
        }

        const uint8_t *g = slugfont_find_glyph(self, cp);
        total += (g ? rd_u16(g + 4) : self->space_advance) * scale;

        prev_cp = cp;
        first = 0;
    }

    return mp_obj_new_int((int)(total + 0.5f));
}
static MP_DEFINE_CONST_FUN_OBJ_3(slugfont_text_width_obj, slugfont_text_width);

// ============================================================================
// METHOD: font.info() -> dict with font metadata
// ============================================================================

static mp_obj_t slugfont_info(mp_obj_t self_in) {
    slugfont_obj_t *self = MP_OBJ_TO_PTR(self_in);

    mp_obj_dict_t *dict = mp_obj_new_dict(8);
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_glyphs),      mp_obj_new_int(self->glyph_count));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_bands),       mp_obj_new_int(self->n_bands));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_units_per_em), mp_obj_new_int(self->units_per_em));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_ascender),    mp_obj_new_int(self->ascender));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_descender),   mp_obj_new_int(self->descender));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_kerns),       mp_obj_new_int(self->kern_count));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_size),        mp_obj_new_int(self->data_len));

    return MP_OBJ_FROM_PTR(dict);
}
static MP_DEFINE_CONST_FUN_OBJ_1(slugfont_info_obj, slugfont_info);

// ============================================================================
// METHOD: font.bold([value]) -> get/set weight boost (sqrt coverage)
// No args: returns current state. With arg: sets it.
// ============================================================================

static mp_obj_t slugfont_bold(size_t n_args, const mp_obj_t *args) {
    slugfont_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args > 1) {
        self->weight_boost = mp_obj_is_true(args[1]) ? 1 : 0;
    }
    return mp_obj_new_bool(self->weight_boost);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(slugfont_bold_obj, 1, 2, slugfont_bold);

// ============================================================================
// TYPE DEFINITION
// ============================================================================

static const mp_rom_map_elem_t slugfont_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_render_glyph), MP_ROM_PTR(&slugfont_render_glyph_obj) },
    { MP_ROM_QSTR(MP_QSTR_render),       MP_ROM_PTR(&slugfont_render_obj) },
    { MP_ROM_QSTR(MP_QSTR_metrics),      MP_ROM_PTR(&slugfont_metrics_obj) },
    { MP_ROM_QSTR(MP_QSTR_text_width),   MP_ROM_PTR(&slugfont_text_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_info),         MP_ROM_PTR(&slugfont_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_bold),         MP_ROM_PTR(&slugfont_bold_obj) },
};
static MP_DEFINE_CONST_DICT(slugfont_locals_dict, slugfont_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    slugfont_type,
    MP_QSTR_SlugFont,
    MP_TYPE_FLAG_NONE,
    make_new, slugfont_make_new,
    locals_dict, &slugfont_locals_dict
);

// ============================================================================
// MODULE DEFINITION: warpig_font
// ============================================================================

static const mp_rom_map_elem_t warpig_font_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),  MP_ROM_QSTR(MP_QSTR_warpig_font) },
    { MP_ROM_QSTR(MP_QSTR_SlugFont),  MP_ROM_PTR(&slugfont_type) },
};
static MP_DEFINE_CONST_DICT(warpig_font_globals, warpig_font_globals_table);

const mp_obj_module_t warpig_font_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&warpig_font_globals,
};

MP_REGISTER_MODULE(MP_QSTR_warpig_font, warpig_font_module);
