"""
WarpPig Slug v3 — Bézier font rendering for microcontrollers
Credit: Slug shader code Copyright 2017 by Eric Lengyel.

This file has TWO halves:
  1. BUILDER (runs on Pi 5) — reads TTF, builds .slugfont binary
  2. RENDERER (runs on MCU) — reads .slugfont, renders to framebuf

The renderer half uses ONLY: math, struct, array — no fonttools, no OS deps.
It's designed to be copy-pasted onto MicroPython boards or compiled as C.
"""

import math
import struct
import time

# ============================================================
# SLUGFONT BINARY FORMAT v3
# ============================================================
#
# All integers little-endian. All floats are float32 (4 bytes).
#
# HEADER (32 bytes):
#   8B  magic "WARPSLUG"
#   1B  version (3)
#   1B  n_bands
#   2B  glyph_count
#   2B  units_per_em
#   2B  ascender (positive, in font units)
#   2B  descender (positive, in font units — stored as abs value)
#   2B  line_gap (font units)
#   2B  space_advance (font units)
#   2B  kern_count
#   4B  curve_pool_offset (byte offset from file start)
#   2B  band_index_pool_offset (byte offset from file start, 0 = no band indices)
#
# GLYPH DIRECTORY (per glyph, variable size):
#   4B  codepoint (u32)
#   2B  advance_width (font units)
#   2B  lsb (left side bearing, font units, signed as u16)
#   2B  bbox_x1 (i16)
#   2B  bbox_y1 (i16)
#   2B  bbox_x2 (i16)
#   2B  bbox_y2 (i16)
#   2B  curve_offset (index into curve pool)
#   2B  curve_count
#   Per band (n_bands horizontal + n_bands vertical = 2*n_bands):
#     2B  band_curve_count
#     2B  band_index_offset (into band index pool)
#
# KERN TABLE (per entry, 8 bytes):
#   2B  left codepoint (u16, truncated for BMP)
#   2B  right codepoint (u16)
#   2B  kern value (i16, font units)
#   2B  padding (0)
#
# BAND INDEX POOL:
#   Array of u16 — each is a curve index within the glyph's curve list
#
# CURVE POOL:
#   Per curve: 6x float32 (x1, y1, x2, y2, x3, y3) = 24 bytes
#   Coordinates are in font units, offset so glyph bbox min = (0,0)

MAGIC = b'WARPSLUG'
VERSION = 3
HEADER_SIZE = 32
GLYPH_FIXED_SIZE = 20  # before band entries
KERN_ENTRY_SIZE = 8
BAND_ENTRY_SIZE = 4
CURVE_SIZE = 24  # 6 * float32
BAND_EPSILON_FRAC = 1.0 / 1024.0

# ============================================================
# RENDERER — MicroPython compatible, no fonttools
# ============================================================

def _clamp01(x):
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def _calc_root_code(y1, y2, y3):
    s = (1 if y1 < 0.0 else 0) | ((1 if y2 < 0.0 else 0) << 1) | ((1 if y3 < 0.0 else 0) << 2)
    code = (0x2E74 >> s) & 0x0101
    return code & 1, code >> 8


def _solve_horiz(p1x, p1y, p2x, p2y, p3x, p3y):
    ay = p1y - 2.0 * p2y + p3y
    by = p1y - p2y
    ax = p1x - 2.0 * p2x + p3x
    bx = p1x - p2x
    if abs(ay) < 1.525878906e-05:  # 1/65536
        if abs(by) < 1e-12:
            return 0.0, 0.0
        t = p1y / (2.0 * by)
        rx = (ax * t - bx * 2.0) * t + p1x
        return rx, rx
    d = math.sqrt(max(by * by - ay * p1y, 0.0))
    ra = 1.0 / ay
    t1 = (by - d) * ra
    t2 = (by + d) * ra
    return (ax * t1 - bx * 2.0) * t1 + p1x, (ax * t2 - bx * 2.0) * t2 + p1x


def _solve_vert(p1x, p1y, p2x, p2y, p3x, p3y):
    ax = p1x - 2.0 * p2x + p3x
    bx = p1x - p2x
    ay = p1y - 2.0 * p2y + p3y
    by = p1y - p2y
    if abs(ax) < 1.525878906e-05:
        if abs(bx) < 1e-12:
            return 0.0, 0.0
        t = p1x / (2.0 * bx)
        ry = (ay * t - by * 2.0) * t + p1y
        return ry, ry
    d = math.sqrt(max(bx * bx - ax * p1x, 0.0))
    ra = 1.0 / ax
    t1 = (bx - d) * ra
    t2 = (bx + d) * ra
    return (ay * t1 - by * 2.0) * t1 + p1y, (ay * t2 - by * 2.0) * t2 + p1y


def _trace_h(curves, curve_off, count, band_idx, band_pool, sx, sy, ppe):
    """Trace horizontal ray through band curves. Returns (cov, wgt)."""
    cov = 0.0
    wgt = 0.0
    bp_off = band_idx  # offset into band_pool
    for i in range(count):
        ci = band_pool[bp_off + i]  # curve index
        ci6 = (curve_off + ci) * 6
        p1x = curves[ci6] - sx
        p1y = curves[ci6 + 1] - sy
        p2x = curves[ci6 + 2] - sx
        p2y = curves[ci6 + 3] - sy
        p3x = curves[ci6 + 4] - sx
        p3y = curves[ci6 + 5] - sy
        mx = p1x
        if p2x > mx: mx = p2x
        if p3x > mx: mx = p3x
        if mx * ppe < -0.5:
            break
        r1, r2 = _calc_root_code(p1y, p2y, p3y)
        if not r1 and not r2:
            continue
        rx1, rx2 = _solve_horiz(p1x, p1y, p2x, p2y, p3x, p3y)
        if r1:
            r = rx1 * ppe
            cov += _clamp01(r + 0.5)
            w = _clamp01(1.0 - abs(r) * 2.0)
            if w > wgt: wgt = w
        if r2:
            r = rx2 * ppe
            cov -= _clamp01(r + 0.5)
            w = _clamp01(1.0 - abs(r) * 2.0)
            if w > wgt: wgt = w
    return cov, wgt


def _trace_v(curves, curve_off, count, band_idx, band_pool, sx, sy, ppe):
    """Trace vertical ray through band curves. Returns (cov, wgt)."""
    cov = 0.0
    wgt = 0.0
    bp_off = band_idx
    for i in range(count):
        ci = band_pool[bp_off + i]
        ci6 = (curve_off + ci) * 6
        p1x = curves[ci6] - sx
        p1y = curves[ci6 + 1] - sy
        p2x = curves[ci6 + 2] - sx
        p2y = curves[ci6 + 3] - sy
        p3x = curves[ci6 + 4] - sx
        p3y = curves[ci6 + 5] - sy
        my = p1y
        if p2y > my: my = p2y
        if p3y > my: my = p3y
        if my * ppe < -0.5:
            break
        r1, r2 = _calc_root_code(p1x, p2x, p3x)
        if not r1 and not r2:
            continue
        ry1, ry2 = _solve_vert(p1x, p1y, p2x, p2y, p3x, p3y)
        if r1:
            r = ry1 * ppe
            cov -= _clamp01(r + 0.5)
            w = _clamp01(1.0 - abs(r) * 2.0)
            if w > wgt: wgt = w
        if r2:
            r = ry2 * ppe
            cov += _clamp01(r + 0.5)
            w = _clamp01(1.0 - abs(r) * 2.0)
            if w > wgt: wgt = w
    return cov, wgt


class SlugFont:
    """Loads and renders a .slugfont file. MicroPython compatible."""

    def __init__(self, path):
        with open(path, 'rb') as f:
            data = f.read()
        self._parse(data)

    def _parse(self, data):
        # Header
        magic = data[0:8]
        if magic != MAGIC:
            raise ValueError("Not a WARPSLUG file")

        (ver, self.n_bands, glyph_count, self.units_per_em,
         self.ascender, self.descender, self.line_gap,
         self.space_advance, kern_count,
         curve_pool_off, band_pool_off) = struct.unpack_from(
            '<BBHHHHHHH II', data, 8)

        if ver != VERSION:
            raise ValueError(f"Version {ver}, expected {VERSION}")

        # Parse glyph directory
        self.glyphs = {}  # codepoint -> glyph dict
        band_per_glyph = self.n_bands * 2
        glyph_entry_size = GLYPH_FIXED_SIZE + band_per_glyph * BAND_ENTRY_SIZE
        off = HEADER_SIZE

        for _ in range(glyph_count):
            cp, adv, lsb = struct.unpack_from('<IHh', data, off)
            bx1, by1, bx2, by2 = struct.unpack_from('<hhhh', data, off + 8)
            c_off, c_cnt = struct.unpack_from('<HH', data, off + 16)

            bands = []
            boff = off + GLYPH_FIXED_SIZE
            for b in range(band_per_glyph):
                bc, bi = struct.unpack_from('<HH', data, boff)
                bands.append((bc, bi))
                boff += BAND_ENTRY_SIZE

            self.glyphs[cp] = {
                'advance': adv,
                'lsb': lsb,
                'bbox': (bx1, by1, bx2, by2),
                'curve_offset': c_off,
                'curve_count': c_cnt,
                'bands': bands,  # [0..n_bands-1] = h-bands, [n_bands..2*n_bands-1] = v-bands
            }
            off += glyph_entry_size

        # Kern table
        self.kerns = {}
        for _ in range(kern_count):
            lc, rc, kv, _ = struct.unpack_from('<HHhH', data, off)
            self.kerns[(lc, rc)] = kv
            off += KERN_ENTRY_SIZE

        # Band index pool (array of u16)
        bp_data = data[band_pool_off:curve_pool_off] if band_pool_off > 0 else b''
        self.band_pool = []
        for i in range(0, len(bp_data), 2):
            self.band_pool.append(struct.unpack_from('<H', bp_data, i)[0])

        # Curve pool (array of float32, stored as flat list: x1,y1,x2,y2,x3,y3,...)
        cp_data = data[curve_pool_off:]
        self.curves = []
        for i in range(0, len(cp_data), 4):
            if i + 4 <= len(cp_data):
                self.curves.append(struct.unpack_from('<f', cp_data, i)[0])

    def get_scale(self, px_size):
        """Pixels per font unit for a given pixel size (ascender height)."""
        return px_size / (self.ascender + self.descender)

    def line_height(self, px_size):
        """Total line height in pixels."""
        scale = self.get_scale(px_size)
        return int((self.ascender + self.descender + self.line_gap) * scale + 0.5)

    def get_kern(self, cp_left, cp_right):
        """Get kerning value in font units between two codepoints."""
        return self.kerns.get((cp_left & 0xFFFF, cp_right & 0xFFFF), 0)

    def render_glyph(self, codepoint, px_size, buf, buf_w, buf_x=0, buf_y=0):
        """Render a single glyph into a grayscale buffer.

        buf: bytearray, row-major, width=buf_w
        buf_x, buf_y: top-left pixel position in buffer
        Returns: advance width in pixels
        """
        g = self.glyphs.get(codepoint)
        if g is None:
            # Space or missing glyph
            scale = self.get_scale(px_size)
            return int(self.space_advance * scale + 0.5)

        scale = self.get_scale(px_size)
        adv_px = int(g['advance'] * scale + 0.5)

        bx1, by1, bx2, by2 = g['bbox']
        gw = bx2 - bx1
        gh = by2 - by1
        if gw <= 0 or gh <= 0:
            return adv_px

        # Pixel dimensions of the glyph bounding box
        lsb_px = g['lsb'] * scale
        gw_px = int(gw * scale + 0.5)
        gh_px = int(gh * scale + 0.5)
        if gw_px < 1 or gh_px < 1:
            return adv_px

        # Where to place the glyph vertically
        # ascender is the baseline position from top of line
        baseline_y = int(self.ascender * scale + 0.5)
        glyph_top = baseline_y - int(by2 * scale + 0.5)  # by2 is top in font coords

        fu_per_px_x = gw / gw_px
        fu_per_px_y = gh / gh_px
        ppe_x = 1.0 / fu_per_px_x
        ppe_y = 1.0 / fu_per_px_y

        nb = self.n_bands
        band_h = gh / nb
        band_w = gw / nb
        c_off = g['curve_offset']
        bands = g['bands']

        glyph_left = int(buf_x + lsb_px + 0.5)

        for py in range(gh_px):
            ey = (gh_px - 1 - py) * fu_per_px_y  # em-space y (bottom-up)
            hbi = int(ey / band_h)
            if hbi >= nb: hbi = nb - 1
            if hbi < 0: hbi = 0
            h_count, h_bp = bands[hbi]

            row = buf_y + glyph_top + py
            if row < 0 or row * buf_w >= len(buf):
                continue

            for px in range(gw_px):
                ex = px * fu_per_px_x
                vbi = int(ex / band_w)
                if vbi >= nb: vbi = nb - 1
                if vbi < 0: vbi = 0
                v_count, v_bp = bands[nb + vbi]

                xcov, xwgt = _trace_h(self.curves, c_off, h_count, h_bp,
                                      self.band_pool, ex, ey, ppe_x)
                ycov, ywgt = _trace_v(self.curves, c_off, v_count, v_bp,
                                      self.band_pool, ex, ey, ppe_y)

                # Lengyel's weighted coverage combination
                wsum = xwgt + ywgt
                if wsum < 1.525878906e-05:
                    wsum = 1.525878906e-05
                weighted = abs(xcov * xwgt + ycov * ywgt) / wsum
                conservative = abs(xcov)
                ay = abs(ycov)
                if ay < conservative:
                    conservative = ay
                cov = weighted if weighted > conservative else conservative
                if cov > 1.0: cov = 1.0

                col = glyph_left + px
                if 0 <= col < buf_w:
                    idx = row * buf_w + col
                    if 0 <= idx < len(buf):
                        old = buf[idx]
                        new = int(cov * 255.0 + 0.5)
                        # Alpha composite (max blend for overlapping glyphs)
                        buf[idx] = new if new > old else old

        return adv_px

    def render_string(self, text, px_size, buf=None, buf_w=None, x=0, y=0):
        """Render a string with proper advance widths and kerning.

        If buf is None, calculates required size and allocates.
        Returns (buf, buf_w, buf_h) or just renders into provided buf.
        """
        scale = self.get_scale(px_size)
        lh = self.line_height(px_size)

        # Calculate total width with kerning
        total_advance = 0.0
        codepoints = [ord(c) for c in text]
        for i, cp in enumerate(codepoints):
            g = self.glyphs.get(cp)
            if g:
                total_advance += g['advance']
            else:
                total_advance += self.space_advance
            if i < len(codepoints) - 1:
                total_advance += self.get_kern(cp, codepoints[i + 1])

        total_w = int(total_advance * scale + 0.5)

        if buf is None:
            buf_w = total_w + x
            buf = bytearray(buf_w * lh)
        buf_h = len(buf) // buf_w if buf_w > 0 else 0

        # Render each glyph
        pen_x = x
        for i, cp in enumerate(codepoints):
            adv = self.render_glyph(cp, px_size, buf, buf_w, int(pen_x + 0.5), y)
            g = self.glyphs.get(cp)
            actual_advance = g['advance'] if g else self.space_advance
            pen_x += actual_advance * scale
            if i < len(codepoints) - 1:
                pen_x += self.get_kern(cp, codepoints[i + 1]) * scale

        return buf, buf_w, lh


# ============================================================
# BUILDER — runs on Pi 5 with fonttools
# ============================================================

def build_slugfont(ttf_path, output_path, chars=None, n_bands=8):
    """Build a .slugfont file from a TTF.

    chars: string of characters to include, or None for printable ASCII.
    Returns: dict with stats.
    """
    from fontTools.ttLib import TTFont
    from fontTools.pens.recordingPen import DecomposingRecordingPen

    font = TTFont(ttf_path)
    cmap_table = font.getBestCmap()
    glyf_table = font['glyf']
    hmtx_table = font['hmtx']
    head = font['head']
    os2 = font['OS/2']

    units_per_em = head.unitsPerEm
    ascender = os2.sTypoAscender
    descender = abs(os2.sTypoDescender)
    line_gap = os2.sTypoLineGap

    # Space advance
    space_name = cmap_table.get(32)
    space_advance = hmtx_table[space_name][0] if space_name else units_per_em // 4

    if chars is None:
        chars = ''.join(chr(c) for c in range(32, 127))

    # Extract kerning
    kern_pairs = {}
    kern_table = font.get('kern')
    if kern_table:
        char_set = set(ord(c) for c in chars)
        for table in kern_table.kernTables:
            for (left_name, right_name), value in table.kernTable.items():
                # Reverse-lookup codepoints from glyph names
                for lcp in char_set:
                    if cmap_table.get(lcp) == left_name:
                        for rcp in char_set:
                            if cmap_table.get(rcp) == right_name:
                                kern_pairs[(lcp, rcp)] = value

    # Process glyphs
    all_curves = []  # flat float list: x1,y1,x2,y2,x3,y3,...
    all_band_indices = []  # flat u16 list
    glyph_entries = []

    for char in chars:
        cp = ord(char)
        glyph_name = cmap_table.get(cp)
        if not glyph_name:
            continue

        glyph = glyf_table[glyph_name]
        advance, lsb = hmtx_table[glyph_name]

        # Skip empty glyphs (spaces etc) — store just metrics
        if glyph.numberOfContours == 0 or glyph.numberOfContours == -1:
            # Check for composite
            pen = DecomposingRecordingPen(glyf_table)
            try:
                glyph.draw(pen, glyf_table)
            except Exception:
                pass
            if not any(op != 'closePath' and op != 'endPath' and op != 'moveTo'
                       for op, _ in pen.value):
                continue

        pen = DecomposingRecordingPen(glyf_table)
        glyph.draw(pen, glyf_table)

        ox, oy = glyph.xMin, glyph.yMin
        gw = glyph.xMax - glyph.xMin
        gh = glyph.yMax - glyph.yMin
        if gw <= 0 or gh <= 0:
            continue

        # Extract curves
        curves = []
        cx, cy = 0.0, 0.0
        for op, args in pen.value:
            if op == 'moveTo':
                cx, cy = args[0][0] - ox, args[0][1] - oy
            elif op == 'lineTo':
                nx, ny = args[0][0] - ox, args[0][1] - oy
                curves.append((cx, cy, (cx + nx) * 0.5, (cy + ny) * 0.5, nx, ny))
                cx, cy = nx, ny
            elif op == 'qCurveTo':
                off = [(a[0] - ox, a[1] - oy) for a in args[:-1]]
                end = (args[-1][0] - ox, args[-1][1] - oy)
                if len(off) == 0:
                    curves.append((cx, cy, (cx + end[0]) * 0.5, (cy + end[1]) * 0.5,
                                   end[0], end[1]))
                elif len(off) == 1:
                    curves.append((cx, cy, off[0][0], off[0][1], end[0], end[1]))
                else:
                    pts = [(cx, cy)] + off + [end]
                    for i in range(1, len(pts) - 1):
                        sx = pts[0][0] if i == 1 else (pts[i - 1][0] + pts[i][0]) * 0.5
                        sy = pts[0][1] if i == 1 else (pts[i - 1][1] + pts[i][1]) * 0.5
                        ex = pts[-1][0] if i == len(pts) - 2 else (pts[i][0] + pts[i + 1][0]) * 0.5
                        ey = pts[-1][1] if i == len(pts) - 2 else (pts[i][1] + pts[i + 1][1]) * 0.5
                        curves.append((sx, sy, pts[i][0], pts[i][1], ex, ey))
                cx, cy = end
            elif op == 'curveTo':
                # Cubic approx: split at midpoint
                p0x, p0y = cx, cy
                p1x, p1y = args[0][0] - ox, args[0][1] - oy
                p2x, p2y = args[1][0] - ox, args[1][1] - oy
                p3x, p3y = args[2][0] - ox, args[2][1] - oy
                mx = (p0x + 3 * p1x + 3 * p2x + p3x) * 0.125
                my = (p0y + 3 * p1y + 3 * p2y + p3y) * 0.125
                curves.append((p0x, p0y, (p0x + p1x) * 0.5, (p0y + p1y) * 0.5, mx, my))
                curves.append((mx, my, (p2x + p3x) * 0.5, (p2y + p3y) * 0.5, p3x, p3y))
                cx, cy = p3x, p3y

        if not curves:
            continue

        # Fix degenerate control points
        fixed = []
        for c in curves:
            x1, y1, x2, y2, x3, y3 = c
            if (x2 == x1 and y2 == y1) or (x2 == x3 and y2 == y3):
                x2, y2 = (x1 + x3) * 0.5, (y1 + y3) * 0.5
            fixed.append((x1, y1, x2, y2, x3, y3))
        curves = fixed

        # Build bands with epsilon overlap
        nb = min(n_bands, max(gw, gh) // 2) if max(gw, gh) > n_bands * 2 else max(1, min(n_bands, max(gw, gh)))
        band_h = gh / nb
        band_w = gw / nb
        eps_h = band_h * BAND_EPSILON_FRAC
        eps_w = band_w * BAND_EPSILON_FRAC

        curve_offset = len(all_curves) // 6  # curves stored as 6 floats each

        # Store curves in pool
        for c in curves:
            all_curves.extend(c)

        # Build band assignments
        band_data = []  # list of (count, band_pool_offset)
        # Horizontal bands
        for b in range(nb):
            blo = b * band_h - eps_h
            bhi = (b + 1) * band_h + eps_h
            band_offset = len(all_band_indices)
            count = 0
            # Collect matching curves, sorted by descending max-x
            matching = []
            for ci, c in enumerate(curves):
                y_min = min(c[1], c[3], c[5])
                y_max = max(c[1], c[3], c[5])
                if c[1] == c[3] == c[5]:  # skip horizontal
                    continue
                if y_max >= blo and y_min <= bhi:
                    x_max = max(c[0], c[2], c[4])
                    matching.append((ci, x_max))
            matching.sort(key=lambda t: -t[1])
            for ci, _ in matching:
                all_band_indices.append(ci)
                count += 1
            band_data.append((count, band_offset))

        # Vertical bands
        for b in range(nb):
            blo = b * band_w - eps_w
            bhi = (b + 1) * band_w + eps_w
            band_offset = len(all_band_indices)
            count = 0
            matching = []
            for ci, c in enumerate(curves):
                x_min = min(c[0], c[2], c[4])
                x_max = max(c[0], c[2], c[4])
                if c[0] == c[2] == c[4]:  # skip vertical
                    continue
                if x_max >= blo and x_min <= bhi:
                    y_max = max(c[1], c[3], c[5])
                    matching.append((ci, y_max))
            matching.sort(key=lambda t: -t[1])
            for ci, _ in matching:
                all_band_indices.append(ci)
                count += 1
            band_data.append((count, band_offset))

        # Pad band_data to exactly 2*n_bands entries
        while len(band_data) < 2 * n_bands:
            band_data.append((0, len(all_band_indices)))

        glyph_entries.append({
            'codepoint': cp,
            'advance': advance,
            'lsb': lsb,
            'bbox': (0, 0, gw, gh),
            'curve_offset': curve_offset,
            'curve_count': len(curves),
            'band_data': band_data[:2 * n_bands],
        })

    font.close()

    # Layout the binary
    glyph_entry_size = GLYPH_FIXED_SIZE + 2 * n_bands * BAND_ENTRY_SIZE
    kern_list = list(kern_pairs.items())
    kern_section_size = len(kern_list) * KERN_ENTRY_SIZE
    band_pool_size = len(all_band_indices) * 2
    curve_pool_size = len(all_curves) * 4  # float32

    band_pool_off = HEADER_SIZE + len(glyph_entries) * glyph_entry_size + kern_section_size
    curve_pool_off = band_pool_off + band_pool_size

    # Write
    with open(output_path, 'wb') as f:
        # Header
        f.write(MAGIC)
        f.write(struct.pack('<BBH', VERSION, n_bands, len(glyph_entries)))
        f.write(struct.pack('<HHHHH', units_per_em, ascender, descender,
                            line_gap, space_advance))
        f.write(struct.pack('<H', len(kern_list)))
        f.write(struct.pack('<II', curve_pool_off, band_pool_off))

        # Glyph directory
        for ge in glyph_entries:
            bx1, by1, bx2, by2 = ge['bbox']
            f.write(struct.pack('<I', ge['codepoint']))
            f.write(struct.pack('<Hh', ge['advance'], ge['lsb']))
            f.write(struct.pack('<hhhh', bx1, by1, bx2, by2))
            f.write(struct.pack('<HH', ge['curve_offset'], ge['curve_count']))
            for count, offset in ge['band_data']:
                f.write(struct.pack('<HH', count, offset))

        # Kern table
        for (lcp, rcp), value in kern_list:
            f.write(struct.pack('<HHhH', lcp & 0xFFFF, rcp & 0xFFFF, value, 0))

        # Band index pool
        for idx in all_band_indices:
            f.write(struct.pack('<H', idx))

        # Curve pool
        for v in all_curves:
            f.write(struct.pack('<f', v))

    total_size = curve_pool_off + curve_pool_size
    return {
        'glyphs': len(glyph_entries),
        'curves': len(all_curves) // 6,
        'kerns': len(kern_list),
        'band_indices': len(all_band_indices),
        'file_size': total_size,
        'n_bands': n_bands,
    }


# ============================================================
# ASCII PREVIEW
# ============================================================

def buf_to_ascii(buf, w, h):
    chars = ' .:-=+*#%@'
    lines = []
    for y in range(h):
        line = ''
        for x in range(w):
            v = buf[y * w + x] if y * w + x < len(buf) else 0
            idx = int(v / 256.0 * len(chars))
            if idx >= len(chars): idx = len(chars) - 1
            line += chars[idx]
        lines.append(line)
    return '\n'.join(lines)


# ============================================================
# CLI
# ============================================================

if __name__ == '__main__':
    import sys
    import os

    print("WarpPig Slug v3 — Bézier Font System")
    print("Credit: Slug shader code Copyright 2017 by Eric Lengyel.\n")

    ttf = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    if len(sys.argv) > 1:
        ttf = sys.argv[1]

    # Build .slugfont
    sfont_path = '/tmp/test.slugfont'
    print(f"Building .slugfont from {os.path.basename(ttf)}...")
    stats = build_slugfont(ttf, sfont_path, n_bands=10)
    print(f"  {stats['glyphs']} glyphs, {stats['curves']} curves, "
          f"{stats['kerns']} kern pairs")
    print(f"  {stats['band_indices']} band indices, {stats['n_bands']} bands")
    print(f"  File size: {stats['file_size']:,} bytes "
          f"({stats['file_size']/1024:.1f} KB)\n")

    # Load and render
    print("Loading .slugfont...")
    sf = SlugFont(sfont_path)
    print(f"  units_per_em={sf.units_per_em}, ascender={sf.ascender}, "
          f"descender={sf.descender}")
    print(f"  {len(sf.glyphs)} glyphs, {len(sf.kerns)} kern pairs\n")

    # Render test strings at different sizes
    for px_size in [16, 24, 32]:
        text = "WarpPig Slug!"
        t0 = time.time()
        buf, w, h = sf.render_string(text, px_size)
        elapsed = (time.time() - t0) * 1000
        print(f"'{text}' at {px_size}px — {w}x{h}, {elapsed:.1f}ms")
        print(buf_to_ascii(buf, w, h))
        print()

    # Kerning demo
    print("Kerning demo (24px):")
    for text in ["AVA AWAY", "WAVE LTY", "Top Yale"]:
        buf, w, h = sf.render_string(text, 24)
        print(f"  '{text}':")
        print(buf_to_ascii(buf, w, h))
        print()

    # Size report for microcontroller deployment
    print("=" * 60)
    print("  Deployment Size Estimates")
    print("=" * 60)
    # Minimal ASCII set (digits + upper + lower + punctuation)
    for charset_name, chars in [
        ("Digits only", "0123456789"),
        ("Upper + digits", "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"),
        ("Full ASCII", None),
        ("Upper + lower + digits + punct",
         "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789.,!?:;-+=/()"),
    ]:
        s = build_slugfont(ttf, '/tmp/test_size.slugfont', chars=chars, n_bands=8)
        print(f"  {charset_name:35s}: {s['file_size']:6,} bytes ({s['file_size']/1024:.1f} KB) "
              f"— {s['glyphs']} glyphs, {s['curves']} curves")
