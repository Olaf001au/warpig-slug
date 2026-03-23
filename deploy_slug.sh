#!/bin/bash
# deploy_slug.sh — Install WarpPig Slug font system into /opt/warpig
# Run on Pi 5: sudo bash deploy_slug.sh
#
# This script:
#   1. Copies the C module to firmware/warpig_font/
#   2. Copies the builder tool to project root
#   3. Copies pre-built .slugfont files to fonts/slugfont/
#   4. Copies source TTFs to fonts/ttf/
#   5. Copies Slug reference shaders to slug_reference/
#   6. Sets ownership to pi:pi

set -e

WARPIG=/opt/warpig
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== WarpPig Slug Font System Installer ==="
echo "Source:  $SCRIPT_DIR"
echo "Target:  $WARPIG"
echo ""

# Check we're on the Pi
if [ ! -d "$WARPIG" ]; then
    echo "ERROR: $WARPIG not found. Run this on the Pi 5."
    exit 1
fi

# 1. C Module
echo "[1/6] Installing C module..."
mkdir -p "$WARPIG/firmware/warpig_font"
cp "$SCRIPT_DIR/firmware/warpig_font/warpig_font.c"      "$WARPIG/firmware/warpig_font/"
cp "$SCRIPT_DIR/firmware/warpig_font/micropython.mk"      "$WARPIG/firmware/warpig_font/"
cp "$SCRIPT_DIR/firmware/warpig_font/micropython.cmake"   "$WARPIG/firmware/warpig_font/"

# 2. Builder tool
echo "[2/6] Installing builder tool..."
cp "$SCRIPT_DIR/warpig_slug.py" "$WARPIG/warpig_slug.py"

# 3. Pre-built .slugfont files
echo "[3/6] Installing .slugfont files..."
mkdir -p "$WARPIG/fonts/slugfont"
cp "$SCRIPT_DIR/fonts/slugfont/"*.slugfont "$WARPIG/fonts/slugfont/"

# 4. Source TTFs
echo "[4/6] Installing source TTF files..."
mkdir -p "$WARPIG/fonts/ttf"
cp "$SCRIPT_DIR/fonts/ttf/"*.ttf "$WARPIG/fonts/ttf/" 2>/dev/null || true

# 5. Slug reference shaders
echo "[5/6] Installing Slug reference shaders..."
mkdir -p "$WARPIG/slug_reference"
cp "$SCRIPT_DIR/slug_reference/"* "$WARPIG/slug_reference/" 2>/dev/null || true

# 6. README
cp "$SCRIPT_DIR/README.md" "$WARPIG/firmware/warpig_font/README.md"

# Fix ownership
echo "[6/6] Fixing ownership..."
chown -R pi:pi "$WARPIG/firmware/warpig_font" \
               "$WARPIG/warpig_slug.py" \
               "$WARPIG/fonts" \
               "$WARPIG/slug_reference" 2>/dev/null || true

echo ""
echo "=== Done! ==="
echo ""
echo "Files installed:"
echo "  $WARPIG/firmware/warpig_font/    — C module (warpig_font.c + build files)"
echo "  $WARPIG/warpig_slug.py           — Font builder (TTF → .slugfont)"
echo "  $WARPIG/fonts/slugfont/          — Pre-built font binaries"
echo "  $WARPIG/fonts/ttf/               — Source TTF files"
echo "  $WARPIG/slug_reference/          — Eric Lengyel's reference shaders"
echo ""
echo "Installed .slugfont files:"
ls -lh "$WARPIG/fonts/slugfont/"
echo ""
echo "Next steps:"
echo "  1. Build firmware:  cd /opt/micropython_build/micropython/ports/esp32"
echo "     make BOARD=WARPIG_PROS3 USER_C_MODULES=$WARPIG/firmware/warpig_font/micropython.mk"
echo "  2. Deploy font:  mpremote cp $WARPIG/fonts/slugfont/0xProto.slugfont :/fonts/"
echo "  3. Test on board:"
echo "     import warpig_font"
echo "     font = warpig_font.SlugFont('/fonts/0xProto.slugfont')"
echo "     buf = bytearray(320 * 40)"
echo "     font.render('Hello WarpPig!', 24, buf, 320)"
