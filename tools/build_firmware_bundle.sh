#!/usr/bin/env bash
# Rebuild every range_test image and refresh firmware/ with ready-to-flash files,
# plus a copy of the Rook v4 Bluetooth companion build from the rook repo.
#
#   tools/build_firmware_bundle.sh            (run from anywhere inside the repo)
#   ROOK_REPO=/path/to/rook tools/build_firmware_bundle.sh
#
# Then commit firmware/ if you want the bundle versioned.
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=firmware
mkdir -p "$OUT"

COMMIT=$(git rev-parse --short HEAD)
if ! git diff --quiet HEAD -- examples variants src platformio.ini; then
  COMMIT="$COMMIT + uncommitted changes"
fi

echo "== building range_test (Heltec V4, Heltec V3, Rook)"
pio run -e heltec_v4_range_test -t mergebin
pio run -e Heltec_v3_range_test -t mergebin
pio run -e Rook_range_test

B=.pio/build
cp "$B/heltec_v4_range_test/firmware.bin"        "$OUT/range_test_heltec_v4_update.bin"
cp "$B/heltec_v4_range_test/firmware-merged.bin" "$OUT/range_test_heltec_v4_full.bin"
cp "$B/Heltec_v3_range_test/firmware.bin"        "$OUT/range_test_heltec_v3_update.bin"
cp "$B/Heltec_v3_range_test/firmware-merged.bin" "$OUT/range_test_heltec_v3_full.bin"
cp "$B/Rook_range_test/firmware.uf2"             "$OUT/range_test_rook.uf2"

ROOK_REPO=${ROOK_REPO:-$HOME/gitwork/rook}
ROOK_COMPANION="$ROOK_REPO/firmware/v_0.4/rook_companion_radio_ble_v0.4.uf2"
if [ -f "$ROOK_COMPANION" ]; then
  cp "$ROOK_COMPANION" "$OUT/rook_v4_companion_ble_meshcore-v1.15.0.uf2"
else
  echo "   (Rook companion not found at $ROOK_COMPANION; keeping any existing copy)"
fi

cat > "$OUT/VERSION.txt" <<EOF
range_test firmware bundle
built:          $(date -u '+%Y-%m-%d %H:%M UTC')
source commit:  $COMMIT (meshcore-range-test, branch $(git rev-parse --abbrev-ref HEAD))
MeshCore base:  v1.14.1 (from MeshCore-simple-sensor) + Heltec V4.3 front-end detection
radio default:  910.525 MHz / BW 62.5 kHz / SF7 / CR5 (US preset; change at runtime with 'set radio')
rook companion: rook repo firmware/v_0.4/rook_companion_radio_ble_v0.4.uf2 (MeshCore v1.15.0)
EOF

(cd "$OUT" && sha256sum *.bin *.uf2 > SHA256SUMS)
echo "== done:"
ls -la "$OUT"
