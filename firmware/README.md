# Ready-to-flash firmware

Prebuilt images for every node in the range test, so you can reflash one without
PlatformIO or the source tree. See `VERSION.txt` for the source commit and build date;
`SHA256SUMS` lists checksums.

| File | Board | Use for |
|---|---|---|
| `range_test_heltec_v4_update.bin` | Heltec V4 / V4.3 | **Updating** a V4 that already runs range_test: **Lincoln, Rover**. Keeps its settings |
| `range_test_heltec_v4_full.bin` | Heltec V4 / V4.3 | A **new or blank** V4 (bootloader + partitions + app) |
| `range_test_heltec_v3_update.bin` | Heltec V3 | **Updating** a V3 that already runs range_test: **Waltham**. Keeps its settings |
| `range_test_heltec_v3_full.bin` | Heltec V3 | A **new or blank** V3 |
| `range_test_rook.uf2` | Rook | range_test on a Rook (relay or pinger, no Wi-Fi) |
| `rook_v4_companion_ble_meshcore-v1.15.0.uf2` | Rook v4 | **Bluetooth companion** for the MeshCore phone app (copied from the `rook` repo) |

Owl Hill and the XIAO repeater run stock MeshCore repeater firmware; they aren't covered here.

**Settings survive an update.** Each node's name, role, Wi-Fi, Bayou keys, contacts,
targets and identity live in a separate flash area that these images don't touch. Only
`esptool erase_flash` wipes them. That gives the node a **new identity**, so you'd have to
exchange cards with the other nodes again.

---

## Heltec V4 (Lincoln, Rover)

The V4's automatic reset over USB is unreliable with this firmware, so use the buttons:

1. Hold **PRG**, tap **RST**, release **PRG**. The board is now in its bootloader.
2. Flash, with one of the three methods below.
3. Tap **RST**. The new firmware starts.

**With esptool** (`pip install esptool`):

```bash
# update a node that already runs range_test (keeps settings)
python3 -m esptool --chip esp32s3 --port /dev/ttyACM0 --before no_reset \
    write_flash 0x10000 range_test_heltec_v4_update.bin

# new / blank board
python3 -m esptool --chip esp32s3 --port /dev/ttyACM0 --before no_reset \
    write_flash 0x0 range_test_heltec_v4_full.bin
```

**In a browser** (Chrome or Edge, nothing to install): open
<https://espressif.github.io/esptool-js/>, click **Connect** and pick the board's port.
Add the file at offset **`0x10000`** (update) or **`0x0`** (full), then click **Program**.

**From the repo:** `pio run -e heltec_v4_range_test -t upload --upload-port /dev/ttyACM0`.

## Heltec V3 (Waltham)

The V3 has its own USB-serial chip, so it resets itself: **no buttons needed**. Its port
is usually `/dev/ttyUSB0`.

```bash
python3 -m esptool --chip esp32s3 --port /dev/ttyUSB0 write_flash 0x10000 range_test_heltec_v3_update.bin
#   new / blank board:                                write_flash 0x0     range_test_heltec_v3_full.bin
```

The browser method works the same way, at the same offsets. From the repo:
`pio run -e Heltec_v3_range_test -t upload --upload-port /dev/ttyUSB0`.

## Rook (UF2: range_test or the Bluetooth companion)

1. Plug the Rook in with a data cable, then **double-tap RESET**. A USB drive named
   **NICENANO** appears.
2. Copy the `.uf2` file onto it. The board flashes itself, reboots, and the drive disappears.

If no drive appears, double-tap faster or try another cable. From a terminal you can also
open the Rook's serial port at **1200 baud** and close it, which triggers the same
bootloader. That's how it was flashed from this machine.

**Companion setup:** in the MeshCore app, connect over **Bluetooth** to `MeshCore-…`. The
pairing PIN shows on the Rook's screen (`123456` if none is shown). Set the radio to
**USA/Canada: 910.525 MHz, 62.5 kHz, SF7, CR5**.

---

## After flashing

- **Check it:** open the serial console (`pio device monitor -b 115200`, or any terminal
  at 115200) and run `ver`, `status` and `radio`.
- **A blank board** needs the one-time setup from the main [README](../README.md#setup-once-with-all-nodes-on-your-desk):
  role, name, node_id, position, Wi-Fi, Bayou keys, card exchange, targets. The other
  nodes' cards are in [`../cards/`](../cards).
- **Rebuild this folder** after firmware changes with `tools/build_firmware_bundle.sh`.
  It rebuilds all images, refreshes `VERSION.txt` and `SHA256SUMS`, and re-copies the
  Rook companion from `~/gitwork/rook` (override with `ROOK_REPO=...`).
