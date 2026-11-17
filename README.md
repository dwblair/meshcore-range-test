# meshcore-range-test

Find out whether two households (Waltham and Lincoln) can reach each other over
MeshCore — through existing public repeaters at first, through your own repeaters
later — and record **every attempt, its result, and the path it took** on a
Bayou feed with a Leaflet map.

Built on the same pattern as
[MeshCore-simple-sensor](https://github.com/edgecollective/MeshCore-simple-sensor):
direct messages (not channel broadcasts) with ACKs, receivers on Wi-Fi posting to
Bayou. This repo is a MeshCore tree (v1.14.1, copied from MeshCore-simple-sensor)
with one added example, `examples/range_test`. Upstream MeshCore's README is in
[`docs/MESHCORE_README.md`](docs/MESHCORE_README.md).

```
  Waltham house                                              Lincoln house
 ┌──────────────┐   routed DM every 10 min (both ways)     ┌──────────────┐
 │ receiver     │ ───► public repeaters ? ► your relay ? ─►│ receiver     │
 │ Heltec + WiFi│ ◄─────────────────── ACK ◄────────────── │ Heltec + WiFi│
 └──────┬───────┘                                          └──────┬───────┘
        │ posts: attempt ok / no ACK, path, SNR, repeaters heard  │
        └──────────────────────────► Bayou ◄──────────────────────┘
                                       ▲      map: /data/<key>/range/
                                       │
                          later: tools/range_log_upload.py
                                       │
                         ┌─────────────┴──────────┐
                         │ sender (Heltec V4+GPS) │ zero-hop pings to each house
                         │ roams, logs to flash   │ every 30 s, logs position +
                         └────────────────────────┘ result of every attempt
```

## One firmware, two roles

Every node runs `examples/range_test`; `set role` picks the behaviour.

| | **receiver** (one per household) | **sender** (roaming, GPS) |
|---|---|---|
| Hardware | Heltec V4 or V3 (Wi-Fi) | Heltec V4 + GPS module |
| Sends to its targets | every 10 min, **routed** (flood → learned path) | every 30 s, **direct** (zero-hop only) |
| Results go to | Bayou, live over Wi-Fi | flash log → upload later |
| Also posts | `rx` rows for everything it receives; hourly status; public repeaters it hears | — |
| Repeats others' packets | on | off (toggle on from the OLED when parked) |
| OLED | Wi-Fi/Bayou status, last message received | GPS, per-target OK / NO ACK + SNR, next send |

**routed** = a normal MeshCore DM. With no known path it floods (any public repeater
can carry it); the receiving house returns the path with its ACK, and later attempts
go direct along that path. A missed ACK on a direct attempt clears the path so the
next attempt floods again. Floods are limited to one per target per 5 minutes.

**direct** = zero-hop: only a direct radio link between the two nodes counts, and
nobody repeats it. The receiving house ACKs zero-hop too. This is the right test
for "would a repeater *here* hear both houses?", and it puts no load on the public mesh.

A Rook (`Rook_range_test`) can run the same firmware as a fixed relay or pinger
without Wi-Fi (its messages show up on Bayou via the houses that receive them), or
just run stock MeshCore repeater firmware.

## Build and flash

```bash
pio run -e heltec_v4_range_test -t upload      # Heltec V4 (GPS supported)
pio run -e Heltec_v3_range_test -t upload      # Heltec V3 (no GPS: set position by hand)
pio run -e Rook_range_test                     # Rook: copy .pio/build/Rook_range_test/firmware.uf2
pio device monitor -b 115200                   # serial console (type 'help')
```

Radio settings are the US public MeshCore preset (910.525 MHz, BW 62.5, SF7, CR5),
so existing community repeaters can carry the traffic. A new node generates its
identity on first boot.

**Heltec V4 flashing quirks:**
- **After flashing, tap RST.** With esptool 4.5 the board stays in the bootloader
  after an upload over its built-in USB.
- **If the upload fails** with `OSError: [Errno 71] Protocol error`, hold PRG, tap
  RST, release PRG, then upload again. A different cable or port also helps.

Settings, contacts and targets live in flash and survive reflashing.

**Heltec V4.3 boards** use a different RF front end (KCT8103L instead of the
GC1109). This tree includes upstream MeshCore's auto-detection
(`variants/heltec_v4/LoRaFEMControl.*`), and `status` prints which board was found.
Without it, a V4.3 receives about 50 dB below normal.

## Setup (once, with all nodes on your desk)

**1. Create a Bayou feed** at <https://bayou.pvos.org> and note its public and
private keys. All nodes use the same feed.

**2. Configure each node** over the serial console:

```text
# Waltham household                      # Lincoln household
set role receiver                        set role receiver
set name Waltham                         set name Lincoln
set node_id 1                            set node_id 2
set lat 42.37xxxx                        set lat 42.42xxxx
set lon -71.23xxxx                       set lon -71.30xxxx
set wifi_ssid <house wifi>               set wifi_ssid <house wifi>
set wifi_password <password>             set wifi_password <password>
set bayou_public_key <pub>               set bayou_public_key <pub>
set bayou_private_key <priv>             set bayou_private_key <priv>
reboot                                   reboot

# Roaming sender (Heltec V4 + GPS)
set role sender
set name Rover
set node_id 3
```

Set the household's own Wi-Fi now if you can. Otherwise use your home network for
the desk test and change it on site. The households need a fixed position (`set
lat/lon`) for the map. A V4 indoors may not get a GPS fix.

**3. Exchange contact cards.** Each node must have the others' cards: you can't
decrypt a DM from someone you don't know. On each node run `card`, copy the
`meshcore://…` line, and `import` it on the other nodes. Imported contacts are
favourited, so they are never evicted when the public mesh fills the contact list.
Check with `list`.

**4. Set targets.**

```text
Waltham:  target add Lincoln
Lincoln:  target add Waltham
Rover:    target add Waltham
          target add Lincoln
```

**5. Desk test.** On Waltham, `send`. Lincoln's console should print an
`EVENT k=ping … res=rx` line and Waltham's an `EVENT … res=ok`. Within a few
seconds both appear on Bayou, and `status` shows `bayou: ok`. At this distance the
path is direct (0 hops), which is expected.

## Near-term test: can the houses reach each other at all?

Install each household node as high as you can (an upstairs window facing the other
town beats a basement). Power it up and leave it. Every 10 minutes each house sends
the other a routed DM, and both post the outcome. The map's **Household link** card
shows, for each direction:

- the attempts, how many were two-way, how many one-way (heard but the ACK didn't
  make it back), and how many failed;
- the last working route, with repeater names (`Waltham → HillRpt (a3) → … → Lincoln`);
- SNR at each end;
- a timeline with one tick per attempt.

Expect "no connection yet" at first. The **Repeaters heard** panel shows which
public repeaters each house can hear, directly or via hops. A repeater heard by
**both** houses is a likely bridge. If a house hears no repeaters at all, work on
its antenna and placement first.

Change the interval with `set interval <secs>`; please keep it at 10 min or more.
While there is no path, every attempt is a flood across the public mesh.

## Later: finding spots for your own repeaters

Carry the Rover around. Every 30 s it sends a zero-hop ping to each house and logs
the result with its GPS position. The OLED shows each house as `OK -3dB 12s` or
`NO ACK`. A spot where **both** houses answer is a candidate relay site. The
households also post every Rover ping they hear, live, so you'll see those spots on
the map even before uploading.

**Bridging test:** park the Rover at a promising spot and switch **Repeat: ON**
(OLED page 3, long press). If the households start reaching each other, their path
will contain the Rover's hash: the first 2 hex digits of its key, shown in `status`.

Back home, upload the Rover's log, failures included:

```bash
pip install pyserial
python3 tools/range_log_upload.py --port /dev/ttyACM0 --public-key <pub> --private-key <priv>
# then on the Rover's console:  log clear
```

The script saves a local copy (`rangelog_<time>.txt`) before posting. Re-post a saved
copy with `--file`, or preview it with `--dry-run`.

## The map

`https://bayou.pvos.org/data/<public key>/range/`. It needs the range page added to
Bayou (`routes/data.js`, `controllers/dataControllers.js`, `views/range.pug`,
`public/js/range.js`), which is currently **uncommitted in `~/gitwork/bayou`** and
not yet deployed. To try it locally with simulated data:

```bash
cd ~/gitwork/bayou && ./run_local.sh            # http://localhost:5000
python3 examples/range_sim.py                     # prints a /range/ URL
```

## Serial commands

| Command | |
|---|---|
| `status` | role, position, GPS/clock, targets with last result, Wi-Fi/Bayou, log |
| `card` / `import <meshcore://…>` | share / add contacts (imports are favourited) |
| `list {n}`, `favourite <name>`, `unfavourite <name>`, `purge` | contacts |
| `target`, `target add <name>`, `target remove <name>`, `target clear` | up to 4 targets |
| `send` | run one send cycle now (also: long press on the OLED's main page) |
| `advert` / `advert flood` | zero-hop / flood advert (a flood advert also goes out at boot and every 12 h) |
| `log`, `log dump`, `log clear` | flash log size / print / erase |
| `set role sender\|receiver` | apply the role defaults in the table above |
| `set name`, `set node_id` | identity on the map (`node_id` 1–65535) |
| `set interval <s>` | send interval, `0` = only on `send` |
| `set mode direct\|routed` | zero-hop only, or normal routed DM |
| `set repeat on\|off` | forward other nodes' packets |
| `set log on\|off`, `set display on\|auto` | flash logging; OLED always on or auto-off |
| `set lat`, `set lon`, `set alt`, `set pos clear` | fixed position (used whenever there's no GPS fix) |
| `set wifi_ssid`, `set wifi_password` | Wi-Fi (reboot to apply) |
| `set bayou_public_key`, `set bayou_private_key`, `set status_interval <s>` | Bayou |
| `reboot`, `ver`, `help` | |

## Data format (Bayou rows)

Each row's `log` is a space-separated `key=value` line, and the same line is what
goes into the flash log. Numeric parts are also copied into Bayou columns so the
standard feed pages can chart them.

| `k=` | Posted by | Key fields | Bayou columns |
|---|---|---|---|
| `ping` | the sending node (`res=ok` / `to`) and the receiving node (`res=rx`) | `src sid sk dst did dk seq m res fix hops path snr_f rssi_f snr_r rssi_r rtt b` | `node_id`=sid, `gps_*`=sender position, `aux_1`=snr_f, `rssi`=rssi_f, `aux_2`=snr_r, `aux_3`=hops, `battery_volts` |
| `adv` | households, for repeaters/rooms they hear | `rep rid rk name key type hops path snr rssi` | `node_id`=rid, `gps_*`=repeater position if advertised, `aux_1`, `rssi`, `aux_3` |
| `status` | households, hourly | `name id key role fix up_s nc wifi fw b` | `node_id`, `gps_*`, `battery_volts` |

- `t` is unix time from the node's clock: GPS on the sender, NTP on the households.
  `0` means the clock isn't set yet.
- `m` is the route: `zh` zero-hop, `fl` flood, `dr` direct along a known path.
- `path` lists repeater hashes from sender to receiver (`a3.7f`). A hash is the first
  byte of a repeater's key, so the map resolves it against `adv` rows.
- `seq` never repeats for a given sender, even across reboots, so (`sk`, `seq`, `dk`)
  identifies one attempt.
- Rows uploaded from a flash log carry `up=1`.
- People's (chat) nodes are never posted. Only repeaters and room servers are.

Full details are in `~/gitwork/bayou/docs/API.md` §4. The C++ row builder
(`buildJson` in `main.cpp`) and `tools/range_log_upload.py` must stay in sync.

## Status and caveats

- Builds for Heltec V4, Heltec V3 and Rook. **Not yet run on hardware.** Check the
  desk test before deploying.
- **GPS:** detected at boot if the module is sending NMEA within ~1 s; `status`
  shows sats and fix.
- **Posting delays:** a Bayou post (HTTPS) blocks the node for about a second.
  Posts are held back while an ACK may be in flight and while a node is waiting for
  its own ACK.
- **Routed timeouts** are long (30 s for floods), so slow multi-hop ACKs aren't
  miscounted as failures. A one-way result means the receiving house heard the
  message but no ACK made it back.
- **Contacts:** the contact list holds 200 contacts on ESP32 and 64 on the Rook.
  When it's full, the oldest non-favourite contact is evicted. Targets and imported
  cards are favourites.
