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
                         │ sender (Heltec V4+GPS) │ every 5 min (or button): a direct
                         │ roams, logs to flash   │ and a routed ping to each house,
                         └────────────────────────┘ + repeaters heard, with position
```

## One firmware, two roles

Every node runs `examples/range_test`; `set role` picks the behaviour.

| | **receiver** (one per household) | **sender** (roaming, GPS) |
|---|---|---|
| Hardware | Heltec V4 or V3 (Wi-Fi) | Heltec V4 + GPS module |
| Sends to its targets | every 10 min, **routed** (flood → learned path) | every 5 min or on a long press of PRG: **both**, a direct ping and then a routed (flooded) ping to each house |
| Results go to | Bayou, live over Wi-Fi | flash log → upload later (live too while it has Wi-Fi) |
| Also posts | `rx` rows for everything it receives; hourly status; repeaters it hears (adverts + direct neighbours) | each round: the repeaters it heard directly, tagged with its GPS position |
| Repeats others' packets | on | off (toggle on from the OLED when parked) |
| OLED | Wi-Fi/Bayou status, last message received | battery, GPS, one line per house with both results, next round |

**routed** = a normal MeshCore DM. With no known path it floods (any public repeater
can carry it); the receiving house returns the path with its ACK, and later attempts
go direct along that path. A missed ACK on a direct attempt clears the path so the
next attempt floods again.

**direct** = zero-hop: only a direct radio link between the two nodes counts, and
nobody repeats it. The receiving house ACKs zero-hop too. This is the right test
for "would a repeater *here* hear both houses?", and it puts no load on the public mesh.

**both** (the sender default) = each round sends the direct ping to every house, then a
routed ping that **always floods**. That's route discovery: the receiving house records
which repeaters carried it *from where the Rover is now*, rather than replaying a route
learned somewhere else. Each kind of ping has its own `seq`, so the map never merges
them. Floods are limited to one per house per minute, so pressing the button again
right away skips the routed ping (`w` on the screen).

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

**Heltec V4 flashing:** esptool's automatic reset over the V4's built-in USB is
unreliable with this firmware (`OSError: [Errno 71] Protocol error`). Use the buttons
instead:
1. Hold **PRG**, tap **RST**, release **PRG**. The board is now in its bootloader.
2. Run the upload.
3. Tap **RST** to start the new firmware.

The Heltec V3 has a separate USB-serial chip, so it flashes and restarts by itself.
Its boot log shows a burst of harmless `i2cRead returned Error -1` lines while it
probes for optional sensors.

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

**Pairing a new Rover with houses that are already deployed.** The Rover needs
the houses' cards. Import `cards/Waltham.txt` and `cards/Lincoln.txt` on its console,
then `target add` each house.

Each house also needs the Rover's key, or it can't decrypt the Rover's pings. Give it
the key in one of these ways:
- **Over USB:** run `import` with the Rover's `card` on the house's console.
- **Over the air:** stand near the house and long-press **Send advert** on the Rover's
  OLED (page 4). The house adds the Rover automatically, and favourites it on the
  first ping.

**Testing from the MeshCore phone app.** Any companion radio works as a pocket tester.

*Setup, once per companion:*
- Scan `cards/Waltham-qr.png` and `cards/Lincoln-qr.png` in the app to add the houses.
  The QR codes use the app's `meshcore://contact/add?name=…&public_key=…&type=1` format;
  the `.txt` cards are for node consoles (`import`).
- Send a **flood advert** from the app near each house, so the house learns your key.
  (Or, over USB on the house: `import` the companion's card, e.g. `cards/WFS2.txt`.)
- To have your position logged automatically, let the companion share it with the
  houses only. In the app's telemetry settings, set battery and location to *selected
  contacts*, and allow both for Waltham and Lincoln. With `meshcore-cli`:
  `set telemetry_mode_base 1`, `set telemetry_mode_loc 1`,
  `change_flags Waltham 7`, `change_flags Lincoln 7`.

*In the field:* send the house **any message**. Then:
- **The delivered tick** means the house heard you and its acknowledgement got back.
- **The house answers in the chat** with how it heard you, e.g.
  `Waltham got #1752: 2 hops via OwlHill>LEX-Fiske, last hop SNR -4.5 RSSI -112`.
  If you get the tick but no answer, the link worked one way only.
- **The house logs the test to Bayou** (never the text). The **Phone tests** panel on
  the map page lists it, with a marker at your position.

Position comes from the companion's GPS, which the house requests after replying. If
the companion has no fix, paste coordinates into the message, e.g.
`42.41208, -71.29789` or `lat=42.41 lon=-71.30`; those win over GPS.

To see the route, **reset the path** to the house in the app before sending. After a
success the app remembers the path and sends direct, so the house can't see the route
(the answer then says "along your saved path"). Each house answers a given phone at
most once every 10 s. Only successes are recorded: the phone can't post its failures.

Carry the Rover around (a USB power bank works). Every 5 minutes, or whenever you
**long-press PRG** on its main screen, it runs a round: a direct ping, then a routed
ping, to each house. It logs every result with its GPS position, plus the repeaters it
heard directly since the last round. The households post every Rover ping they hear,
live, so those spots appear on the map before you upload anything.

The Rover's screen:

```
Rover #487        4.08V     name, round number, battery (≈4.2 V full, ≈3.4 V empty)
GPS fix 9sat                no fix yet: rounds carry no position
Waltha D+12 R2h 40s         one line per house, then time since its last result
Lincol Dx  R3h 40s
next 245s log 12K
```

| On a house's line | Meaning |
|---|---|
| `D+12` | **direct** ping ACKed, at +12 dB SNR |
| `R2h` | **routed** ping ACKed, over 2 repeater hops (`R0h` = direct radio link) |
| `x` | no ACK |
| `..` | waiting for the ACK |
| `w` | routed ping skipped (one flood per house per minute) |
| `-` | not tried yet |

So `Lincol Dx R3h` means "no direct link from here, but it got through via 3 repeaters".
A spot where the **direct** ping reaches **both** houses is a candidate relay site.
Short presses of PRG step through the screens: main → Send now → Repeat → Send advert.

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

`https://bayou.pvos.org/data/<public key>/range/`. The page lives in Bayou's
`range-test-page` branch (`routes/data.js`, `controllers/dataControllers.js`,
`views/range.pug`, `public/js/range.js`), which is deployed on bayou.pvos.org. To try it
locally with simulated data:

```bash
cd ~/gitwork/bayou && ./run_local.sh            # http://localhost:5000
python3 examples/range_sim.py                     # prints a /range/ URL
```

### Clearing the feed

To start over (after desk testing, or before a real deployment), delete every row in
the feed:

```bash
curl https://bayou.pvos.org/manage/clear/<public key>/<private key>
# → "feed cleared of historical data!"   (or "keys don't match!")
```

This deletes **all** of the feed's measurements: pings, status, adverts and
neighbour rows. It can't be undone. The feed and its keys stay, so the nodes keep
posting to it with no changes. Each household re-posts its status within one status
interval, or at once after a reboot; until then the page shows no households.
Because the clear is a plain GET, don't paste the URL anywhere that might preview it.
To keep a copy first, save `https://bayou.pvos.org/data/<public key>/csv/`, which
returns every row (`/json/` stops at the newest 2000).

## Serial commands

| Command | |
|---|---|
| `status` | role, position, GPS/clock, targets with last result, Wi-Fi/Bayou, log |
| `card` / `import <meshcore://…>` | share / add contacts (imports are favourited) |
| `list {n}`, `favourite <name>`, `unfavourite <name>`, `purge` | contacts |
| `target`, `target add <name>`, `target remove <name>`, `target clear` | up to 4 targets |
| `send` | run one send cycle now (also: long press on the OLED's main page) |
| `nbr` | repeaters this node hears **directly** (from all flood traffic), with packet count, SNR and RSSI. Use it to find a good spot for a household node |
| `advert` / `advert flood` | zero-hop / flood advert (a flood advert also goes out at boot and every 12 h) |
| `log`, `log dump`, `log clear` | flash log size / print / erase |
| `set role sender\|receiver` | apply the role defaults in the table above |
| `set name`, `set node_id` | identity on the map (`node_id` 1–65535) |
| `set interval <s>` | send interval, `0` = only on `send` |
| `set mode direct\|routed\|both` | zero-hop only / normal routed DM / both every round, with the routed ping always flooding (sender default) |
| `set repeat on\|off` | forward other nodes' packets |
| `set log on\|off`, `set display on\|auto` | flash logging; OLED always on or auto-off |
| `set lat`, `set lon`, `set alt`, `set pos clear` | fixed position (used whenever there's no GPS fix) |
| `set radio <MHz>,<kHz>,<sf>,<cr>` | radio settings, applied immediately and saved (e.g. `set radio 910.525,62.5,7,5`). **Every node you want to reach must match**; nodes on different settings can't hear each other |
| `set radio default`, `radio` | back to the built-in US preset (910.525 / 62.5 / SF7 / CR5); show the current settings |
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
| `status` | households, hourly (and any node with Wi-Fi) | `name id key role fix up_s nc wifi rf fw b` (`rf` = radio settings, e.g. `910.525,62.5,7,5`) | `node_id`, `gps_*`, `battery_volts` |
| `nbr` | households hourly (with `status`); the sender at the start of every round, with `fix` + its position | `rep rid rk hash n snr snr_min snr_max rssi span [fix lat lon alt]`: one row per node heard **directly**, aggregated over `span` seconds from all flood traffic (no message contents) | `node_id`=rid, `aux_1`=snr, `rssi`, `aux_2`=n, `gps_*` (sender) |

- `t` is unix time from the node's clock: GPS on the sender, NTP on the households.
  `0` means the clock isn't set yet.
- `m` is the route: `zh` zero-hop, `fl` flood, `dr` direct along a known path.
- `path` lists repeater hashes from sender to receiver (`a3.7f`). A hash is the first
  byte of a repeater's key, so the map resolves it against `adv` rows.
- `seq` never repeats for a given sender, even across reboots (each leg of a sender round gets its own), so (`sk`, `seq`, `dk`)
  identifies one attempt.
- Rows uploaded from a flash log carry `up=1`.
- **Chat tests** (a person messaging a house from the phone app) are `k=ping res=rx` rows
  posted by the house with `via=chat sid=0`. `seq` comes from the house's counter, `fix` is
  `gps` (the companion's GPS, requested by the house), `man` (typed into the message) or
  `none`, and `b` is the companion's battery. The message text is never posted.
- Adverts from people's (chat) nodes are never posted; only repeaters and room servers are.

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
