#!/usr/bin/env python3
"""
range_log_upload.py — copy a range_test node's flash log to a Bayou feed.

The roaming sender logs every attempt (ok / no ACK, position, ACK SNR) to flash
because it has no internet in the field. Back home, plug it in over USB and run:

    python3 tools/range_log_upload.py --port /dev/ttyACM0 \
        --public-key <feed public key> --private-key <feed private key>

This sends `log dump` to the node, saves the dump to a local file
(rangelog_<timestamp>.txt, so you always keep a copy), and POSTs each line to
Bayou as one row tagged `up=1`. Then, on the node's serial console, run
`log clear` once you're happy (or pass --clear-after).

Lines the feed already has are skipped (rows the node posted live over Wi-Fi, or an
earlier upload of the same log); --all posts everything. If the feed was cleared
while the node's log still held older lines, --since <UTC time> leaves those out.

Re-upload a saved dump (no device needed):

    python3 tools/range_log_upload.py --file rangelog_20261117-1530.txt --public-key ... --private-key ...

Keys can also come from the environment: BAYOU_PUBLIC_KEY / BAYOU_PRIVATE_KEY.
Requires: pyserial (only for --port).  `pip install pyserial`

The line -> Bayou-row mapping here must match buildJson() in
examples/range_test/main.cpp.
"""

import argparse
import csv
import datetime
import io
import json
import os
import sys
import time
import urllib.error
import urllib.request

BEGIN = "--- range log begin"
END = "--- range log end ---"
POS_KEYS = ("lat", "lon", "alt")
TIME_VALID_MIN = 1600000000   # a t= below this is an unset device clock


def parse_line(line):
    """'k=ping v=1 t=...' -> ordered list of (key, value)."""
    pairs = []
    for tok in line.strip().split(" "):
        if "=" in tok:
            k, v = tok.split("=", 1)
            pairs.append((k, v))
    return pairs


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def line_to_row(line, private_key, mark_uploaded=True):
    pairs = parse_line(line)
    kv = dict(pairs)
    kind = kv.get("k")
    if kind is None:
        return None

    row = {"private_key": private_key}
    id_key = {"ping": "sid", "adv": "rid", "nbr": "rid"}.get(kind, "id")

    def num(key, col):
        v = kv.get(key)
        if v is not None and is_number(v):
            row[col] = float(v) if "." in v else int(v)

    num(id_key, "node_id")
    num("lat", "gps_lat")
    num("lon", "gps_lon")
    num("alt", "gps_alt")
    num("b", "battery_volts")
    if kind == "ping":
        num("snr_f", "aux_1")
        num("rssi_f", "rssi")
        num("snr_r", "aux_2")
    elif kind in ("adv", "nbr"):
        num("snr", "aux_1")
        num("rssi", "rssi")
        if kind == "nbr":
            num("n", "aux_2")
    hops = kv.get("hops")
    if hops is not None and is_number(hops) and not hops.startswith("-"):
        row["aux_3"] = int(hops)

    # log = the line without position keys (those are in the gps columns), plus up=1
    out = []
    for k, v in pairs:
        if k in POS_KEYS or k == "up":
            continue
        out.append(f"{k}={v}")
        if k == "v" and mark_uploaded:
            out.append("up=1")
    row["log"] = " ".join(out)[:250]
    return row


# USB-serial bridge chips (CP210x on the Heltec V3, CH34x, FTDI): their DTR/RTS lines drive the
# ESP32's EN/IO0, so asserting them resets the board.
BRIDGE_VIDS = {0x10C4, 0x1A86, 0x0403}


def open_console(port):
    """Open a node's serial console without resetting it.

    A USB-serial bridge (V3) resets the board if DTR/RTS move, so both stay low. A board with
    native USB (V4, nRF52) has no reset wiring, but its USB CDC only sends output while DTR is
    asserted: with DTR low, `log dump` is accepted and nothing comes back.
    """
    try:
        import serial  # pyserial
        from serial.tools import list_ports
    except ImportError:
        sys.exit("pyserial is required for --port:  pip install pyserial")

    info = next((p for p in list_ports.comports() if p.device == os.path.realpath(port) or p.device == port), None)
    if info is not None and info.vid is not None:
        bridge = info.vid in BRIDGE_VIDS
    else:   # no USB info: guess from the name (Linux ttyUSB*, macOS *usbserial*)
        bridge = "ttyUSB" in port or "usbserial" in port.lower()

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 1
    if bridge:
        ser.dtr = False
        ser.rts = False
    else:
        ser.dtr = True
    ser.open()
    return ser


def dump_from_device(port, timeout):
    ser = open_console(port)
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.write(b"\r\nlog dump\r\n")

    lines, inside, deadline = [], False, time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        text = raw.decode("utf-8", errors="replace").strip()
        if text.startswith(BEGIN):
            inside = True
            print(text)
            continue
        if text.startswith(END):
            ser.close()
            return lines, True
        if inside and text.startswith("k="):
            lines.append(text)
            deadline = time.time() + timeout   # keep going while data flows
    ser.close()
    return lines, False


def send_clear(port):
    ser = open_console(port)
    time.sleep(0.3)
    ser.write(b"\r\nlog clear\r\n")
    time.sleep(1.0)
    print(ser.read(ser.in_waiting or 1).decode("utf-8", errors="replace").strip())
    ser.close()


def dup_key(log):
    """A row's identity for duplicate checks. Bayou keeps `log` without the position keys (the
    firmware cuts it at ' lat='), uploads add 'up=1', and long lines may be truncated."""
    return log.split(" lat=")[0].replace(" up=1", "").strip()[:200]


def fetch_existing(base_url, public_key):
    """(dup keys of every row already on the feed, the feed's oldest timestamp or None)."""
    with urllib.request.urlopen(f"{base_url}/data/{public_key}/csv/", timeout=60) as resp:
        text = resp.read().decode("utf-8", errors="replace")
    reader = csv.DictReader(io.StringIO(text))
    if not reader.fieldnames or "log" not in reader.fieldnames:
        raise ValueError("the feed's /csv/ has no log column (wrong public key?)")
    keys, first = set(), None
    for r in reader:
        keys.add(dup_key(r.get("log") or ""))
        ts = r.get("timestamp")
        if ts and (first is None or ts < first):
            first = ts
    return keys, first


def line_times(lines):
    """Unix time of each line. A t=0 line (clock not set yet, just after boot) takes the time of
    the next dated line, which comes from the same boot, else of the previous one."""
    ts = []
    for line in lines:
        t = dict(parse_line(line)).get("t", "0")
        ts.append(int(t) if t.isdigit() and int(t) >= TIME_VALID_MIN else 0)
    out = []
    for i, t in enumerate(ts):
        if not t:
            t = next((x for x in ts[i + 1:] if x), 0) or next((x for x in reversed(ts[:i]) if x), 0)
        out.append(t)
    return out


def parse_since(s):
    """'2026-11-19', '2026-11-19T01:35' (UTC) or unix seconds -> unix seconds."""
    if s.isdigit():
        return int(s)
    d = datetime.datetime.fromisoformat(s.replace("Z", "+00:00"))
    if d.tzinfo is None:
        d = d.replace(tzinfo=datetime.timezone.utc)
    return int(d.timestamp())


def post_row(url, row):
    data = json.dumps(row).encode()
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            return resp.status, resp.read().decode(errors="replace").strip()
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode(errors="replace").strip()
    except urllib.error.URLError as e:
        return -1, str(e.reason)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the node, e.g. /dev/ttyACM0 or /dev/ttyUSB0")
    src.add_argument("--file", help="previously saved dump file")
    ap.add_argument("--public-key", default=os.environ.get("BAYOU_PUBLIC_KEY"))
    ap.add_argument("--private-key", default=os.environ.get("BAYOU_PRIVATE_KEY"))
    ap.add_argument("--url", default="https://bayou.pvos.org", help="Bayou base URL")
    ap.add_argument("--dry-run", action="store_true", help="print the rows instead of posting")
    ap.add_argument("--timeout", type=float, default=10, help="serial read timeout (s)")
    ap.add_argument("--clear-after", action="store_true", help="send 'log clear' to the node if every row posted OK")
    ap.add_argument("--all", action="store_true",
                    help="post every line, even ones the feed already has (default: skip those)")
    ap.add_argument("--since", help="skip lines dated before this UTC time (e.g. 2026-11-19T01:35), "
                    "for a log that predates the last time the feed was cleared")
    args = ap.parse_args()

    if not args.dry_run and not (args.public_key and args.private_key):
        sys.exit("need --public-key and --private-key (or BAYOU_PUBLIC_KEY / BAYOU_PRIVATE_KEY)")

    if args.port:
        lines, complete = dump_from_device(args.port, args.timeout)
        if not complete:
            print(f"WARNING: did not see '{END}' — dump may be incomplete ({len(lines)} lines read)")
        fname = time.strftime("rangelog_%Y%m%d-%H%M%S.txt")
        with open(fname, "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        print(f"saved {len(lines)} lines to {fname}")
    else:
        with open(args.file) as f:
            lines = [l.strip() for l in f if l.strip().startswith("k=")]
        complete = True

    base = args.url.rstrip("/")
    times = dict(zip(range(len(lines)), line_times(lines)))   # dated from the whole log, before any filtering
    idx = list(range(len(lines)))
    if args.since and idx:
        since = parse_since(args.since)
        kept = [i for i in idx if times[i] >= since]
        print(f"skipping {len(idx) - len(kept)} lines dated before {args.since}")
        idx = kept

    if idx and not args.all and args.public_key:
        try:
            have, first = fetch_existing(base, args.public_key)
        except (OSError, ValueError) as e:
            sys.exit(f"couldn't read the feed to skip rows it already has: {e}\n"
                     "check --public-key, or pass --all to post everything")
        kept = []
        for i in idx:
            row = line_to_row(lines[i], "")
            if row is None or dup_key(row["log"]) not in have:
                kept.append(i)
        print(f"skipping {len(idx) - len(kept)} lines the feed already has")
        idx = kept
        if first and not args.since:
            first_t = parse_since(first)
            older = sum(1 for i in idx if times[i] and times[i] < first_t)
            if older:
                print(f"note: {older} of the remaining lines are older than the feed's first row ({first}). "
                      f"If the feed was cleared after them, re-run with --since {first[:16]} to leave them out.")
    lines = [lines[i] for i in idx]

    if not lines:
        print("nothing to upload")
        return

    url = f"{base}/data/{args.public_key}"
    ok = fail = 0
    for i, line in enumerate(lines, 1):
        row = line_to_row(line, args.private_key or "PRIVATE_KEY")
        if row is None:
            continue
        if args.dry_run:
            print(json.dumps(row))
            continue
        code, body = post_row(url, row)
        if 200 <= code < 300:
            ok += 1
        else:
            fail += 1
            print(f"  line {i}: HTTP {code} {body}")
            if code in (400, 401, 403, 404) and fail >= 3 and ok == 0:
                sys.exit("repeated rejections — check the feed keys")
        if i % 50 == 0:
            print(f"  {i}/{len(lines)} posted")
        time.sleep(0.05)

    if not args.dry_run:
        print(f"done: {ok} posted, {fail} failed")
        print(f"map: {args.url.rstrip('/')}/data/{args.public_key}/range/")
        if args.clear_after and args.port and complete and fail == 0:
            send_clear(args.port)
        elif args.port:
            print("(log left on the device; run 'log clear' on its console when you're happy)")


if __name__ == "__main__":
    main()
