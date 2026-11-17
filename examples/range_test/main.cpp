/*
 * range_test — MeshCore link / range testing firmware.
 *
 * One image, two roles (selected at runtime with `set role ...`):
 *
 *   receiver  Fixed station (e.g. one per house). Listens for [RANGE] direct
 *             messages, ACKs them, and posts each reception to Bayou over Wi-Fi
 *             (sender position, SNR/RSSI, hops, path). Also posts an hourly
 *             status row (its own position) and the positions of repeaters it
 *             hears adverts from, so the map can draw paths.
 *
 *   sender    Roaming node with GPS. Every N seconds sends a [RANGE] DM with its
 *             position to each target (one at a time), waits for the ACK, and
 *             appends every attempt (ok / no ACK, ACK SNR, position) to a log in
 *             flash. `log dump` + tools/range_log_upload.py put that log on Bayou.
 *
 * Send modes:
 *   direct  (default for sender) zero-hop: only a direct radio link counts; the
 *           message is never repeated, and the receiver ACKs zero-hop too. This
 *           is what you want while driving around looking for relay spots, and
 *           it puts no load on the public mesh.
 *   routed  normal MeshCore DM: flood first, learn a path, then direct. Use it
 *           to test reachability through repeaters (rate-limited floods).
 *
 * Bayou row format: see README.md ("Data format").
 */

#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include <stdarg.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <helpers/AdvertDataHelpers.h>
#include <RTClib.h>
#include <target.h>

#ifdef ESP32
  #include <WiFi.h>
  #include <HTTPClient.h>
  #include <time.h>
  #define HAS_WIFI 1
#else
  #define HAS_WIFI 0
#endif

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#define FIRMWARE_VER_TEXT   "range_test v1.0 (Nov 2026)"
#define FW_TAG              "rt1.0"

#ifndef LORA_FREQ
  #define LORA_FREQ   910.525
#endif
#ifndef LORA_BW
  #define LORA_BW     62.5
#endif
#ifndef LORA_SF
  #define LORA_SF     7
#endif
#ifndef LORA_CR
  #define LORA_CR     5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif
#ifndef MAX_CONTACTS
  #define MAX_CONTACTS   64
#endif
#ifndef ADVERT_NAME
  #define ADVERT_NAME  "RangeNode"
#endif
#ifndef BAYOU_BASE_URL
  #define BAYOU_BASE_URL "https://bayou.pvos.org/data/"
#endif
#ifndef LOG_MAX_BYTES
  #ifdef ESP32
    #define LOG_MAX_BYTES  1000000UL
  #else
    #define LOG_MAX_BYTES  16000UL
  #endif
#endif

#define MAX_TARGETS               4
#define TARGET_PREFIX_LEN         6
#define SENDER_DEFAULT_INTERVAL   300          // seconds (roaming sender: a direct + a routed ping per round)
#define MAX_ROUND_NBR_ROWS        8            // sender: neighbours logged per round
#define RECEIVER_DEFAULT_INTERVAL 600          // seconds (household -> household, routed; only if targets set)
#define FLOOD_ACK_TIMEOUT_MIN_MS  30000UL      // multi-hop public routes are slow: be generous
#define DIRECT_ACK_TIMEOUT_MIN_MS 15000UL
#define ROUTED_MIN_FLOOD_GAP_MS   (60UL*1000UL)      // at most one flood per target per minute (button presses)
#define STATUS_DEFAULT_INTERVAL   3600         // seconds
#define ADVERT_FLOOD_INTERVAL_MS  (12UL*3600UL*1000UL)
#define ADV_POST_MIN_INTERVAL_MS  (6UL*3600UL*1000UL)
#define POST_QUEUE_SIZE           40           // hourly burst: status + up to MAX_NBR_ROWS nbr rows
#define MAX_NBR_ROWS              16           // busiest neighbours posted per summary
#define LINE_MAX                  300
#define SEQ_BLOCK                 50
#define TIME_VALID_MIN            1735689600UL // 2025-01-01: earlier = clock never synced
#define CONTACTS_SAVE_DELAY_MS    30000UL
#define FLOOD_FORWARD_MAX_HOPS    16
#define MAX_NBRS                  32
#define MSG_PREFIX                "[RANGE]"

#include <helpers/BaseChatMesh.h>

#define SEND_TIMEOUT_BASE_MILLIS          500
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250

#define DISPLAY_REFRESH_MS    500
#define DISPLAY_AUTO_OFF_MS   60000
#define ALERT_DURATION_MS     1500

/* ---------------------------------- HELPERS ------------------------------------------- */

enum { ROLE_RECEIVER = 0, ROLE_SENDER = 1 };
enum { MODE_DIRECT = 0, MODE_ROUTED = 1, MODE_BOTH = 2 };
enum { POS_NONE = 0, POS_MANUAL = 1, POS_GPS = 2, POS_LAST = 3 };
#define SENDER_LAST_FIX_MAX_MS  60000UL   // a moving sender may reuse its last fix only briefly
enum { ROUTE_ZH = 0, ROUTE_FL = 1, ROUTE_DR = 2 };
enum { RES_NONE = 0, RES_PENDING, RES_OK, RES_TIMEOUT, RES_FAIL, RES_NOCONTACT, RES_SKIPPED };

static const char* roleName(uint8_t r)  { return r == ROLE_SENDER ? "sender" : "receiver"; }
static const char* modeName(uint8_t m)  { return m == MODE_BOTH ? "both" : m == MODE_ROUTED ? "routed" : "direct"; }
static const char* fixName(uint8_t s) {
  return s == POS_GPS ? "gps" : s == POS_LAST ? "last" : s == POS_MANUAL ? "man" : "none";
}
static const char* routeName(uint8_t r) { return r == ROUTE_FL ? "fl" : (r == ROUTE_DR ? "dr" : "zh"); }

static void toHexLower(char* dst, const uint8_t* src, int n) {
  static const char* H = "0123456789abcdef";
  for (int i = 0; i < n; i++) {
    dst[i*2]   = H[src[i] >> 4];
    dst[i*2+1] = H[src[i] & 0x0F];
  }
  dst[n*2] = 0;
}

// 8-hex public key prefix (first 2 hex chars = the 1-byte path hash)
static void keyHex(char* dst, const uint8_t* pub_key) { toHexLower(dst, pub_key, 4); }

// Names go into space-separated key=value lines and JSON: keep [A-Za-z0-9_.-], max 16 chars.
static void sanitizeName(char* dst, size_t sz, const char* src) {
  size_t i = 0;
  for (; src[i] && i < sz - 1 && i < 16; i++) {
    char c = src[i];
    bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '.' || c == '-';
    dst[i] = ok ? c : '_';
  }
  dst[i] = 0;
  if (i == 0) StrHelper::strncpy(dst, "unnamed", sz);
}

// Path bytes -> "a3.7f" (each hop = hash_size bytes). enc_len uses the MeshCore encoding:
// low 6 bits = hop count, top 2 bits = hash size - 1.
// Paths are in transmit order, so the LAST hops are the ones nearest the node that received
// the packet. If the path is longer than max_hops (or the buffer), keep those last hops: they
// are what says something about local connectivity. The full count is reported separately.
static void formatPath(char* dst, size_t sz, const uint8_t* path, uint8_t enc_len, uint8_t max_hops) {
  if (enc_len == OUT_PATH_UNKNOWN || (enc_len & 63) == 0) { StrHelper::strncpy(dst, "-", sz); return; }
  uint8_t count = enc_len & 63;
  uint8_t hsz = (enc_len >> 6) + 1;
  uint8_t fit = (uint8_t)(sz / (hsz*2 + 1));   // each hop takes hsz*2 hex chars + '.' (or the NUL)
  uint8_t n = count;
  if (n > max_hops) n = max_hops;
  if (n > fit) n = fit;
  size_t off = 0;
  for (uint8_t h = count - n; h < count; h++) {
    if (off) dst[off++] = '.';
    toHexLower(&dst[off], &path[h*hsz], hsz);
    off += hsz*2;
  }
  dst[off] = 0;
}

// Find "key=value" at a token boundary in a space-separated line.
static bool kvGet(const char* s, const char* key, char* out, size_t outsz) {
  size_t klen = strlen(key);
  const char* p = s;
  while ((p = strstr(p, key)) != NULL) {
    if ((p == s || p[-1] == ' ') && p[klen] == '=') {
      const char* v = p + klen + 1;
      size_t i = 0;
      while (v[i] && v[i] != ' ' && i < outsz - 1) { out[i] = v[i]; i++; }
      out[i] = 0;
      return true;
    }
    p += klen;
  }
  return false;
}

static bool isNumber(const char* s) {
  if (*s == '-') s++;
  bool digit = false, dot = false;
  for (; *s; s++) {
    if (*s >= '0' && *s <= '9') digit = true;
    else if (*s == '.' && !dot) dot = true;
    else return false;
  }
  return digit;
}

template <int N>
struct LineT {
  char s[N];
  int n;
  LineT() { n = 0; s[0] = 0; }
  void add(const char* fmt, ...) {
    if (n >= (int)sizeof(s) - 1) return;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(&s[n], sizeof(s) - n, fmt, ap);
    va_end(ap);
    if (w > 0) { n += w; if (n > (int)sizeof(s) - 1) n = sizeof(s) - 1; }
  }
};
typedef LineT<LINE_MAX> Line;       // event lines (<= ~250 chars in practice)
#define JSON_MAX  640
typedef LineT<JSON_MAX> JsonLine;   // Bayou POST body: event line + numeric columns + key

struct Position {
  uint8_t src;      // POS_*
  double lat, lon;
  int alt;          // meters
  int sats;
};

#define PREFS_MAGIC  0x31475452UL   // "RTG1"

struct RangePrefs {   // persisted to /range_prefs
  uint32_t magic;
  char node_name[32];
  uint16_t node_id;
  uint8_t role;
  uint8_t mode;
  uint8_t repeat;
  uint8_t log_enabled;
  uint8_t display_always_on;
  uint8_t reserved0;
  uint32_t send_interval_secs;    // 0 = never send automatically
  uint32_t status_interval_secs;
  int32_t fixed_lat_e6, fixed_lon_e6;
  int16_t fixed_alt_m;
  uint8_t has_fixed_pos;
  uint8_t reserved1;
  uint32_t seq_base;
  char wifi_ssid[48];
  char wifi_password[64];
  char bayou_public_key[32];
  char bayou_private_key[32];
};

// A round sends one or two legs to every target: a zero-hop ping ("can this spot reach the
// house directly?") and/or a routed one ("which repeaters carry me from here?").
enum { LEG_ZH = 0, LEG_RT = 1 };

struct LegStat {
  uint8_t result;          // RES_*
  float ack_snr;
  int16_t ack_rssi;
  uint32_t rtt_ms;
  int8_t hops;             // routed leg: hops of the route that worked (-1 unknown)
  unsigned long at_millis;
  uint32_t attempts, oks;
};

struct TargetStat {
  LegStat leg[2];          // [LEG_ZH], [LEG_RT]
};

/* -------------------------------------------------------------------------------------- */

class RangeMesh : public BaseChatMesh, ContactVisitor {
  FILESYSTEM* _fs;
  RangePrefs _prefs;
  char command[512];
  uint8_t tmp_buf[256];
  char hex_buf[512];
  char self_key[9];

  // targets (6-byte pub key prefixes, persisted in /targets)
  uint8_t target_prefixes[MAX_TARGETS][TARGET_PREFIX_LEN];
  uint8_t target_count;
  TargetStat tstat[MAX_TARGETS];
  unsigned long last_flood_at[MAX_TARGETS];

  // sender state machine: for each leg of the round, one target at a time, wait for its
  // ACK, then the next
  enum { SS_IDLE = 0, SS_NEXT, SS_WAIT_ACK };
  uint8_t ss_state;
  uint8_t ss_idx;
  uint8_t cycle_legs[2];
  uint8_t cycle_nlegs;
  uint8_t ss_leg;
  uint32_t seq;
  uint32_t cycle_seq;
  uint32_t cycle_t;
  Position cycle_pos;
  float cycle_batt;
  unsigned long next_cycle_at, ss_next_at, ss_deadline, ss_sent_at;
  uint32_t expected_ack;
  uint8_t ss_route;
  uint8_t ss_fwd_hops;
  char ss_fwd_path[64];
  char ss_dst_name[20];
  char ss_dst_key[9];

  // last received packet (stashed in logRxRaw/logRx, before dispatch)
  float last_rx_snr, last_rx_rssi;
  unsigned long last_rx_millis;

  // Direct neighbours: every flood packet we hear ends its path with the hash of the node
  // that just transmitted it, so each copy tells us one repeater we hear directly, and how
  // well. Aggregated per status interval and posted as k=nbr rows (no packet contents).
  struct NbrStat {
    uint8_t hash[3];
    uint8_t hsz;
    uint16_t n;
    float snr_sum, snr_min, snr_max;
    float rssi_sum;
    uint16_t rssi_n;
    unsigned long last_at;
  };
  NbrStat nbrs[MAX_NBRS];
  uint8_t nbr_count;
  uint16_t nbr_overflow;
  unsigned long nbr_window_start;

  // receiver stats (display)
  char rx_from[20];
  float rx_snr, rx_rssi;
  int rx_hops;
  char rx_mode[4];
  unsigned long rx_at;
  uint32_t rx_count;

  // last GPS fix since boot (RAM only, see getPosition)
  Position last_fix;
  unsigned long last_fix_at;
  bool has_last_fix;

  // flash log
  uint32_t log_bytes, log_lines_boot;
  bool log_full;

  // contacts persistence (lazy)
  unsigned long contacts_save_at;

  // auto-favourite on import
  uint8_t pending_fav_pub_key[PUB_KEY_SIZE];
  bool has_pending_fav;

  unsigned long next_flood_advert_at;
  unsigned long next_status_at;

  // repeater-advert throttle (only repeaters/rooms with a position get posted)
  struct AdvSeen { uint8_t key[4]; int32_t lat, lon; unsigned long at; };
  AdvSeen adv_seen[64];
  uint8_t adv_next;

#if HAS_WIFI
  char post_q[POST_QUEUE_SIZE][LINE_MAX];
  uint8_t q_head, q_count;
  unsigned long q_next_try, q_head_since;
  uint32_t post_ok, post_fail, post_dropped;
  char post_status[24];
  char wifi_status[24];
  bool ntp_started;
  unsigned long last_wifi_kick;
#endif

  /* ---------------- persistence ---------------- */

  File openRead(const char* path) {
  #if defined(RP2040_PLATFORM)
    return _fs->open(path, "r");
  #else
    return _fs->open(path);
  #endif
  }

  File openWrite(const char* path) {   // truncate
  #if defined(NRF52_PLATFORM)
    _fs->remove(path);
    return _fs->open(path, FILE_O_WRITE);
  #elif defined(RP2040_PLATFORM)
    return _fs->open(path, "w");
  #else
    return _fs->open(path, "w", true);
  #endif
  }

  File openAppend(const char* path) {
  #if defined(NRF52_PLATFORM)
    return _fs->open(path, FILE_O_WRITE);   // Adafruit LittleFS seeks to end
  #else
    return _fs->open(path, "a");
  #endif
  }

  void loadContacts() {
    if (!_fs->exists("/contacts")) return;
    File file = openRead("/contacts");
    if (!file) return;
    while (true) {
      ContactInfo c;
      uint8_t pub_key[32];
      uint8_t unused;
      uint32_t reserved;
      bool ok = (file.read(pub_key, 32) == 32);
      ok = ok && (file.read((uint8_t *) &c.name, 32) == 32);
      ok = ok && (file.read(&c.type, 1) == 1);
      ok = ok && (file.read(&c.flags, 1) == 1);
      ok = ok && (file.read(&unused, 1) == 1);
      ok = ok && (file.read((uint8_t *) &reserved, 4) == 4);
      ok = ok && (file.read((uint8_t *) &c.out_path_len, 1) == 1);
      ok = ok && (file.read((uint8_t *) &c.last_advert_timestamp, 4) == 4);
      ok = ok && (file.read(c.out_path, 64) == 64);
      if (!ok) break;
      c.gps_lat = c.gps_lon = 0;
      c.id = mesh::Identity(pub_key);
      c.lastmod = 0;
      c.sync_since = 0;
      c.shared_secret_valid = false;
      if (!addContact(c)) break;
    }
    file.close();
  }

  void saveContacts() {
    File file = openWrite("/contacts");
    if (!file) return;
    ContactsIterator iter;
    ContactInfo c;
    uint8_t unused = 0;
    uint32_t reserved = 0;
    while (iter.hasNext(this, c)) {
      bool ok = (file.write(c.id.pub_key, 32) == 32);
      ok = ok && (file.write((uint8_t *) &c.name, 32) == 32);
      ok = ok && (file.write(&c.type, 1) == 1);
      ok = ok && (file.write(&c.flags, 1) == 1);
      ok = ok && (file.write(&unused, 1) == 1);
      ok = ok && (file.write((uint8_t *) &reserved, 4) == 4);
      ok = ok && (file.write((uint8_t *) &c.out_path_len, 1) == 1);
      ok = ok && (file.write((uint8_t *) &c.last_advert_timestamp, 4) == 4);
      ok = ok && (file.write(c.out_path, 64) == 64);
      if (!ok) break;
    }
    file.close();
  }

  void markContactsDirty() {
    if (contacts_save_at == 0) contacts_save_at = millis() + CONTACTS_SAVE_DELAY_MS;
  }

  void applyRoleDefaults(uint8_t role) {
    _prefs.role = role;
    if (role == ROLE_SENDER) {
      _prefs.mode = MODE_BOTH;
      _prefs.send_interval_secs = SENDER_DEFAULT_INTERVAL;
      _prefs.repeat = 0;
      _prefs.log_enabled = 1;
      _prefs.display_always_on = 1;
    } else {
      // Households: routed DMs to the other household(s) (if any targets are set), so any
      // public repeaters in between can carry them. Both ends post results live.
      _prefs.mode = MODE_ROUTED;
      _prefs.send_interval_secs = RECEIVER_DEFAULT_INTERVAL;
      _prefs.repeat = 1;
      _prefs.log_enabled = 0;
      _prefs.display_always_on = 0;
    }
  }

  void setDefaultPrefs() {
    memset(&_prefs, 0, sizeof(_prefs));
    _prefs.magic = PREFS_MAGIC;
    StrHelper::strncpy(_prefs.node_name, ADVERT_NAME, sizeof(_prefs.node_name));
    _prefs.node_id = 1;
    _prefs.status_interval_secs = STATUS_DEFAULT_INTERVAL;
    applyRoleDefaults(ROLE_RECEIVER);
  }

  void loadPrefs() {
    setDefaultPrefs();
    if (!_fs->exists("/range_prefs")) return;
    File file = openRead("/range_prefs");
    if (!file) return;
    RangePrefs p;
    if (file.read((uint8_t*)&p, sizeof(p)) == sizeof(p) && p.magic == PREFS_MAGIC) {
      memcpy(&_prefs, &p, sizeof(_prefs));
      _prefs.node_name[sizeof(_prefs.node_name)-1] = 0;
      _prefs.wifi_ssid[sizeof(_prefs.wifi_ssid)-1] = 0;
      _prefs.wifi_password[sizeof(_prefs.wifi_password)-1] = 0;
      _prefs.bayou_public_key[sizeof(_prefs.bayou_public_key)-1] = 0;
      _prefs.bayou_private_key[sizeof(_prefs.bayou_private_key)-1] = 0;
    }
    file.close();
  }

  void savePrefs() {
    File file = openWrite("/range_prefs");
    if (file) {
      file.write((const uint8_t*)&_prefs, sizeof(_prefs));
      file.close();
    }
  }

  void loadTargets() {
    target_count = 0;
    if (!_fs->exists("/targets")) return;
    File file = openRead("/targets");
    if (!file) return;
    uint8_t count = 0;
    if (file.read(&count, 1) == 1 && count <= MAX_TARGETS) {
      for (uint8_t i = 0; i < count; i++) {
        if (file.read(target_prefixes[i], TARGET_PREFIX_LEN) != TARGET_PREFIX_LEN) break;
        target_count++;
      }
    }
    file.close();
  }

  void saveTargets() {
    File file = openWrite("/targets");
    if (!file) return;
    file.write(&target_count, 1);
    for (uint8_t i = 0; i < target_count; i++) file.write(target_prefixes[i], TARGET_PREFIX_LEN);
    file.close();
  }

  int findTargetIndex(const uint8_t* prefix) {
    for (uint8_t i = 0; i < target_count; i++) {
      if (memcmp(target_prefixes[i], prefix, TARGET_PREFIX_LEN) == 0) return i;
    }
    return -1;
  }

  bool addTarget(ContactInfo* c) {
    if (findTargetIndex(c->id.pub_key) >= 0) return true;
    if (target_count >= MAX_TARGETS) return false;
    memcpy(target_prefixes[target_count], c->id.pub_key, TARGET_PREFIX_LEN);
    memset(&tstat[target_count], 0, sizeof(TargetStat));
    last_flood_at[target_count] = 0;
    target_count++;
    c->flags |= 0x01;   // favourite: protect from eviction on a busy public mesh
    markContactsDirty();
    saveTargets();
    return true;
  }

  bool removeTargetAt(int idx) {
    if (idx < 0 || idx >= target_count) return false;
    for (uint8_t i = idx; i < target_count - 1; i++) {
      memcpy(target_prefixes[i], target_prefixes[i+1], TARGET_PREFIX_LEN);
      tstat[i] = tstat[i+1];
      last_flood_at[i] = last_flood_at[i+1];
    }
    target_count--;
    saveTargets();
    return true;
  }

  /* ---------------- position / time / battery ---------------- */

  // Position, best source first:
  //   1. live GPS fix                                   fix=gps
  //   2. last GPS fix since THIS boot (GPS dropout)     fix=last  (sender: only for 60 s)
  //   3. the configured position (set lat/lon)          fix=man
  // The last fix is deliberately not kept across reboots: a node that was moved has been
  // power-cycled, and a persisted fix would put it back where it used to be (e.g. on the
  // desk at the other house) until GPS reacquires. Use `set pos gps` to store a fix.
  Position getPosition() {
    Position p;
    memset(&p, 0, sizeof(p));
  #if ENV_INCLUDE_GPS
    LocationProvider* lp = sensors.getLocationProvider();
    if (lp) {
      p.sats = (int)lp->satellitesCount();
      if (lp->isValid()) {
        p.src = POS_GPS;
        p.lat = ((double)lp->getLatitude()) / 1000000.0;
        p.lon = ((double)lp->getLongitude()) / 1000000.0;
        p.alt = (int)(lp->getAltitude() / 1000);
        last_fix = p;
        last_fix_at = millis();
        has_last_fix = true;
        return p;
      }
    }
  #endif
    if (has_last_fix && (_prefs.role != ROLE_SENDER || millis() - last_fix_at < SENDER_LAST_FIX_MAX_MS)) {
      int sats = p.sats;
      p = last_fix;
      p.src = POS_LAST;
      p.sats = sats;
      return p;
    }
    if (_prefs.has_fixed_pos) {
      p.src = POS_MANUAL;
      p.lat = ((double)_prefs.fixed_lat_e6) / 1000000.0;
      p.lon = ((double)_prefs.fixed_lon_e6) / 1000000.0;
      p.alt = _prefs.fixed_alt_m;
    }
    return p;
  }

  bool gpsPresent() {
  #if ENV_INCLUDE_GPS
    return sensors.getLocationProvider() != NULL;
  #else
    return false;
  #endif
  }

  uint32_t nowUnix() {   // 0 = clock not synced (no GPS fix / NTP yet)
    uint32_t t = getRTCClock()->getCurrentTime();
    return t >= TIME_VALID_MIN ? t : 0;
  }

  float battVolts() { return board.getBattMilliVolts() / 1000.0f; }

  uint32_t nextSeq() {
    seq++;
    if (seq >= _prefs.seq_base + SEQ_BLOCK) { _prefs.seq_base = seq; savePrefs(); }
    return seq;
  }

  void addPosition(Line& L, const Position& p) {
    if (p.src != POS_NONE) L.add(" lat=%.6f lon=%.6f alt=%d", p.lat, p.lon, p.alt);
  }

  /* ---------------- events: serial + flash log + Bayou ---------------- */

  void recordEvent(const char* line) {
    Serial.printf("EVENT %s\n", line);
    if (_prefs.log_enabled) appendLog(line);
  #if HAS_WIFI
    if (bayouConfigured()) enqueuePost(line);
  #endif
  }

  void appendLog(const char* line) {
    if (log_full) return;
    size_t len = strlen(line) + 1;
    if (log_bytes + len > LOG_MAX_BYTES) {
      log_full = true;
      Serial.println("   LOG FULL — upload it, then 'log clear'");
      return;
    }
    File f = openAppend("/rangelog");
    if (!f) { Serial.println("   ERROR: can't open log"); return; }
    f.write((const uint8_t*)line, len - 1);
    f.write((const uint8_t*)"\n", 1);
    f.close();
    log_bytes += len;
    log_lines_boot++;
  }

  void scanLog() {
    log_bytes = 0;
    log_full = false;
    if (!_fs->exists("/rangelog")) return;
    File f = openRead("/rangelog");
    if (f) { log_bytes = f.size(); f.close(); }
    if (log_bytes >= LOG_MAX_BYTES) log_full = true;
  }

  uint32_t countLogLines() {
    if (!_fs->exists("/rangelog")) return 0;
    File f = openRead("/rangelog");
    if (!f) return 0;
    uint32_t n = 0;
    uint8_t buf[128];
    int r;
    while ((r = f.read(buf, sizeof(buf))) > 0) {
      for (int i = 0; i < r; i++) if (buf[i] == '\n') n++;
    }
    f.close();
    return n;
  }

  void dumpLog() {
    uint32_t n = countLogLines();
    Serial.printf("--- range log begin (%lu lines, %lu bytes) ---\n", (unsigned long)n, (unsigned long)log_bytes);
    if (_fs->exists("/rangelog")) {
      File f = openRead("/rangelog");
      if (f) {
        uint8_t buf[128];
        int r;
        while ((r = f.read(buf, sizeof(buf))) > 0) {
          Serial.write(buf, r);
          delay(2);   // let USB CDC drain
        }
        f.close();
      }
    }
    Serial.println("--- range log end ---");
  }

  void clearLog() {
    _fs->remove("/rangelog");
    log_bytes = 0;
    log_lines_boot = 0;
    log_full = false;
  }

#if HAS_WIFI
  bool bayouConfigured() const {
    return _prefs.wifi_ssid[0] && _prefs.bayou_public_key[0] && _prefs.bayou_private_key[0];
  }

  void enqueuePost(const char* line) {
    if (q_count >= POST_QUEUE_SIZE) {   // drop oldest
      q_head = (q_head + 1) % POST_QUEUE_SIZE;
      q_count--;
      post_dropped++;
      q_head_since = millis();
    }
    uint8_t slot = (q_head + q_count) % POST_QUEUE_SIZE;
    StrHelper::strncpy(post_q[slot], line, LINE_MAX);
    if (q_count == 0) q_head_since = millis();
    q_count++;
  }

  // Same mapping as tools/range_log_upload.py — keep them in sync.
  bool buildJson(const char* line, char* out, size_t sz) {
    char kind[12], v[40];
    if (!kvGet(line, "k", kind, sizeof(kind))) return false;
    JsonLine J;
    J.add("{\"private_key\":\"%s\"", _prefs.bayou_private_key);

    const char* id_key = strcmp(kind, "ping") == 0 ? "sid"
                       : (strcmp(kind, "adv") == 0 || strcmp(kind, "nbr") == 0) ? "rid" : "id";
    if (kvGet(line, id_key, v, sizeof(v)) && isNumber(v)) J.add(",\"node_id\":%s", v);
    if (kvGet(line, "lat", v, sizeof(v)) && isNumber(v)) J.add(",\"gps_lat\":%s", v);
    if (kvGet(line, "lon", v, sizeof(v)) && isNumber(v)) J.add(",\"gps_lon\":%s", v);
    if (kvGet(line, "alt", v, sizeof(v)) && isNumber(v)) J.add(",\"gps_alt\":%s", v);
    if (kvGet(line, "b", v, sizeof(v)) && isNumber(v)) J.add(",\"battery_volts\":%s", v);
    if (strcmp(kind, "ping") == 0) {
      if (kvGet(line, "snr_f", v, sizeof(v)) && isNumber(v)) J.add(",\"aux_1\":%s", v);
      if (kvGet(line, "rssi_f", v, sizeof(v)) && isNumber(v)) J.add(",\"rssi\":%s", v);
      if (kvGet(line, "snr_r", v, sizeof(v)) && isNumber(v)) J.add(",\"aux_2\":%s", v);
    } else if (strcmp(kind, "adv") == 0 || strcmp(kind, "nbr") == 0) {
      if (kvGet(line, "snr", v, sizeof(v)) && isNumber(v)) J.add(",\"aux_1\":%s", v);
      if (kvGet(line, "rssi", v, sizeof(v)) && isNumber(v)) J.add(",\"rssi\":%s", v);
      if (kind[0] == 'n' && kvGet(line, "n", v, sizeof(v)) && isNumber(v)) J.add(",\"aux_2\":%s", v);
    }
    if (kvGet(line, "hops", v, sizeof(v)) && isNumber(v) && v[0] != '-') J.add(",\"aux_3\":%s", v);

    // log = the line minus the trailing position keys (they're in the gps columns); <= 250 chars
    char logbuf[252];
    const char* cut = strstr(line, " lat=");
    size_t n = cut ? (size_t)(cut - line) : strlen(line);
    if (n > sizeof(logbuf) - 1) n = sizeof(logbuf) - 1;
    memcpy(logbuf, line, n);
    logbuf[n] = 0;
    J.add(",\"log\":\"%s\"}", logbuf);
    if (J.n >= (int)sizeof(J.s) - 1 || (size_t)J.n >= sz) return false;
    memcpy(out, J.s, J.n + 1);
    return true;
  }

  int httpPost(const char* json) {
    HTTPClient http;
    String url = String(BAYOU_BASE_URL) + String(_prefs.bayou_public_key);
    if (!http.begin(url)) return -100;
    http.setConnectTimeout(5000);
    http.setTimeout(8000);
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST((uint8_t*)json, strlen(json));
    if (code >= 300 || code < 0) {
      String body = (code > 0) ? http.getString() : String();
      Serial.printf("   Bayou POST failed (%d) %s\n", code, body.c_str());
    }
    http.end();
    return code;
  }

  void serviceBayou() {
    if (q_count == 0) return;
    unsigned long now = millis();
    if (WiFi.status() != WL_CONNECTED) { StrHelper::strncpy(post_status, "no wifi", sizeof(post_status)); return; }
    if ((long)(now - q_next_try) < 0) return;
    // An HTTPS post blocks the loop for ~1 s. Don't do it while our ACK / reply may still be in
    // flight (just after a packet arrived) or while we are waiting for our own ACK.
    if (ss_state == SS_WAIT_ACK) return;
    if ((now - last_rx_millis) < 2500 && (now - q_head_since) < 30000) return;

    char json[JSON_MAX];
    if (!buildJson(post_q[q_head], json, sizeof(json))) {
      Serial.println("   Bayou: dropping unparseable row");
      q_head = (q_head + 1) % POST_QUEUE_SIZE; q_count--; q_head_since = now;
      return;
    }
    int code = httpPost(json);
    if (code >= 200 && code < 300) {
      post_ok++;
      StrHelper::strncpy(post_status, "ok", sizeof(post_status));
      q_head = (q_head + 1) % POST_QUEUE_SIZE; q_count--; q_head_since = millis();
    } else if (code >= 400 && code < 500) {   // bad key / bad feed: retrying won't help
      post_fail++;
      snprintf(post_status, sizeof(post_status), "rejected %d", code);
      q_head = (q_head + 1) % POST_QUEUE_SIZE; q_count--; q_head_since = millis();
    } else {
      post_fail++;
      snprintf(post_status, sizeof(post_status), "fail %d", code);
      q_next_try = millis() + 15000;
    }
  }

  void beginWifi() {
    if (_prefs.wifi_ssid[0] == 0) {
      StrHelper::strncpy(wifi_status, "not set", sizeof(wifi_status));
      return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(_prefs.wifi_ssid, _prefs.wifi_password);
    last_wifi_kick = millis();
    StrHelper::strncpy(wifi_status, "connecting", sizeof(wifi_status));
    Serial.printf("   WiFi connecting to: %s\n", _prefs.wifi_ssid);
  }

  void serviceWifi() {
    if (_prefs.wifi_ssid[0] == 0) return;
    unsigned long now = millis();
    if (WiFi.status() == WL_CONNECTED) {
      StrHelper::strncpy(wifi_status, WiFi.localIP().toString().c_str(), sizeof(wifi_status));
      if (!ntp_started) {
        configTime(0, 0, "pool.ntp.org", "time.google.com");   // UTC
        ntp_started = true;
      }
    } else {
      StrHelper::strncpy(wifi_status, "connecting", sizeof(wifi_status));
      if (now - last_wifi_kick > 60000) {
        last_wifi_kick = now;
        WiFi.disconnect(false, false);
        WiFi.begin(_prefs.wifi_ssid, _prefs.wifi_password);
      }
    }
    // configTime() sets the ESP32 system clock, which the fallback RTC reads; nothing else to do.
  }
#else
  bool bayouConfigured() const { return false; }
#endif

  /* ---------------- direct neighbours ---------------- */

  static bool rssiValid(float rssi) { return rssi < -1.0f; }   // 0 = register read after RX restarted

  void recordNeighbour(const uint8_t* hash, uint8_t hsz) {
    NbrStat* e = NULL;
    for (uint8_t i = 0; i < nbr_count; i++) {
      if (nbrs[i].hsz == hsz && memcmp(nbrs[i].hash, hash, hsz) == 0) { e = &nbrs[i]; break; }
    }
    if (!e) {
      if (nbr_count >= MAX_NBRS) { nbr_overflow++; return; }
      e = &nbrs[nbr_count++];
      memset(e, 0, sizeof(*e));
      memcpy(e->hash, hash, hsz);
      e->hsz = hsz;
      e->snr_min = 99; e->snr_max = -99;
    }
    e->n++;
    e->snr_sum += last_rx_snr;
    if (last_rx_snr < e->snr_min) e->snr_min = last_rx_snr;
    if (last_rx_snr > e->snr_max) e->snr_max = last_rx_snr;
    if (rssiValid(last_rx_rssi)) { e->rssi_sum += last_rx_rssi; e->rssi_n++; }
    e->last_at = millis();
  }

  // The same repeater shows up under 1-, 2- and 3-byte hashes (senders choose the path hash
  // size), e.g. 35 / 3582 / 358276. Fold each shorter hash into the longest entry it prefixes,
  // unless the longer entries it prefixes disagree (then it's ambiguous and stays separate).
  void foldNeighbours() {
    for (uint8_t i = 0; i < nbr_count; i++) {
      NbrStat& s = nbrs[i];
      if (s.n == 0) continue;
      int best = -1;
      bool ambiguous = false;
      for (uint8_t j = 0; j < nbr_count && !ambiguous; j++) {
        const NbrStat& c = nbrs[j];
        if (j == i || c.n == 0 || c.hsz <= s.hsz || memcmp(c.hash, s.hash, s.hsz) != 0) continue;
        if (best < 0) { best = j; continue; }
        const NbrStat& b = nbrs[best];
        uint8_t m = b.hsz < c.hsz ? b.hsz : c.hsz;
        if (memcmp(b.hash, c.hash, m) != 0) ambiguous = true;
        else if (c.hsz > b.hsz) best = j;
      }
      if (best < 0 || ambiguous) continue;
      NbrStat& d = nbrs[best];
      d.n += s.n;
      d.snr_sum += s.snr_sum;
      if (s.snr_min < d.snr_min) d.snr_min = s.snr_min;
      if (s.snr_max > d.snr_max) d.snr_max = s.snr_max;
      d.rssi_sum += s.rssi_sum;
      d.rssi_n += s.rssi_n;
      if ((long)(s.last_at - d.last_at) > 0) d.last_at = s.last_at;
      s.n = 0;   // folded
    }
    // compact
    uint8_t k = 0;
    for (uint8_t i = 0; i < nbr_count; i++) if (nbrs[i].n) nbrs[k++] = nbrs[i];
    nbr_count = k;
  }

  void sortNeighboursByCount() {
    for (uint8_t i = 1; i < nbr_count; i++) {
      NbrStat t = nbrs[i];
      int j = i - 1;
      while (j >= 0 && nbrs[j].n < t.n) { nbrs[j + 1] = nbrs[j]; j--; }
      nbrs[j + 1] = t;
    }
  }

  void printNeighbours() {
    foldNeighbours();
    sortNeighboursByCount();
    unsigned long span = (millis() - nbr_window_start) / 1000;
    Serial.printf("   direct neighbours heard in the last %lus (from flood traffic):\n", span);
    if (nbr_count == 0) { Serial.println("   (none yet)"); return; }
    for (uint8_t i = 0; i < nbr_count; i++) {
      const NbrStat& e = nbrs[i];
      char h[8];
      toHexLower(h, e.hash, e.hsz);
      Serial.printf("   %-6s n=%-4u snr avg %+.1f (min %+.1f max %+.1f)", h, (unsigned)e.n, e.snr_sum / e.n, e.snr_min, e.snr_max);
      if (e.rssi_n) Serial.printf("  rssi %.0f", e.rssi_sum / e.rssi_n);
      Serial.printf("  last %lus ago\n", (millis() - e.last_at) / 1000);
    }
    if (nbr_overflow) Serial.printf("   (+%u packets from neighbours beyond the %d tracked)\n", (unsigned)nbr_overflow, MAX_NBRS);
  }

  // One k=nbr row per neighbour heard since the last summary (busiest first, capped), then
  // start a new window. Households: hourly, no position (they're fixed). Sender: every round,
  // tagged with the round's position (fix=...), so the map shows where each repeater is heard.
  void emitNeighbours(bool at_round = false) {
    foldNeighbours();
    sortNeighboursByCount();
    if (nbr_count > 0) {
      char me[20];
      sanitizeName(me, sizeof(me), _prefs.node_name);
      unsigned long span = (millis() - nbr_window_start) / 1000;
      uint8_t cap = at_round ? MAX_ROUND_NBR_ROWS : MAX_NBR_ROWS;
      for (uint8_t i = 0; i < nbr_count && i < cap; i++) {
        const NbrStat& e = nbrs[i];
        char h[8];
        toHexLower(h, e.hash, e.hsz);
        Line L;
        L.add("k=nbr v=1 t=%lu rep=%s rid=%u rk=%s hash=%s n=%u snr=%.2f snr_min=%.2f snr_max=%.2f",
              (unsigned long)nowUnix(), me, (unsigned)_prefs.node_id, self_key, h, (unsigned)e.n,
              e.snr_sum / e.n, e.snr_min, e.snr_max);
        if (e.rssi_n) L.add(" rssi=%d", (int)lroundf(e.rssi_sum / e.rssi_n));
        L.add(" span=%lu", span);
        if (at_round) {
          L.add(" fix=%s", fixName(cycle_pos.src));
          addPosition(L, cycle_pos);
        }
        recordEvent(L.s);
      }
    }
    nbr_count = 0;
    nbr_overflow = 0;
    nbr_window_start = millis();
  }

  /* ---------------- status + repeater adverts (posted by Wi-Fi nodes) ---------------- */

  void emitStatus() {
    char nm[20];
    sanitizeName(nm, sizeof(nm), _prefs.node_name);
    Position p = getPosition();
    Line L;
    L.add("k=status v=1 t=%lu name=%s id=%u key=%s role=%s fix=%s",
          (unsigned long)nowUnix(), nm, (unsigned)_prefs.node_id, self_key, roleName(_prefs.role), fixName(p.src));
    L.add(" up_s=%lu nc=%d", (unsigned long)(millis() / 1000), getNumContacts());
  #if HAS_WIFI
    if (WiFi.status() == WL_CONNECTED) L.add(" wifi=%d", (int)WiFi.RSSI());
  #endif
    L.add(" fw=%s b=%.2f", FW_TAG, battVolts());
    addPosition(L, p);
    recordEvent(L.s);
  }

  void serviceStatus() {
    if (!bayouConfigured()) return;
    unsigned long now = millis();
    if ((long)(now - next_status_at) < 0) return;
    // first status: wait for a synced clock (or 2 min), so the row has a real time
    if (next_status_at == 0 || (nowUnix() == 0 && now < 120000)) {
      if (next_status_at == 0) next_status_at = now + 15000;
      return;
    }
    // Neighbour summary rides along with the status row (skip a near-empty window at boot).
    // Status goes last: if the post queue ever overflows it drops the OLDEST rows, and the
    // status row is what puts this station on the map.
    // (a sender summarises its neighbours every round instead, with position)
    if (_prefs.role != ROLE_SENDER && now - nbr_window_start >= 5UL * 60UL * 1000UL) emitNeighbours();
    emitStatus();
    next_status_at = now + _prefs.status_interval_secs * 1000UL;
  }

  // Repeater/room adverts this node hears: name + hash (+ position if advertised) + the flood path
  // the advert took to reach us. Lets the map name the hops in a route, and shows which public
  // repeaters each household can hear (and through whom).
  void maybePostAdvert(const ContactInfo& c, uint8_t path_len, const uint8_t* path) {
    if (!bayouConfigured()) return;
    if (c.type != ADV_TYPE_REPEATER && c.type != ADV_TYPE_ROOM) return;   // never post people's (chat) nodes
    bool has_pos = !(c.gps_lat == 0 && c.gps_lon == 0);

    unsigned long now = millis();
    AdvSeen* slot = NULL;
    for (uint8_t i = 0; i < sizeof(adv_seen)/sizeof(adv_seen[0]); i++) {
      if (memcmp(adv_seen[i].key, c.id.pub_key, 4) == 0) { slot = &adv_seen[i]; break; }
    }
    if (slot) {
      bool moved = has_pos && (abs(slot->lat - c.gps_lat) > 1000 || abs(slot->lon - c.gps_lon) > 1000);   // ~100 m
      if (!moved && (now - slot->at) < ADV_POST_MIN_INTERVAL_MS) return;
    } else {
      slot = &adv_seen[adv_next];
      adv_next = (adv_next + 1) % (sizeof(adv_seen)/sizeof(adv_seen[0]));
    }
    memcpy(slot->key, c.id.pub_key, 4);
    slot->lat = c.gps_lat; slot->lon = c.gps_lon; slot->at = now;

    char me[20], nm[20], k[9], pth[64];
    sanitizeName(me, sizeof(me), _prefs.node_name);
    sanitizeName(nm, sizeof(nm), c.name);
    keyHex(k, c.id.pub_key);
    formatPath(pth, sizeof(pth), path, path_len, 12);
    Line L;
    L.add("k=adv v=1 t=%lu rep=%s rid=%u rk=%s name=%s key=%s type=%s hops=%d path=%s snr=%.2f",
          (unsigned long)nowUnix(), me, (unsigned)_prefs.node_id, self_key, nm, k,
          c.type == ADV_TYPE_REPEATER ? "rpt" : "room", (int)(path_len & 63), pth, last_rx_snr);
    if (rssiValid(last_rx_rssi)) L.add(" rssi=%d", (int)lroundf(last_rx_rssi));
    if (has_pos) L.add(" lat=%.6f lon=%.6f", c.gps_lat / 1000000.0, c.gps_lon / 1000000.0);
    recordEvent(L.s);
  }

  /* ---------------- sender ---------------- */

  uint8_t curLeg() const { return cycle_legs[ss_leg]; }

  void startCycle() {
    cycle_pos = getPosition();
    cycle_batt = battVolts();
    cycle_t = nowUnix();
    // a roaming sender also logs which repeaters it heard directly since the last round,
    // tagged with this round's position: where each repeater is audible
    if (_prefs.role == ROLE_SENDER) emitNeighbours(true);
    cycle_nlegs = 0;
    if (_prefs.mode != MODE_ROUTED) cycle_legs[cycle_nlegs++] = LEG_ZH;
    if (_prefs.mode != MODE_DIRECT) cycle_legs[cycle_nlegs++] = LEG_RT;
    ss_leg = 0;
    startLeg();
  }

  // Each leg gets its own seq: (sender key, seq, target) identifies one attempt, and the
  // map must never merge a zero-hop ping with a routed one.
  void startLeg() {
    cycle_seq = nextSeq();
    ss_idx = 0;
    ss_state = SS_NEXT;
    ss_next_at = millis();
    Serial.printf("[ROUND %lu %s] pos=%s", (unsigned long)cycle_seq, curLeg() == LEG_ZH ? "direct" : "routed",
                  fixName(cycle_pos.src));
    if (cycle_pos.src) Serial.printf(" %.6f,%.6f", cycle_pos.lat, cycle_pos.lon);
    Serial.printf(" targets=%d\n", target_count);
  }

  void advanceTarget(unsigned long gap_ms) {
    expected_ack = 0;
    ss_idx++;
    ss_state = SS_NEXT;
    ss_next_at = millis() + gap_ms;
  }

  void sendToTarget(uint8_t i) {
    unsigned long now = millis();
    LegStat& ts = tstat[i].leg[curLeg()];
    ContactInfo* c = lookupContactByPubKey(target_prefixes[i], TARGET_PREFIX_LEN);
    if (!c) {
      Serial.print("   skip: no contact for target ");
      mesh::Utils::printHex(Serial, target_prefixes[i], TARGET_PREFIX_LEN);
      Serial.println(" (import its card)");
      ts.result = RES_NOCONTACT; ts.at_millis = now;
      advanceTarget(0);
      return;
    }
    sanitizeName(ss_dst_name, sizeof(ss_dst_name), c->name);
    keyHex(ss_dst_key, c->id.pub_key);

    bool direct_mode = (curLeg() == LEG_ZH);
    // In "both" mode the routed leg is route DISCOVERY: always flood, so the receiving house
    // records which repeaters carry us from this spot (a learned path would just replay the
    // route found somewhere else).
    if (!direct_mode && _prefs.mode == MODE_BOTH) c->out_path_len = OUT_PATH_UNKNOWN;
    if (!direct_mode && c->out_path_len == OUT_PATH_UNKNOWN &&
        last_flood_at[i] != 0 && (now - last_flood_at[i]) < ROUTED_MIN_FLOOD_GAP_MS) {
      Serial.printf("   -> %s: skip routed (last flood to it <1 min ago)\n", c->name);
      ts.result = RES_SKIPPED; ts.at_millis = now;
      advanceTarget(0);
      return;
    }

    // forward path as we know it (routed + direct send only)
    if (direct_mode) {
      ss_fwd_hops = 0;
      StrHelper::strncpy(ss_fwd_path, "-", sizeof(ss_fwd_path));
    } else if (c->out_path_len == OUT_PATH_UNKNOWN) {
      ss_fwd_hops = 255;
      StrHelper::strncpy(ss_fwd_path, "-", sizeof(ss_fwd_path));
    } else {
      ss_fwd_hops = c->out_path_len & 63;
      formatPath(ss_fwd_path, sizeof(ss_fwd_path), c->out_path, c->out_path_len, 6);
    }

    Line msg;
    msg.add(MSG_PREFIX " seq=%lu", (unsigned long)cycle_seq);
    if (cycle_t) msg.add(" t=%lu", (unsigned long)cycle_t);
    msg.add(" sid=%u m=%s fix=%s", (unsigned)_prefs.node_id, direct_mode ? "zh" : "rt", fixName(cycle_pos.src));
    addPosition(msg, cycle_pos);
    msg.add(" b=%.2f", cycle_batt);
    if (!direct_mode) msg.add(" fh=%u fp=%s", (unsigned)ss_fwd_hops, ss_fwd_path);

    uint8_t saved_len = c->out_path_len;
    if (direct_mode) c->out_path_len = 0;    // zero-hop: empty direct path = not repeated by anyone
    uint32_t est_timeout = 0;
    int rc = sendMessage(*c, getRTCClock()->getCurrentTime(), 0, msg.s, expected_ack, est_timeout);
    if (direct_mode) c->out_path_len = saved_len;

    ts.attempts++;
    if (rc == MSG_SEND_FAILED) {
      Serial.printf("   -> %s: send FAILED\n", c->name);
      ts.result = RES_FAIL; ts.at_millis = now;
      advanceTarget(500);
      return;
    }
    ss_route = direct_mode ? ROUTE_ZH : (rc == MSG_SEND_SENT_FLOOD ? ROUTE_FL : ROUTE_DR);
    if (ss_route == ROUTE_FL) last_flood_at[i] = now;
    ss_sent_at = now;
    // Our own deadline (a late ACK after this would be mis-scored as a failure, so err long
    // for routed sends: floods + the flooded path-return can take many seconds per hop).
    uint32_t to;
    if (ss_route == ROUTE_ZH) {
      to = est_timeout * 2 + 1000;
      if (to < 4000) to = 4000;
    } else if (ss_route == ROUTE_FL) {
      to = est_timeout * 3;
      if (to < FLOOD_ACK_TIMEOUT_MIN_MS) to = FLOOD_ACK_TIMEOUT_MIN_MS;
    } else {
      to = est_timeout * 2 + 2000;
      if (to < DIRECT_ACK_TIMEOUT_MIN_MS) to = DIRECT_ACK_TIMEOUT_MIN_MS;
    }
    ss_deadline = now + to;
    ts.result = RES_PENDING; ts.at_millis = now;
    ss_state = SS_WAIT_ACK;
    Serial.printf("   -> %s: sent (%s, path=%s, wait %lums)\n", c->name, routeName(ss_route), ss_fwd_path, (unsigned long)to);
  }

  void emitSendEvent(const char* res, int hops, const char* path, bool with_ack, const LegStat& ts) {
    char me[20];
    sanitizeName(me, sizeof(me), _prefs.node_name);
    Line L;
    L.add("k=ping v=1 t=%lu src=%s sid=%u sk=%s dst=%s did=0 dk=%s seq=%lu m=%s res=%s fix=%s hops=%d path=%s",
          (unsigned long)cycle_t, me, (unsigned)_prefs.node_id, self_key, ss_dst_name, ss_dst_key,
          (unsigned long)cycle_seq, routeName(ss_route), res, fixName(cycle_pos.src), hops, path);
    if (with_ack) {
      L.add(" snr_r=%.2f", ts.ack_snr);
      if (rssiValid(ts.ack_rssi)) L.add(" rssi_r=%d", (int)ts.ack_rssi);
      L.add(" rtt=%lu", (unsigned long)ts.rtt_ms);
    }
    L.add(" b=%.2f", cycle_batt);
    addPosition(L, cycle_pos);
    recordEvent(L.s);
  }

  void onTargetAck(ContactInfo* c) {
    unsigned long now = millis();
    LegStat& ts = tstat[ss_idx].leg[curLeg()];
    ts.result = RES_OK;
    ts.oks++;
    ts.rtt_ms = now - ss_sent_at;
    ts.ack_snr = last_rx_snr;
    ts.ack_rssi = (int16_t)lroundf(last_rx_rssi);
    ts.at_millis = now;

    int hops = 0;
    char path[64];
    StrHelper::strncpy(path, "-", sizeof(path));
    if (ss_route == ROUTE_DR) {
      hops = ss_fwd_hops;
      StrHelper::strncpy(path, ss_fwd_path, sizeof(path));
    } else if (ss_route == ROUTE_FL) {   // path just learned from the receiver's path-return
      if (c && c->out_path_len != OUT_PATH_UNKNOWN) {
        hops = c->out_path_len & 63;
        formatPath(path, sizeof(path), c->out_path, c->out_path_len, 12);
      } else {
        hops = -1;
      }
    }
    ts.hops = (int8_t)hops;
    Serial.printf("   ACK from %s: rtt=%lums snr=%.2f rssi=%d hops=%d path=%s\n", ss_dst_name,
                  (unsigned long)ts.rtt_ms, ts.ack_snr, ts.ack_rssi, hops, path);
    emitSendEvent("ok", hops, path, true, ts);
    advanceTarget(400);
  }

  void onTargetTimeout() {
    LegStat& ts = tstat[ss_idx].leg[curLeg()];
    ts.result = RES_TIMEOUT;
    ts.at_millis = millis();
    int hops = (ss_route == ROUTE_ZH) ? 0 : (ss_route == ROUTE_DR ? ss_fwd_hops : -1);
    Serial.printf("   no ACK from %s\n", ss_dst_name);
    emitSendEvent("to", hops, ss_route == ROUTE_DR ? ss_fwd_path : "-", false, ts);
    if (ss_route == ROUTE_DR) {   // stale path: next routed attempt floods to rediscover
      ContactInfo* c = lookupContactByPubKey(target_prefixes[ss_idx], TARGET_PREFIX_LEN);
      if (c && c->out_path_len != OUT_PATH_UNKNOWN) {
        c->out_path_len = OUT_PATH_UNKNOWN;
        markContactsDirty();
      }
    }
    advanceTarget(400);
  }

  void serviceSender() {
    unsigned long now = millis();
    switch (ss_state) {
      case SS_IDLE:
        if (_prefs.send_interval_secs > 0 && target_count > 0 && (long)(now - next_cycle_at) >= 0) {
          next_cycle_at = now + _prefs.send_interval_secs * 1000UL;
          startCycle();
        }
        break;
      case SS_WAIT_ACK:
        if ((long)(now - ss_deadline) >= 0) onTargetTimeout();
        break;
      case SS_NEXT:
        if ((long)(now - ss_next_at) < 0) break;
        if (ss_idx >= target_count) {                    // leg done
          if (++ss_leg < cycle_nlegs) startLeg(); else ss_state = SS_IDLE;
          break;
        }
        sendToTarget(ss_idx);
        break;
    }
  }

  /* ---------------- receiver ---------------- */

  void handleRangeMessage(const ContactInfo& from, mesh::Packet* pkt, const char* text) {
    char v_seq[16] = "0", v_t[16] = "0", v_sid[8] = "0", v_m[6] = "", v_fix[8] = "none";
    char v_lat[16], v_lon[16], v_alt[10], v_b[10] = "0", v_fh[6] = "", v_fp[64] = "-";
    kvGet(text, "seq", v_seq, sizeof(v_seq));
    kvGet(text, "t", v_t, sizeof(v_t));
    kvGet(text, "sid", v_sid, sizeof(v_sid));
    kvGet(text, "m", v_m, sizeof(v_m));
    kvGet(text, "fix", v_fix, sizeof(v_fix));
    kvGet(text, "b", v_b, sizeof(v_b));
    kvGet(text, "fh", v_fh, sizeof(v_fh));
    kvGet(text, "fp", v_fp, sizeof(v_fp));
    bool has_pos = kvGet(text, "lat", v_lat, sizeof(v_lat)) && kvGet(text, "lon", v_lon, sizeof(v_lon))
                   && isNumber(v_lat) && isNumber(v_lon);
    if (!kvGet(text, "alt", v_alt, sizeof(v_alt)) || !isNumber(v_alt)) StrHelper::strncpy(v_alt, "0", sizeof(v_alt));

    ContactInfo* c = lookupContactByPubKey(from.id.pub_key, PUB_KEY_SIZE);
    const char* mode;
    int hops;
    char path[64];
    if (strcmp(v_m, "zh") == 0) {
      mode = "zh"; hops = 0;
      StrHelper::strncpy(path, "-", sizeof(path));
      // The base class ACKs after this returns, along c->out_path. Empty path = zero-hop ACK,
      // so a direct-mode test never floods an ACK onto the mesh.
      if (c) c->out_path_len = 0;
    } else if (pkt->isRouteFlood()) {
      mode = "fl";
      hops = pkt->getPathHashCount();
      formatPath(path, sizeof(path), pkt->path, pkt->path_len, 12);
    } else {
      // direct arrival: repeaters strip the path as they go, so trust the sender's view
      mode = "dr";
      hops = (v_fh[0] && isNumber(v_fh) && atoi(v_fh) != 255) ? atoi(v_fh) : -1;
      StrHelper::strncpy(path, v_fp, sizeof(path));
    }

    if (c && !(c->flags & 0x01)) {   // keep senders from being evicted
      c->flags |= 0x01;
      markContactsDirty();
    }

    uint32_t t = strtoul(v_t, NULL, 10);
    if (t < TIME_VALID_MIN) t = nowUnix();

    char src[20], me[20], sk[9];
    sanitizeName(src, sizeof(src), from.name);
    sanitizeName(me, sizeof(me), _prefs.node_name);
    keyHex(sk, from.id.pub_key);
    float snr = pkt->getSNR();

    Line L;
    L.add("k=ping v=1 t=%lu src=%s sid=%s sk=%s dst=%s did=%u dk=%s seq=%s m=%s res=rx fix=%s hops=%d path=%s",
          (unsigned long)t, src, isNumber(v_sid) ? v_sid : "0", sk, me, (unsigned)_prefs.node_id, self_key,
          isNumber(v_seq) ? v_seq : "0", mode, v_fix, hops, path);
    L.add(" snr_f=%.2f", snr);
    if (rssiValid(last_rx_rssi)) L.add(" rssi_f=%d", (int)lroundf(last_rx_rssi));
    L.add(" b=%s", isNumber(v_b) ? v_b : "0");
    if (has_pos) L.add(" lat=%s lon=%s alt=%s", v_lat, v_lon, v_alt);
    recordEvent(L.s);

    StrHelper::strncpy(rx_from, src, sizeof(rx_from));
    rx_snr = snr;
    rx_rssi = last_rx_rssi;
    rx_hops = hops;
    StrHelper::strncpy(rx_mode, mode, sizeof(rx_mode));
    rx_at = millis();
    rx_count++;
  }

  void importCard(const char* cmd) {
    while (*cmd == ' ') cmd++;
    if (memcmp(cmd, "meshcore://", 11) == 0) {
      cmd += 11;
      char *ep = (char*)strchr(cmd, 0);
      while (ep > cmd) {
        ep--;
        if (mesh::Utils::isHexChar(*ep)) break;
        *ep = 0;
      }
      int len = strlen(cmd);
      if (len % 2 == 0 && len / 2 <= (int)sizeof(tmp_buf)) {
        len >>= 1;
        if (mesh::Utils::fromHex(tmp_buf, len, cmd)) {
          if (importContact(tmp_buf, len)) {
            // remember the pub_key so the contact gets favourited when it is added
            uint8_t off = 1;
            uint8_t route = tmp_buf[0] & 0x03;
            if (route == 0x00 || route == 0x03) off += 4;   // transport codes
            uint8_t plen = tmp_buf[off++];
            off += (plen & 63) * ((plen >> 6) + 1);
            if ((int)(off + PUB_KEY_SIZE) <= len) {
              memcpy(pending_fav_pub_key, &tmp_buf[off], PUB_KEY_SIZE);
              has_pending_fav = true;
            }
            Serial.println("   OK, imported (watch for 'ADVERT from -> ...')");
          } else {
            Serial.println("   error: importContact failed (bad packet)");
          }
          return;
        }
      }
    }
    Serial.println("   error: invalid format (expected meshcore://<hex>)");
  }

protected:
  float getAirtimeBudgetFactor() const override { return 1.0; }
  int calcRxDelay(float score, uint32_t air_time) const override { return 0; }

  bool allowPacketForward(const mesh::Packet* packet) override {
    if (!_prefs.repeat) return false;
    if (packet->isRouteFlood() && packet->getPathHashCount() >= FLOOD_FORWARD_MAX_HOPS) return false;
    return true;
  }

  // Called by the Dispatcher straight after the radio hands over a packet: the only
  // reliable moment to read its RSSI (once RX restarts the register can read 0).
  void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override {
    last_rx_rssi = rssi;
  }

  void logRx(mesh::Packet* packet, int len, float score) override {
    last_rx_snr = packet->getSNR();
    last_rx_millis = millis();
    if (packet->isRouteFlood() && packet->getPathHashCount() > 0) {
      uint8_t hsz = packet->getPathHashSize();
      if (hsz >= 1 && hsz <= 3) recordNeighbour(&packet->path[(packet->getPathHashCount() - 1) * hsz], hsz);
    }
  }

  bool shouldOverwriteWhenFull() const override { return true; }

  void onContactOverwrite(const uint8_t* pub_key) override { }

  void onContactsFull() override {
    Serial.printf("   WARNING: contacts full (%d/%d) and all favourited\n", getNumContacts(), MAX_CONTACTS);
  }

  void onDiscoveredContact(ContactInfo& contact, bool is_new, uint8_t path_len, const uint8_t* path) override {
    if (has_pending_fav && memcmp(contact.id.pub_key, pending_fav_pub_key, PUB_KEY_SIZE) == 0) {
      contact.flags |= 0x01;
      has_pending_fav = false;
      Serial.printf("ADVERT from -> %s (imported, favourited)\n", contact.name);
    } else if (is_new || (contact.flags & 0x01)) {
      Serial.printf("ADVERT from -> %s%s (hops=%d snr=%.1f)\n", contact.name, is_new ? " [NEW]" : "",
                    (int)(path_len & 63), last_rx_snr);
    }
    maybePostAdvert(contact, path_len, path);
    markContactsDirty();
  }

  void onContactPathUpdated(const ContactInfo& contact) override {
    char p[64];
    formatPath(p, sizeof(p), contact.out_path, contact.out_path_len, 16);
    Serial.printf("PATH to %s: hops=%d path=%s\n", contact.name, (int)(contact.out_path_len & 63), p);
    markContactsDirty();
  }

  ContactInfo* processAck(const uint8_t *data) override {
    if (ss_state == SS_WAIT_ACK && expected_ack != 0 && memcmp(data, &expected_ack, 4) == 0) {
      ContactInfo* c = lookupContactByPubKey(target_prefixes[ss_idx], TARGET_PREFIX_LEN);
      onTargetAck(c);
      return c;
    }
    return NULL;
  }

  void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
    if (memcmp(text, MSG_PREFIX, strlen(MSG_PREFIX)) == 0) {
      Serial.printf("(%s) %s from %s\n", pkt->isRouteDirect() ? "DIRECT" : "FLOOD", MSG_PREFIX, from.name);
      handleRangeMessage(from, pkt, text);
    } else {
      Serial.printf("(%s) MSG from %s: %s\n", pkt->isRouteDirect() ? "DIRECT" : "FLOOD", from.name, text);
    }
  }

  void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {}
  void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override {}
  void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t timestamp, const char *text) override {}
  uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp, const uint8_t* data, uint8_t len, uint8_t* reply) override { return 0; }
  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override {}

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    uint8_t path_hash_count = path_len & 63;
    return SEND_TIMEOUT_BASE_MILLIS +
         ((pkt_airtime_millis*DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_hash_count + 1));
  }
  void onSendTimeout() override { }   // we run our own per-target deadline (serviceSender)

public:
  RangeMesh(mesh::Radio& radio, StdRNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    command[0] = 0;
    target_count = 0;
    memset(tstat, 0, sizeof(tstat));
    memset(last_flood_at, 0, sizeof(last_flood_at));
    ss_state = SS_IDLE; ss_idx = 0; seq = 0; cycle_seq = 0; cycle_t = 0;
    cycle_legs[0] = LEG_ZH; cycle_legs[1] = LEG_RT; cycle_nlegs = 0; ss_leg = 0;
    memset(&cycle_pos, 0, sizeof(cycle_pos));
    cycle_batt = 0;
    next_cycle_at = ss_next_at = ss_deadline = ss_sent_at = 0;
    expected_ack = 0; ss_route = ROUTE_ZH; ss_fwd_hops = 0;
    ss_fwd_path[0] = 0; ss_dst_name[0] = 0; ss_dst_key[0] = 0;
    last_rx_snr = 0; last_rx_rssi = 0; last_rx_millis = 0;
    nbr_count = 0; nbr_overflow = 0; nbr_window_start = 0;
    memset(&last_fix, 0, sizeof(last_fix)); last_fix_at = 0; has_last_fix = false;
    rx_from[0] = 0; rx_snr = 0; rx_rssi = 0; rx_hops = 0; rx_mode[0] = 0; rx_at = 0; rx_count = 0;
    log_bytes = 0; log_lines_boot = 0; log_full = false;
    contacts_save_at = 0;
    has_pending_fav = false;
    next_flood_advert_at = 0;
    next_status_at = 0;
    memset(adv_seen, 0, sizeof(adv_seen));
    adv_next = 0;
  #if HAS_WIFI
    q_head = q_count = 0; q_next_try = q_head_since = 0;
    post_ok = post_fail = post_dropped = 0;
    StrHelper::strncpy(post_status, "idle", sizeof(post_status));
    StrHelper::strncpy(wifi_status, "off", sizeof(wifi_status));
    ntp_started = false;
    last_wifi_kick = 0;
  #endif
    setDefaultPrefs();
  }

  /* ---------------- accessors for the display ---------------- */
  const char* getNodeName() const { return _prefs.node_name; }
  uint8_t getRole() const { return _prefs.role; }
  bool isRepeating() const { return _prefs.repeat != 0; }
  bool displayAlwaysOn() const { return _prefs.display_always_on != 0; }
  uint8_t getTargetCount() const { return target_count; }
  const TargetStat& getTargetStat(int i) const { return tstat[i]; }
  uint32_t getSeq() const { return cycle_seq; }
  bool isLogFull() const { return log_full; }
  uint32_t getLogKB() const { return log_bytes / 1024; }
  uint32_t getRxCount() const { return rx_count; }
  const char* getRxFrom() const { return rx_from; }
  float getRxSnr() const { return rx_snr; }
  float getRxRssi() const { return rx_rssi; }
  int getRxHops() const { return rx_hops; }
  const char* getRxMode() const { return rx_mode; }
  unsigned long getRxAt() const { return rx_at; }
  Position currentPosition() { return getPosition(); }
  bool hasGps() { return gpsPresent(); }
  long secsToNextCycle() const {
    if (_prefs.send_interval_secs == 0 || target_count == 0) return -1;
    long d = (long)(next_cycle_at - millis());
    return d > 0 ? d / 1000 : 0;
  }
  bool isBusy() const { return ss_state != SS_IDLE; }
#if HAS_WIFI
  const char* getWifiStatus() const { return wifi_status; }
  const char* getPostStatus() const { return post_status; }
  uint8_t getQueueLen() const { return q_count; }
  uint8_t getNbrCount() const { return nbr_count; }
  bool wifiConfigured() const { return _prefs.wifi_ssid[0] != 0; }
#endif

  void getTargetName(int i, char* dst, size_t sz) {
    ContactInfo* c = lookupContactByPubKey(target_prefixes[i], TARGET_PREFIX_LEN);
    if (c) StrHelper::strncpy(dst, c->name, sz);
    else { toHexLower(dst, target_prefixes[i], 3); }
  }

  /* ---------------- actions ---------------- */

  bool sendNow() {
    if (target_count == 0) { Serial.println("   No targets (use 'target add <name>')"); return false; }
    if (ss_state != SS_IDLE) { Serial.println("   busy (cycle in progress)"); return false; }
    if (_prefs.send_interval_secs > 0) next_cycle_at = millis() + _prefs.send_interval_secs * 1000UL;
    startCycle();
    return true;
  }

  bool sendAdvert(bool flood) {
    auto pkt = createSelfAdvert(_prefs.node_name);   // no location in adverts
    if (!pkt) { Serial.println("   ERR: unable to send advert"); return false; }
    if (flood) sendFlood(pkt, 1200); else sendZeroHop(pkt);
    Serial.printf("   (advert sent, %s)\n", flood ? "flood" : "zero hop");
    return true;
  }

  void toggleRepeat() {
    _prefs.repeat = !_prefs.repeat;
    savePrefs();
    Serial.printf("   repeat = %s\n", _prefs.repeat ? "on" : "off");
  }

  void begin(FILESYSTEM& fs) {
    _fs = &fs;
    BaseChatMesh::begin();

  #if defined(NRF52_PLATFORM)
    IdentityStore store(fs, "");
  #elif defined(RP2040_PLATFORM)
    IdentityStore store(fs, "/identity");
    store.begin();
  #else
    IdentityStore store(fs, "/identity");
  #endif
    char unused_name[32];
    if (!store.load("_main", self_id, unused_name, sizeof(unused_name))) {
      Serial.println("No identity yet: generating one...");
      self_id = radio_new_identity();
      int count = 0;
      while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {   // reserved hashes
        self_id = radio_new_identity(); count++;
      }
      store.save("_main", self_id);
    }
    keyHex(self_key, self_id.pub_key);

    loadPrefs();
    loadContacts();
    loadTargets();
    scanLog();

    // sequence numbers stay unique across reboots (skip ahead one block)
    seq = _prefs.seq_base + SEQ_BLOCK;
    _prefs.seq_base = seq;
    savePrefs();

    // first automatic cycle: give Wi-Fi/NTP (or GPS) a minute so rows carry a real time
    // plus random jitter, so two households powered up together don't ping in lock-step
    next_cycle_at = millis() + (_prefs.wifi_ssid[0] ? 60000 : 15000) + getRNG()->nextInt(0, 30000);
    next_flood_advert_at = millis() + ADVERT_FLOOD_INTERVAL_MS;
  }

  void beginNetwork() {
  #if HAS_WIFI
    beginWifi();
  #endif
  }

  void showWelcome() {
    Serial.println();
    Serial.println("===== MeshCore range_test =====");
    Serial.println(FIRMWARE_VER_TEXT);
    Serial.printf("Board: %s\n", board.getManufacturerName());
    Serial.printf("Node: %s (id %u)  role: %s  mode: %s\n", _prefs.node_name, (unsigned)_prefs.node_id,
                  roleName(_prefs.role), modeName(_prefs.mode));
    Serial.print("Public key: "); mesh::Utils::printHex(Serial, self_id.pub_key, PUB_KEY_SIZE); Serial.println();
    Serial.printf("Radio: %.3f MHz BW %.1f SF%d CR%d\n", (double)LORA_FREQ, (double)LORA_BW, LORA_SF, LORA_CR);
    printCard();
    Serial.println("(enter 'help' for commands)");
    Serial.println();
  }

  void printCard() {
    auto pkt = createSelfAdvert(_prefs.node_name);
    if (pkt) {
      uint8_t len = pkt->writeTo(tmp_buf);
      releasePacket(pkt);
      mesh::Utils::toHex(hex_buf, tmp_buf, len);
      Serial.println("Business card (import this on the other nodes):");
      Serial.print("meshcore://"); Serial.println(hex_buf);
    }
  }

  // ContactVisitor
  void onContactVisit(const ContactInfo& contact) override {
    bool fav = (contact.flags & 0x01) != 0;
    char k[9];
    keyHex(k, contact.id.pub_key);
    Serial.printf("   %s%s [%s] type=%d - ", contact.name, fav ? " *" : "", k, (int)contact.type);
    char tmp[40];
    int32_t secs = contact.last_advert_timestamp - getRTCClock()->getCurrentTime();
    AdvertTimeHelper::formatRelativeTimeDiff(tmp, secs, false);
    Serial.println(tmp);
  }

  void printStatus() {
    Position p = getPosition();
    Serial.printf("   board: %s\n", board.getManufacturerName());
    Serial.printf("   node: %s (id %u, key %s)  role=%s mode=%s repeat=%s\n", _prefs.node_name,
                  (unsigned)_prefs.node_id, self_key, roleName(_prefs.role), modeName(_prefs.mode), _prefs.repeat ? "on" : "off");
    Serial.printf("   position: %s", fixName(p.src));
    if (p.src) Serial.printf(" %.6f, %.6f alt %dm", p.lat, p.lon, p.alt);
    if (gpsPresent()) Serial.printf("  (GPS sats=%d)", p.sats);
    Serial.printf("\n   clock: %s (%lu)\n", nowUnix() ? "synced" : "NOT synced", (unsigned long)getRTCClock()->getCurrentTime());
    Serial.printf("   send interval: %lus  next cycle in %lds  seq=%lu\n", (unsigned long)_prefs.send_interval_secs,
                  secsToNextCycle(), (unsigned long)cycle_seq);
    for (uint8_t i = 0; i < target_count; i++) {
      char nm[32];
      getTargetName(i, nm, sizeof(nm));
      static const char* rn[] = {"-", "pending", "ok", "no ack", "send fail", "no contact", "skipped"};
      Serial.printf("   target %d: %s\n", i + 1, nm);
      for (uint8_t k = 0; k < 2; k++) {
        const LegStat& ts = tstat[i].leg[k];
        if (ts.attempts == 0 && ts.result == RES_NONE) continue;
        Serial.printf("      %s: last=%s  ok %lu/%lu", k == LEG_ZH ? "direct" : "routed", rn[ts.result],
                      (unsigned long)ts.oks, (unsigned long)ts.attempts);
        if (ts.result == RES_OK) {
          Serial.printf("  ack snr=%.2f rssi=%d rtt=%lums", ts.ack_snr, ts.ack_rssi, (unsigned long)ts.rtt_ms);
          if (k == LEG_RT) Serial.printf(" hops=%d", ts.hops);
        }
        Serial.println();
      }
    }
    if (rx_count) {
      Serial.printf("   received: %lu  last from %s %lus ago  snr=%.2f rssi=%.0f hops=%d (%s)\n", (unsigned long)rx_count,
                    rx_from, (millis() - rx_at) / 1000, rx_snr, rx_rssi, rx_hops, rx_mode);
    }
    Serial.printf("   log: %s, %lu bytes%s\n", _prefs.log_enabled ? "on" : "off", (unsigned long)log_bytes, log_full ? " (FULL)" : "");
  #if HAS_WIFI
    Serial.printf("   wifi: %s (ssid '%s')\n", wifi_status, _prefs.wifi_ssid);
    Serial.printf("   bayou: %s  feed=%s  queue=%d  ok=%lu fail=%lu dropped=%lu\n", post_status,
                  _prefs.bayou_public_key[0] ? _prefs.bayou_public_key : "(not set)", q_count,
                  (unsigned long)post_ok, (unsigned long)post_fail, (unsigned long)post_dropped);
  #endif
    Serial.printf("   contacts: %d/%d\n", getNumContacts(), MAX_CONTACTS);
  }

  void printHelp() {
    Serial.println("Commands:");
    Serial.println("   status                       - role, position, targets, last results, wifi/bayou");
    Serial.println("   card                         - print this node's business card");
    Serial.println("   import <meshcore://...>      - import another node's card (auto-favourited)");
    Serial.println("   list {n}                     - list contacts (* = favourite)");
    Serial.println("   favourite|unfavourite <name> - protect / unprotect a contact from eviction");
    Serial.println("   purge                        - delete all contacts");
    Serial.println("   target                       - show targets");
    Serial.println("   target add|remove <name>     - add/remove a target (max 4)");
    Serial.println("   target clear                 - remove all targets");
    Serial.println("   send                         - run one send cycle now");
    Serial.println("   nbr                          - repeaters heard directly (since last summary)");
    Serial.println("   advert [flood]               - send advert (zero hop, or flood)");
    Serial.println("   log | log dump | log clear   - flash log size / print all / erase");
    Serial.println("   set role sender|receiver     - apply role defaults (see README)");
    Serial.println("   set name <name>              - node name (reboot to re-advertise)");
    Serial.println("   set node_id <n>              - numeric id used on Bayou (1-65535)");
    Serial.println("   set interval <secs>          - send interval (0 = only on 'send'/button)");
    Serial.println("   set mode direct|routed|both  - zero-hop only / routed DM / both each round");
    Serial.println("                                  (both: the routed leg always floods, to discover");
    Serial.println("                                  which repeaters carry it from where you are)");
    Serial.println("   set repeat on|off            - forward other nodes' packets");
    Serial.println("   set log on|off               - log send/receive events to flash");
    Serial.println("   set display on|auto          - keep OLED on, or auto-off after 60 s");
    Serial.println("   set lat <deg> / set lon <deg> / set alt <m>  - fixed position (used without GPS fix)");
    Serial.println("   set pos gps                  - store the current GPS fix as the fixed position");
    Serial.println("   set pos clear                - forget the fixed position");
  #if HAS_WIFI
    Serial.println("   set wifi_ssid <ssid> / set wifi_password <pwd>  (reboot to apply)");
    Serial.println("   set bayou_public_key <key> / set bayou_private_key <key>");
    Serial.println("   set status_interval <secs>   - how often to post a status row");
  #endif
    Serial.println("   reboot | ver | help");
  }

  void handleSet(const char* config) {
    if (memcmp(config, "role ", 5) == 0) {
      const char* r = &config[5];
      if (strcmp(r, "sender") == 0) applyRoleDefaults(ROLE_SENDER);
      else if (strcmp(r, "receiver") == 0) applyRoleDefaults(ROLE_RECEIVER);
      else { Serial.println("   ERROR: role must be sender or receiver"); return; }
      savePrefs();
      next_cycle_at = millis() + 5000;
      Serial.printf("   OK, role=%s: mode=%s interval=%lus repeat=%s log=%s display=%s\n", roleName(_prefs.role),
                    modeName(_prefs.mode), (unsigned long)_prefs.send_interval_secs, _prefs.repeat ? "on" : "off",
                    _prefs.log_enabled ? "on" : "off", _prefs.display_always_on ? "on" : "auto");
    } else if (memcmp(config, "name ", 5) == 0) {
      StrHelper::strncpy(_prefs.node_name, &config[5], sizeof(_prefs.node_name));
      savePrefs();
      Serial.printf("   OK, name = %s (send 'advert flood' or reboot so others learn it)\n", _prefs.node_name);
    } else if (memcmp(config, "node_id ", 8) == 0) {
      long id = atol(&config[8]);
      if (id < 1 || id > 65535) { Serial.println("   ERROR: 1-65535"); return; }
      _prefs.node_id = (uint16_t)id;
      savePrefs();
      Serial.printf("   OK, node_id = %u\n", (unsigned)_prefs.node_id);
    } else if (memcmp(config, "interval ", 9) == 0) {
      long s = atol(&config[9]);
      if (s != 0 && s < 10) { Serial.println("   ERROR: 0 (off) or >= 10 seconds"); return; }
      _prefs.send_interval_secs = (uint32_t)s;
      savePrefs();
      next_cycle_at = millis() + 2000;
      Serial.printf("   OK, interval = %lus\n", (unsigned long)_prefs.send_interval_secs);
    } else if (memcmp(config, "mode ", 5) == 0) {
      const char* m = &config[5];
      if (strcmp(m, "direct") == 0) _prefs.mode = MODE_DIRECT;
      else if (strcmp(m, "routed") == 0) _prefs.mode = MODE_ROUTED;
      else if (strcmp(m, "both") == 0) _prefs.mode = MODE_BOTH;
      else { Serial.println("   ERROR: mode must be direct, routed or both"); return; }
      savePrefs();
      Serial.printf("   OK, mode = %s\n", modeName(_prefs.mode));
    } else if (memcmp(config, "repeat ", 7) == 0) {
      _prefs.repeat = strcmp(&config[7], "on") == 0 ? 1 : 0;
      savePrefs();
      Serial.printf("   OK, repeat = %s\n", _prefs.repeat ? "on" : "off");
    } else if (memcmp(config, "log ", 4) == 0) {
      _prefs.log_enabled = strcmp(&config[4], "on") == 0 ? 1 : 0;
      savePrefs();
      Serial.printf("   OK, log = %s\n", _prefs.log_enabled ? "on" : "off");
    } else if (memcmp(config, "display ", 8) == 0) {
      _prefs.display_always_on = strcmp(&config[8], "on") == 0 ? 1 : 0;
      savePrefs();
      Serial.printf("   OK, display = %s\n", _prefs.display_always_on ? "always on" : "auto-off");
    } else if (memcmp(config, "lat ", 4) == 0) {
      _prefs.fixed_lat_e6 = (int32_t)lround(atof(&config[4]) * 1000000.0);
      _prefs.has_fixed_pos = 1;
      savePrefs();
      Serial.printf("   OK, fixed lat = %.6f\n", _prefs.fixed_lat_e6 / 1000000.0);
    } else if (memcmp(config, "lon ", 4) == 0) {
      _prefs.fixed_lon_e6 = (int32_t)lround(atof(&config[4]) * 1000000.0);
      _prefs.has_fixed_pos = 1;
      savePrefs();
      Serial.printf("   OK, fixed lon = %.6f\n", _prefs.fixed_lon_e6 / 1000000.0);
    } else if (memcmp(config, "alt ", 4) == 0) {
      _prefs.fixed_alt_m = (int16_t)atoi(&config[4]);
      savePrefs();
      Serial.printf("   OK, fixed alt = %dm\n", (int)_prefs.fixed_alt_m);
    } else if (strcmp(config, "pos gps") == 0) {   // store the current GPS fix as the configured position
      Position p = getPosition();
      if (p.src != POS_GPS) { Serial.println("   ERROR: no live GPS fix right now (see 'status')"); return; }
      _prefs.fixed_lat_e6 = (int32_t)lround(p.lat * 1000000.0);
      _prefs.fixed_lon_e6 = (int32_t)lround(p.lon * 1000000.0);
      _prefs.fixed_alt_m = (int16_t)p.alt;
      _prefs.has_fixed_pos = 1;
      savePrefs();
      Serial.printf("   OK, configured position = GPS fix %.6f, %.6f alt %dm\n", p.lat, p.lon, p.alt);
    } else if (strcmp(config, "pos clear") == 0) {
      _prefs.has_fixed_pos = 0;
      _prefs.fixed_lat_e6 = _prefs.fixed_lon_e6 = 0;
      _prefs.fixed_alt_m = 0;
      savePrefs();
      Serial.println("   OK, fixed position cleared");
  #if HAS_WIFI
    } else if (memcmp(config, "wifi_ssid ", 10) == 0) {
      StrHelper::strncpy(_prefs.wifi_ssid, &config[10], sizeof(_prefs.wifi_ssid));
      savePrefs();
      Serial.printf("   OK, wifi_ssid = %s (reboot to apply)\n", _prefs.wifi_ssid);
    } else if (memcmp(config, "wifi_password ", 14) == 0) {
      StrHelper::strncpy(_prefs.wifi_password, &config[14], sizeof(_prefs.wifi_password));
      savePrefs();
      Serial.println("   OK, wifi_password set (reboot to apply)");
    } else if (memcmp(config, "bayou_public_key ", 17) == 0) {
      StrHelper::strncpy(_prefs.bayou_public_key, &config[17], sizeof(_prefs.bayou_public_key));
      savePrefs();
      Serial.printf("   OK, bayou_public_key = %s\n", _prefs.bayou_public_key);
    } else if (memcmp(config, "bayou_private_key ", 18) == 0) {
      StrHelper::strncpy(_prefs.bayou_private_key, &config[18], sizeof(_prefs.bayou_private_key));
      savePrefs();
      Serial.println("   OK, bayou_private_key set");
    } else if (memcmp(config, "status_interval ", 16) == 0) {
      long s = atol(&config[16]);
      if (s < 60) { Serial.println("   ERROR: >= 60 seconds"); return; }
      _prefs.status_interval_secs = (uint32_t)s;
      savePrefs();
      next_status_at = millis() + 5000;
      Serial.printf("   OK, status_interval = %lus\n", (unsigned long)_prefs.status_interval_secs);
  #endif
    } else {
      Serial.printf("   ERROR: unknown setting: %s\n", config);
    }
  }

  void handleTarget(const char* arg) {   // arg points after "target"
    while (*arg == ' ') arg++;
    if (*arg == 0) {
      if (target_count == 0) { Serial.println("   No targets"); return; }
      for (uint8_t i = 0; i < target_count; i++) {
        char nm[32];
        getTargetName(i, nm, sizeof(nm));
        Serial.printf("   %d: %s\n", i + 1, nm);
      }
    } else if (strcmp(arg, "clear") == 0) {
      target_count = 0;
      saveTargets();
      Serial.println("   OK, targets cleared");
    } else if (memcmp(arg, "add ", 4) == 0 || memcmp(arg, "remove ", 7) == 0) {
      bool add = arg[0] == 'a';
      const char* name = add ? &arg[4] : &arg[7];
      ContactInfo* c = searchContactsByPrefix(name);
      if (!c) { Serial.printf("   ERROR: no contact matching '%s' (see 'list')\n", name); return; }
      if (add) {
        if (addTarget(c)) Serial.printf("   OK, target added: %s (%d/%d)\n", c->name, target_count, MAX_TARGETS);
        else Serial.println("   ERROR: max targets reached");
      } else {
        if (removeTargetAt(findTargetIndex(c->id.pub_key))) Serial.printf("   OK, removed %s\n", c->name);
        else Serial.println("   ERROR: not a target");
      }
    } else {
      Serial.println("   usage: target | target add <name> | target remove <name> | target clear");
    }
  }

  void handleCommand(const char* cmd) {
    while (*cmd == ' ') cmd++;
    if (strcmp(cmd, "status") == 0) {
      printStatus();
    } else if (strcmp(cmd, "card") == 0) {
      printCard();
    } else if (memcmp(cmd, "import ", 7) == 0) {
      importCard(&cmd[7]);
    } else if (memcmp(cmd, "list", 4) == 0) {
      int n = (cmd[4] == ' ') ? atoi(&cmd[5]) : 0;
      Serial.printf("Contacts: %d/%d\n", getNumContacts(), MAX_CONTACTS);
      scanRecentContacts(n, this);
    } else if (memcmp(cmd, "favourite ", 10) == 0 || memcmp(cmd, "unfavourite ", 12) == 0) {
      bool fav = cmd[0] == 'f';
      ContactInfo* c = searchContactsByPrefix(fav ? &cmd[10] : &cmd[12]);
      if (!c) { Serial.println("   contact not found"); return; }
      if (fav) c->flags |= 0x01; else c->flags &= ~0x01;
      markContactsDirty();
      Serial.printf("   %s: %s\n", c->name, fav ? "favourite" : "not favourite");
    } else if (strcmp(cmd, "purge") == 0) {
      resetContacts();
      saveContacts();
      Serial.println("   all contacts deleted");
    } else if (memcmp(cmd, "target", 6) == 0 && (cmd[6] == 0 || cmd[6] == ' ')) {
      handleTarget(&cmd[6]);
    } else if (strcmp(cmd, "send") == 0) {
      sendNow();
    } else if (strcmp(cmd, "nbr") == 0) {
      printNeighbours();
    } else if (strcmp(cmd, "advert") == 0) {
      sendAdvert(false);
    } else if (strcmp(cmd, "advert flood") == 0) {
      sendAdvert(true);
    } else if (strcmp(cmd, "log") == 0) {
      Serial.printf("   log %s: %lu lines, %lu bytes (max %lu)%s\n", _prefs.log_enabled ? "on" : "off",
                    (unsigned long)countLogLines(), (unsigned long)log_bytes, (unsigned long)LOG_MAX_BYTES, log_full ? " FULL" : "");
    } else if (strcmp(cmd, "log dump") == 0) {
      dumpLog();
    } else if (strcmp(cmd, "log clear") == 0) {
      clearLog();
      Serial.println("   log cleared");
    } else if (memcmp(cmd, "set ", 4) == 0) {
      handleSet(&cmd[4]);
    } else if (strcmp(cmd, "reboot") == 0) {
      if (contacts_save_at) saveContacts();
      board.reboot();
    } else if (strcmp(cmd, "ver") == 0) {
      Serial.println(FIRMWARE_VER_TEXT);
    } else if (strcmp(cmd, "help") == 0) {
      printHelp();
    } else {
      Serial.printf("   ERROR: unknown command: %s (try 'help')\n", cmd);
    }
  }

  void loop() {
    BaseChatMesh::loop();
    serviceSender();

  #if HAS_WIFI
    serviceWifi();
    serviceStatus();
    serviceBayou();
  #endif

    unsigned long now = millis();
    if (contacts_save_at && (long)(now - contacts_save_at) >= 0) {
      saveContacts();
      contacts_save_at = 0;
    }
    if ((long)(now - next_flood_advert_at) >= 0) {
      next_flood_advert_at = now + ADVERT_FLOOD_INTERVAL_MS;
      sendAdvert(true);
    }

    // serial CLI
    int len = strlen(command);
    while (Serial.available() && len < (int)sizeof(command) - 1) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') {
        if (len > 0) {
          Serial.println();
          command[len] = 0;
          handleCommand(command);
          command[0] = 0;
          len = 0;
        }
        continue;
      }
      command[len++] = c;
      command[len] = 0;
      Serial.print(c);
    }
    if (len >= (int)sizeof(command) - 1) {
      command[len] = 0;
      Serial.println();
      handleCommand(command);
      command[0] = 0;
    }
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
RangeMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables);

/* ---------------------------------- DISPLAY & BUTTON ---------------------------------- */

#ifdef DISPLAY_CLASS
enum PageId { PG_MAIN = 0, PG_SEND, PG_REPEAT, PG_ADVERT };
static const uint8_t sender_pages[]   = { PG_MAIN, PG_SEND, PG_REPEAT, PG_ADVERT };
static const uint8_t receiver_pages[] = { PG_MAIN, PG_REPEAT, PG_ADVERT };

static uint8_t page_idx = 0;
static unsigned long next_display_refresh = 0;
static unsigned long last_button_activity = 0;
static char alert_text[32] = {0};
static unsigned long alert_expiry = 0;

static uint8_t pageCount() {
  return the_mesh.getRole() == ROLE_SENDER ? sizeof(sender_pages) : sizeof(receiver_pages);
}
static uint8_t currentPage() {
  if (page_idx >= pageCount()) page_idx = 0;
  return the_mesh.getRole() == ROLE_SENDER ? sender_pages[page_idx] : receiver_pages[page_idx];
}

static void showAlert(const char* text) {
  strncpy(alert_text, text, sizeof(alert_text) - 1);
  alert_text[sizeof(alert_text) - 1] = 0;
  alert_expiry = millis() + ALERT_DURATION_MS;
}

static void fmtAge(char* buf, size_t sz, unsigned long since) {
  unsigned long s = (millis() - since) / 1000UL;
  if (s < 100) snprintf(buf, sz, "%lus", s);
  else if (s < 6000) snprintf(buf, sz, "%lum", s / 60);
  else snprintf(buf, sz, "%luh", s / 3600);
}

static void drawLine(DisplayDriver& d, int row, const char* text) {
  d.setCursor(0, 10 + row * 9);
  d.print(text);
}

static void drawPageDots(DisplayDriver& d) {
  int n = pageCount();
  int x = d.width() / 2 - 5 * (n - 1);
  for (int i = 0; i < n; i++, x += 10) {
    if (i == page_idx) d.fillRect(x - 1, 1, 3, 3);
    else d.fillRect(x, 2, 1, 1);
  }
}

// Compact leg result for the OLED: direct "+12" (ACK SNR) / routed "2h" (hops); "x" no ACK,
// ".." in flight, "w" waiting on the flood rate limit, "!" send failed, "?" no contact, "-" none.
static void legText(char* out, size_t sz, const LegStat& ls, bool routed) {
  switch (ls.result) {
    case RES_OK:
      if (routed) { if (ls.hops >= 0) snprintf(out, sz, "%dh", ls.hops); else snprintf(out, sz, "ok"); }
      else snprintf(out, sz, "%+.0f", ls.ack_snr);
      break;
    case RES_PENDING:   snprintf(out, sz, ".."); break;
    case RES_TIMEOUT:   snprintf(out, sz, "x"); break;
    case RES_SKIPPED:   snprintf(out, sz, "w"); break;
    case RES_FAIL:      snprintf(out, sz, "!"); break;
    case RES_NOCONTACT: snprintf(out, sz, "?"); break;
    default:            snprintf(out, sz, "-"); break;
  }
}

static void renderSenderMain(DisplayDriver& d) {
  char buf[40], nm[12], age[8];
  Position p = the_mesh.currentPosition();
  snprintf(buf, sizeof(buf), "%.10s #%lu%s", the_mesh.getNodeName(), (unsigned long)the_mesh.getSeq(),
           the_mesh.isRepeating() ? " R" : "");
  drawLine(d, 0, buf);
  if (p.src == POS_GPS)          snprintf(buf, sizeof(buf), "GPS fix %dsat", p.sats);
  else if (p.src == POS_LAST)    snprintf(buf, sizeof(buf), "GPS lost: last fix");
  else if (the_mesh.hasGps())    snprintf(buf, sizeof(buf), "GPS no fix (%dsat)%s", p.sats, p.src == POS_MANUAL ? " M" : "");
  else if (p.src == POS_MANUAL)  snprintf(buf, sizeof(buf), "fixed position");
  else                           snprintf(buf, sizeof(buf), "NO POSITION");
  drawLine(d, 1, buf);

  int n = the_mesh.getTargetCount();
  if (n == 0) {
    drawLine(d, 2, "no targets:");
    drawLine(d, 3, "target add <name>");
  }
  // one line per house: "Waltha D+12 R2h 40s" = direct ok at +12 dB, routed ok over 2 hops
  for (int i = 0; i < n && i < 3; i++) {
    const TargetStat& ts = the_mesh.getTargetStat(i);
    char tn[32], dz[8], rt[8];
    the_mesh.getTargetName(i, tn, sizeof(tn));
    snprintf(nm, sizeof(nm), "%.6s", tn);
    legText(dz, sizeof(dz), ts.leg[LEG_ZH], false);
    legText(rt, sizeof(rt), ts.leg[LEG_RT], true);
    unsigned long at = ts.leg[LEG_ZH].at_millis;
    if ((long)(ts.leg[LEG_RT].at_millis - at) > 0) at = ts.leg[LEG_RT].at_millis;
    if (at) fmtAge(age, sizeof(age), at); else age[0] = 0;
    snprintf(buf, sizeof(buf), "%-6s D%-3s R%-3s %s", nm, dz, rt, age);
    drawLine(d, 2 + i, buf);
  }
  long nx = the_mesh.secsToNextCycle();
  if (the_mesh.isLogFull()) snprintf(buf, sizeof(buf), "LOG FULL!");
  else if (nx >= 0)         snprintf(buf, sizeof(buf), "next %lds log %luK", nx, (unsigned long)the_mesh.getLogKB());
  else                      snprintf(buf, sizeof(buf), "manual  log %luK", (unsigned long)the_mesh.getLogKB());
  drawLine(d, 5, buf);
}

static void renderReceiverMain(DisplayDriver& d) {
  char buf[40], age[8];
  snprintf(buf, sizeof(buf), "%.14s RECV%s", the_mesh.getNodeName(), the_mesh.isRepeating() ? " R" : "");
  drawLine(d, 0, buf);
#if HAS_WIFI
  if (!the_mesh.wifiConfigured()) snprintf(buf, sizeof(buf), "WiFi: not set");
  else snprintf(buf, sizeof(buf), "WiFi %s", the_mesh.getWifiStatus());
  drawLine(d, 1, buf);
  snprintf(buf, sizeof(buf), "Bayou %s q%d nb%d", the_mesh.getPostStatus(), the_mesh.getQueueLen(),
           the_mesh.getNbrCount());
  drawLine(d, 2, buf);
#else
  drawLine(d, 1, "(no WiFi on board)");
#endif
  if (the_mesh.getRxCount() > 0) {
    fmtAge(age, sizeof(age), the_mesh.getRxAt());
    snprintf(buf, sizeof(buf), "RX#%lu %.8s %s", (unsigned long)the_mesh.getRxCount(), the_mesh.getRxFrom(), age);
    drawLine(d, 3, buf);
    snprintf(buf, sizeof(buf), "SNR %.1f RSSI %.0f", the_mesh.getRxSnr(), the_mesh.getRxRssi());
    drawLine(d, 4, buf);
    snprintf(buf, sizeof(buf), "hops %d (%s)", the_mesh.getRxHops(), the_mesh.getRxMode());
    drawLine(d, 5, buf);
  } else {
    drawLine(d, 3, "waiting for sender...");
  }
}

static void renderDisplay() {
  if (!display.isOn()) return;
  if (millis() < next_display_refresh) return;
  display.startFrame();
  display.setTextSize(1);
  display.setColor(DisplayDriver::LIGHT);
  drawPageDots(display);
  int cx = display.width() / 2;
  switch (currentPage()) {
    case PG_MAIN:
      if (the_mesh.getRole() == ROLE_SENDER) renderSenderMain(display); else renderReceiverMain(display);
      break;
    case PG_SEND:
      display.drawTextCentered(cx, 18, "Send now");
      display.drawTextCentered(cx, 34, the_mesh.isBusy() ? "(cycle running)" : "long press");
      break;
    case PG_REPEAT:
      display.drawTextCentered(cx, 18, the_mesh.isRepeating() ? "Repeat: ON" : "Repeat: OFF");
      display.drawTextCentered(cx, 34, "long press = toggle");
      break;
    case PG_ADVERT:
      display.drawTextCentered(cx, 18, "Send advert");
      display.drawTextCentered(cx, 34, "long press");
      break;
  }
  if (millis() < alert_expiry) {
    int y = display.height() / 3;
    int p = display.height() / 16;
    display.setColor(DisplayDriver::DARK);
    display.fillRect(p, y, display.width() - p*2, y);
    display.setColor(DisplayDriver::LIGHT);
    display.drawRect(p, y, display.width() - p*2, y);
    display.drawTextCentered(cx, y + p + 6, alert_text);
  }
  display.endFrame();
  next_display_refresh = millis() + DISPLAY_REFRESH_MS;
}

static void handleButtonEvents() {
  int ev = user_btn.check();
  if (ev == BUTTON_EVENT_NONE) return;
  last_button_activity = millis();
  if (!display.isOn()) {
    display.turnOn();
    next_display_refresh = 0;
    return;
  }
  if (ev == BUTTON_EVENT_CLICK) {
    page_idx = (page_idx + 1) % pageCount();
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    switch (currentPage()) {
      case PG_MAIN:
      case PG_SEND:
        if (the_mesh.getRole() == ROLE_SENDER) {
          if (the_mesh.getTargetCount() == 0) showAlert("No targets");
          else if (the_mesh.sendNow()) { page_idx = 0; showAlert("Sending"); }
          else showAlert("Busy");
        }
        break;
      case PG_REPEAT:
        the_mesh.toggleRepeat();
        showAlert(the_mesh.isRepeating() ? "Repeat ON" : "Repeat OFF");
        break;
      case PG_ADVERT:
        showAlert(the_mesh.sendAdvert(false) ? "Advert sent" : "Advert failed");
        break;
    }
  }
  next_display_refresh = 0;
}

static void displayLoop() {
  handleButtonEvents();
  renderDisplay();
  if (!the_mesh.displayAlwaysOn() && display.isOn() && last_button_activity > 0 &&
      (millis() - last_button_activity) > DISPLAY_AUTO_OFF_MS) {
    display.turnOff();
  }
}
#endif  // DISPLAY_CLASS

/* ---------------------------------- SETUP & LOOP -------------------------------------- */

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  board.begin();

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("range_test starting");
    display.endFrame();
  }
#endif

  if (!radio_init()) {
    Serial.println("radio_init() FAILED - halting");
    halt();
  }
  fast_rng.begin(radio_get_rng_seed());

#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  the_mesh.begin(InternalFS);
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  the_mesh.begin(LittleFS);
#elif defined(ESP32)
  SPIFFS.begin(true);
  the_mesh.begin(SPIFFS);
#else
  #error "need to define filesystem"
#endif

  radio_set_params(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
  radio_set_tx_power(LORA_TX_POWER);

  sensors.begin();
#if ENV_INCLUDE_GPS
  // EnvironmentSensorManager powers the GPS down after detection unless asked to keep it on
  if (sensors.setSettingValue("gps", "1")) Serial.println("GPS detected, enabled");
  else Serial.println("No GPS detected");
#endif

  the_mesh.beginNetwork();
  the_mesh.showWelcome();

#ifdef DISPLAY_CLASS
  last_button_activity = millis();
#endif

#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendAdvert(true);
#endif
}

void loop() {
  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  displayLoop();
#endif
  rtc_clock.tick();
}
