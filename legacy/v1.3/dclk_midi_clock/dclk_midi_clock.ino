// ********************* //
// DCLK                  //
// Dual MIDI Clock       //
// firmware version 1.3  //
// hardware version: 1.0 //
// powered by STM32 BP   //
// ********************* //
// controller by         //
// .axs instruments      //
// ********************* //
// Description: Dual MIDI clock with two TRS (Type A)
//              MIDI outputs and USB MIDI, quantised
//              start/stop and per-output clock division.
// ********************* //

// ********************* //
// BUILD SETTINGS
//
// Board:           "DCLK Dual MIDI Clock" (custom boards.txt entry)
// USB support:     None
// U(S)ART support: Enabled (generic 'Serial') - BusIO needs 'Serial' to exist
// ********************* //

// ********************* //
// MENUS
//
// LEDS menu:
//   LO / MID / HI - brightness of the CH1, CH2 and BEAT LEDs
//                   (base level + beat flash, see led_levels[] below)
//
// MIDI menu:
//   WAIT LEN  - bars between quantised starts (and stops, if STOP Q on)
//   STOP Q    - ON: stop lands on the next WAIT LEN boundary
//   CH1 DIV   - clock divider for output 1 (/1 /2 /3 /4 /6 /8)
//   CH2 DIV   - clock divider for output 2
//   USB OUT   - which output the USB MIDI clock follows
//   CLK SRC   - INT: internal tempo (encoder). USB: follow MIDI clock
//               coming in over USB (DAW, OP-XY...). In USB mode the
//               encoder doesn't set tempo and no clock is sent back to
//               USB (the host is the master).
//   EXT TPT   - USB mode only. HOST: follow the host's Start/Stop/
//               Continue/Song Position; the WAIT LEN grid is the host's
//               bar grid and armed outputs start/stop with the host.
//               FREE: ignore host transport, use the incoming clock as a
//               plain tempo source (for clock-only sources).
//
// Clock, Start and Stop are MIDI System Real-Time messages: they carry no
// MIDI channel and every device on the cable receives them. "CH1"/"CH2"
// here mean the two output jacks, not MIDI channels.
//
// SCREEN menu:
//   SAVER     - after OFF/1/5/10/30 min without input the OLED dims if a
//               clock is running, or switches off if everything is
//               stopped. When off, the BEAT LED breathes softly once per
//               bar at the current tempo to show the unit is on standby.
//
// MONITOR menu: USB / clock diagnostics (default font for legibility).
//   Encoder button toggles a USB test note (C4, ch 1, once a second).
//
// ABOUT menu: product, maker and firmware version (FW_VERSION below).
// ********************* //

// ********************* //
// WEB EDITOR (USB SysEx)
//
// Every setting can also be read and changed over USB MIDI SysEx by the
// DCLK web editor (Chrome / Edge, Web MIDI). Messages:
//   F0 7D 44 43 <cmd> <data...> F7     (7D = non-commercial ID, "DC")
// Host -> DCLK:
//   01                  GET_INFO   -> reply 41 <proto> <fw ascii...>
//   02                  GET_STATE  -> reply 42 (see sendState())
//   03 <param> <lo> <hi> SET        -> applies, replies 42
//   04 <out> <action>   TRANSPORT  out 0/1 (2 = both), action 0 toggle,
//                                  1 start, 2 stop -> replies 42
//   06 44 45 46         FACTORY RESET ("DEF") -> replies 42
// Values are 14-bit: lo = v & 0x7F, hi = v >> 7. Param ids: PARAM_* below.
// Changes are saved to flash with the usual rule (once all clocks stop).
// ********************* //

// ********************* //
// LICENSE & COPYRIGHT
//
// This project is released under the MIT License.
// Copyright (c) 2026 .axs instruments
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ********************* //

// -- LIBRARIES --
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/Picopixel.h>
#include <EEPROM.h>
#include <Adafruit_TinyUSB.h>

#if !defined(ARDUINO_DCLK)
  #warning "Board is not 'DCLK Dual MIDI Clock' - USB clock may not be a clean 48 MHz"
#endif

// ---------- Firmware version (shown on the ABOUT screen; keep in sync with the header) ----------
const char* const FW_VERSION = "1.3";

// ---------- Config ----------
constexpr uint8_t  PPQN       = 24;
constexpr uint16_t BPM_MIN = 30, BPM_MAX = 300;
constexpr uint16_t BPM_SAVE_DELAY_MS = 750;   // save BPM after this long without a change

// ---------- LED brightness ----------
// Each level: steady brightness while playing + extra flash on each beat
// (both as PWM out of 255). The BEAT LED peaks at base + flash.
struct LedLevel { uint8_t base; uint8_t flash; const char* name; };
constexpr LedLevel led_levels[] = {
  {  5,  25, "LO"  },   //  2% base, 10% flash
  { 25,  80, "MID" },   // 10% base, 31% flash
  { 80, 150, "HI"  },   // 31% base, 59% flash
};
constexpr uint8_t N_LED_LEVELS = sizeof(led_levels) / sizeof(led_levels[0]);

// ---------- Pins ----------
constexpr uint8_t PIN_ENC_A = PB4, PIN_ENC_B = PB5;
constexpr uint8_t PIN_BTN[2] = { PB0, PB1 };
constexpr uint8_t PIN_ENC_BTN      = PB13;  // encoder shaft push button
constexpr uint8_t PIN_BTN_SETTINGS = PB14;  // settings menu button

constexpr uint8_t PIN_LED_BEAT = PA6;                // TIM3_CH1 PWM
constexpr uint8_t PIN_LED_STATUS[2] = { PB8, PB9 };  // TIM4 PWM

HardwareSerial MidiOut1(PA10, PA9);  // USART1 (RX unused)
HardwareSerial MidiOut2(PA3, PA2);   // USART2 (RX unused)
HardwareSerial* midi[2] = { &MidiOut1, &MidiOut2 };

Adafruit_SSD1306 display(128, 32, &Wire, -1);
constexpr uint8_t DISPLAY_ROTATION = 2;   // 0 = normal, 2 = rotated 180 degrees
HardwareTimer* tickTimer = new HardwareTimer(TIM2);

// ---------- USB MIDI ----------
Adafruit_USBD_MIDI usb_midi;

// USB-MIDI 1.0 Code Index Numbers
constexpr uint8_t CIN_SINGLE_BYTE = 0x0F;   // real-time: F8 / FA / FC
constexpr uint8_t CIN_NOTE_OFF    = 0x08;
constexpr uint8_t CIN_NOTE_ON     = 0x09;

// ---------- USB TX ring buffer (ISR-safe SPSC: ISR produces, loop consumes) ----------
constexpr uint8_t USB_TX_RING_SIZE = 32;          // must be power of 2
volatile uint8_t usb_tx_ring[USB_TX_RING_SIZE];
volatile uint8_t usb_tx_head = 0;
volatile uint8_t usb_tx_tail = 0;

// Diagnostics (shown on the DIAG screen)
volatile uint32_t usb_ring_drops = 0;   // ISR found ring full
uint32_t usb_pkts_sent  = 0;            // packets accepted by TinyUSB
uint32_t usb_pkts_retry = 0;            // TinyUSB FIFO full, retried later

inline bool usb_tx_enqueue(uint8_t b) {
  uint8_t next = (usb_tx_head + 1) & (USB_TX_RING_SIZE - 1);
  if (next == usb_tx_tail) { usb_ring_drops++; return false; }   // full
  usb_tx_ring[usb_tx_head] = b;
  usb_tx_head = next;
  return true;
}

// ---------- SysEx (web editor) ----------
constexpr uint8_t SYSEX_MFR   = 0x7D;        // non-commercial / educational ID
constexpr uint8_t SYSEX_ID1   = 0x44;        // 'D'
constexpr uint8_t SYSEX_ID2   = 0x43;        // 'C'
constexpr uint8_t SYSEX_PROTO = 1;           // bump if the message layout changes

enum SysexCmd : uint8_t {
  CMD_GET_INFO = 0x01, CMD_GET_STATE = 0x02, CMD_SET = 0x03,
  CMD_TRANSPORT = 0x04, CMD_FACTORY_RESET = 0x06,
  RPL_INFO = 0x41, RPL_STATE = 0x42
};

enum Param : uint8_t {
  PARAM_BPM, PARAM_CLK_SRC, PARAM_WAIT, PARAM_STOPQ, PARAM_DIV1, PARAM_DIV2,
  PARAM_USB_OUT, PARAM_LED, PARAM_SAVER, PARAM_EXT_TPT, PARAM_COUNT
};

// Outgoing SysEx is queued as ready-made 4-byte USB-MIDI packets (loop
// context only) and sent after any pending clock bytes.
constexpr uint8_t SX_TX_PKTS = 32;           // power of 2; one STATE reply = 12
uint8_t sx_tx[SX_TX_PKTS][4];
uint8_t sx_tx_head = 0, sx_tx_tail = 0;

// Incoming SysEx assembly buffer
constexpr uint8_t SX_RX_MAX = 32;
uint8_t sx_rx[SX_RX_MAX];
uint8_t sx_rx_len   = 0;
bool    sx_rx_on    = false;
bool    sx_rx_ovf   = false;

// ---------- Options ----------
constexpr uint8_t wait_options[]  = {1, 2, 4, 8, 16};
constexpr uint8_t div_options[]   = {1, 2, 3, 4, 6, 8};   // all divide 96 (one bar)
constexpr uint8_t saver_options[] = {0, 1, 5, 10, 30};    // minutes, 0 = off
constexpr uint8_t N_WAIT  = sizeof(wait_options);
constexpr uint8_t N_DIV   = sizeof(div_options);
constexpr uint8_t N_SAVER = sizeof(saver_options);

// ---------- Persistent settings (emulated EEPROM in flash) ----------
// BPM sits after 'magic' so settings saved by older firmware stay valid
// (their BPM bytes read as erased flash and fall back to the default).
struct PersistedSettings {
  uint8_t led_level;
  uint8_t wait_bars;
  uint8_t usb_out_channel;
  uint8_t clk_div[2];
  uint8_t stop_quant;
  uint8_t saver_idx;
  uint8_t magic;
  uint8_t bpm_lo;
  uint8_t bpm_hi;
  uint8_t clk_src;     // v1.3+: erased (0xFF) on older saves -> INT
  uint8_t ext_tpt;     // v1.3+: erased -> HOST
};

constexpr uint16_t EEPROM_SETTINGS_ADDR = 0;
constexpr uint8_t  EEPROM_MAGIC         = 0xAD;

// ---------- State ----------
volatile uint16_t bpm = 120;
volatile bool bpm_changed = true;
volatile bool ui_dirty = true;

volatile bool running[2]    = { false, false };
volatile bool pending[2]    = { false, false };   // waiting to start on the grid
volatile bool stop_pend[2]  = { false, false };   // waiting to stop on the grid
volatile bool play_req[2]   = { false, false };
volatile bool stop_req[2]   = { false, false };
volatile uint32_t loop_tick = 0;
volatile uint32_t loop_ticks = 384; // dynamically calculated

// Tick-ISR-only state
uint32_t out_tick[2] = { 0, 0 };             // ticks since this output started

// Settings
uint8_t          led_level  = 1;       // MID
volatile uint8_t wait_bars  = 4;
volatile uint8_t clk_div[2] = { 1, 1 };
volatile bool    stop_quant = false;
uint8_t          saver_idx  = 2;       // 5 min

// USB routing. usb_out_channel is only ever changed inside the tick ISR;
// the UI posts a request via usb_out_req so Stop/Start can be sent cleanly.
volatile uint8_t usb_out_channel = 0;   // 0 = CH1, 1 = CH2
volatile int8_t  usb_out_req     = -1;  // -1 = no request, else new channel
volatile bool    usb_running     = false; // USB host has seen Start (and no Stop since)
volatile bool    usb_resync      = false; // host just (re)connected: re-send Start at next boundary

// Clock source. clk_src only changes in loop() via setClockSource(), with
// the tick timer paused, so processTick() never runs from two places.
enum ClockSource : uint8_t { CLK_INT = 0, CLK_USB = 1 };
volatile uint8_t clk_src     = CLK_INT;
volatile bool    ext_tpt_host = true;    // USB mode: follow host Start/Stop
volatile bool    host_running = false;   // host transport (USB mode, HOST)

// Incoming USB clock: tracked in every mode so the editor can show it
constexpr uint32_t EXT_PRESENT_MS = 500;     // no F8 for this long = no clock
constexpr uint32_t EXT_LOST_MS    = 2000;    // host running but silent = stop
uint32_t ext_last_ms   = 0;
bool     ext_seen      = false;
uint32_t ext_ts[PPQN];                       // F8 timestamps, one beat
uint8_t  ext_ts_idx    = 0;
uint8_t  ext_ts_count  = 0;
uint16_t ext_bpm_x10   = 0;                  // measured tempo, 0 = unknown
uint16_t shown_ext_bpm = 0;                  // what the main screen shows (hysteresis)

bool save_pending = false;

// BPM save debounce: armed on every BPM change, fires once the encoder
// has been idle for BPM_SAVE_DELAY_MS
bool     bpm_save_armed = false;
uint32_t bpm_changed_ms = 0;

// ---------- UI state (loop() context only) ----------
enum class Screen : uint8_t {
  MAIN, SETTINGS,
  LED_VIEW, LED_EDIT,
  MIDI_VIEW, MIDI_EDIT,
  SAVER_EDIT,
  DIAG,                        // MONITOR: USB diagnostics
  ABOUT
};
Screen  screen        = Screen::MAIN;
uint8_t menu_sel      = 0;     // SETTINGS: 0=LEDS 1=MIDI 2=SCREEN
uint8_t midi_menu_sel = 0;
uint8_t edit_idx      = 0;     // option index being edited (MIDI_EDIT / SAVER_EDIT)
uint8_t led_edit_orig = 0;     // LED level to restore on cancel

constexpr uint8_t N_SETTINGS = 5;
const char* const settings_labels[N_SETTINGS] = { "LEDS", "MIDI", "SCREEN", "MONITOR", "ABOUT" };
enum SettingsItem : uint8_t { SI_LEDS, SI_MIDI, SI_SCREEN, SI_MONITOR, SI_ABOUT };

bool test_note_enabled = false;   // MONITOR: encoder button toggles

enum MidiItem : uint8_t { MI_WAIT, MI_STOPQ, MI_DIV1, MI_DIV2, MI_USB, MI_SRC, MI_TPT, MI_COUNT };
const char* const midi_labels[MI_COUNT] = {
  "WAIT LEN", "STOP Q", "CH1 DIV", "CH2 DIV", "USB OUT", "CLK SRC", "EXT TPT"
};

// Screen saver
uint32_t last_activity = 0;
bool     screen_asleep = false;   // panel off (nothing running)
bool     screen_dimmed = false;   // panel dimmed (clock running)

volatile int16_t encoder_delta = 0;

// Display flush state (non-blocking, see displayService())
bool     disp_busy = false;
uint16_t disp_pos  = 0;

// ---------- Helpers ----------
inline bool extMode()     { return clk_src == CLK_USB; }
inline bool hostGrid()    { return clk_src == CLK_USB && ext_tpt_host; }
inline bool extPresent()  { return ext_seen && (millis() - ext_last_ms) < EXT_PRESENT_MS; }
void updateTiming() {
  constexpr uint32_t ticks_per_bar = PPQN * 4;
  loop_ticks = (uint32_t)wait_bars * ticks_per_bar;
  if (loop_ticks == 0) loop_ticks = 1;
}

uint8_t indexOf(const uint8_t* opts, uint8_t n, uint8_t v) {
  for (uint8_t i = 0; i < n; i++) if (opts[i] == v) return i;
  return 0;
}

uint8_t wrapAdd(uint8_t v, int16_t d, uint8_t n) {
  int16_t r = ((int16_t)v + d) % n;
  if (r < 0) r += n;
  return (uint8_t)r;
}

// Send a single byte to one physical DIN output. USB is handled separately.
void sendMIDIByte(uint8_t idx, uint8_t b) {
  midi[idx]->write(b);
}

void stopOutput(uint8_t i) {
  sendMIDIByte(i, 0xFC);   // MIDI Stop
  running[i]   = false;
  stop_pend[i] = false;
  ui_dirty = true;
}

// Play / stop requests that don't need a clock tick. In USB mode with the
// host stopped no ticks arrive, so loop() calls this directly to arm and
// disarm outputs. Only ever called from the context that runs processTick().
void processIdleRequests() {
  for (int i = 0; i < 2; i++) {
    if (stop_req[i]) {
      stop_req[i] = false;
      if (pending[i])      pending[i] = false;         // disarm
      else if (running[i]) stopOutput(i);              // safety: shouldn't happen
      ui_dirty = true;
    }
    if (play_req[i]) {
      play_req[i] = false;
      pending[i]  = true;                              // armed: starts with the host
      ui_dirty = true;
    }
  }
}

// ---------- Core tick (24 PPQN) ----------
// INT: called from the TIM2 ISR. USB: called from loop() once per incoming
// F8 (only while the host is running in HOST transport mode).
void processTick() {
  const bool ext  = extMode();
  const bool hgrd = hostGrid();

  // 1. Stop requests
  const bool on_grid = (loop_tick % loop_ticks == 0);
  for (int i = 0; i < 2; i++) {
    if (stop_req[i]) {
      stop_req[i] = false;
      if (pending[i]) {
        pending[i] = false;                 // cancel a waiting start
      } else if (running[i]) {
        if (stop_pend[i])    stop_pend[i] = false;   // press again: cancel quantised stop
        else if (stop_quant) stop_pend[i] = true;    // stop at next boundary
        else                 stopOutput(i);          // immediate stop
      }
      ui_dirty = true;
    }
  }
  // 1b. Quantised stops land exactly on the boundary, before that tick's clock
  for (int i = 0; i < 2; i++) {
    if (stop_pend[i] && running[i] && on_grid) stopOutput(i);
  }

  // 2. Play requests
  for (int i = 0; i < 2; i++) {
    if (play_req[i]) {
      play_req[i] = false;
      pending[i] = true;
      ui_dirty = true;
    }
  }

  // 3. Start pending outputs. Normally the grid restarts when nothing is
  // running; when following a host, the grid is the host's bar grid.
  bool any = running[0] || running[1];
  if (!hgrd && !any && (pending[0] || pending[1])) loop_tick = 0;
  const bool boundary = (!hgrd && !any) || (loop_tick % loop_ticks == 0);

  for (int i = 0; i < 2; i++) {
    if (pending[i] && boundary) {
      pending[i]  = false;
      running[i]  = true;
      out_tick[i] = 0;
      sendMIDIByte(i, 0xFA);   // MIDI Start
      ui_dirty = true;
    }
  }

  // 4. USB follows whichever output is selected
  if (usb_out_req >= 0) {
    usb_out_channel = (uint8_t)usb_out_req;
    usb_out_req = -1;
    ui_dirty = true;
  }
  if (usb_resync) {
    usb_resync  = false;
    usb_running = false;          // new host hasn't seen a Start yet
  }
  const uint8_t u = usb_out_channel;
  if (!ext) {   // USB clock out only when DCLK is the master
    // Selected output stopped (button, quantised stop, or switched) -> Stop
    if (usb_running && !running[u]) {
      usb_tx_enqueue(0xFC);
      usb_running = false;
    }
    // Selected output running but USB isn't -> Start on the bar grid.
    // Same tick as the DIN Start normally; after a channel switch or
    // reconnect it waits for the next boundary.
    if (!usb_running && running[u] && (loop_tick % loop_ticks == 0)) {
      usb_tx_enqueue(0xFA);
      usb_running = true;
    }
  }

  // 5. Clock, with per-output division
  bool clk[2];
  for (int i = 0; i < 2; i++) {
    clk[i] = running[i] && (out_tick[i] % clk_div[i] == 0);
    if (clk[i]) sendMIDIByte(i, 0xF8);
  }
  if (clk[u] && !ext) usb_tx_enqueue(0xF8);   // USB mirrors the selected output, divider included

  // 6. Advance counters
  for (int i = 0; i < 2; i++) if (running[i]) out_tick[i]++;
  if (hgrd || running[0] || running[1]) loop_tick++;
}

// ---------- Timer ISR ----------
void onTick() {
  processTick();
}

// ---------- Transport helpers (loop context) ----------
// Stop everything cleanly: used when the clock source or transport mode
// changes. Caller pauses the tick timer first in INT mode.
void stopAll() {
  for (int i = 0; i < 2; i++) {
    if (running[i]) sendMIDIByte(i, 0xFC);
    running[i] = pending[i] = stop_pend[i] = false;
    play_req[i] = stop_req[i] = false;
  }
  if (usb_running) { usb_tx_enqueue(0xFC); usb_running = false; }
  loop_tick = 0;
  host_running = false;
  ui_dirty = true;
}

void setClockSource(uint8_t src) {
  if (src > CLK_USB || src == clk_src) return;
  tickTimer->pause();
  stopAll();
  clk_src = src;
  if (src == CLK_INT) {
    applyTempo();
    tickTimer->resume();
  }
  save_pending = true;
}

void setExtTransport(bool host) {
  if (host == ext_tpt_host) return;
  if (extMode()) stopAll();       // ticks run in loop() in USB mode: safe
  ext_tpt_host = host;
  save_pending = true;
  ui_dirty = true;
}

// Host transport (USB mode, HOST). Outputs that were running go back to
// "armed" (pending) so they restart with the host on its next Start.
void hostStart(bool from_top) {
  if (from_top) {
    loop_tick = 0;
    for (int i = 0; i < 2; i++) {
      if (running[i]) { sendMIDIByte(i, 0xFA); out_tick[i] = 0; }   // host restarted mid-play
    }
  }
  host_running = true;
  ui_dirty = true;
}

void hostStop() {
  for (int i = 0; i < 2; i++) {
    if (running[i]) {
      sendMIDIByte(i, 0xFC);
      running[i]  = false;
      pending[i]  = !stop_pend[i];   // a stop that was already queued stays stopped
      stop_pend[i] = false;
    }
  }
  host_running = false;
  ui_dirty = true;
}

// ---------- Incoming USB clock ----------
void extClockTick() {
  uint32_t now = micros();
  uint32_t ms  = millis();
  if (!ext_seen || (ms - ext_last_ms) > EXT_PRESENT_MS) ext_ts_count = 0;   // stale
  ext_seen = true;
  ext_last_ms = ms;

  // Tempo over the last beat (24 ticks): loop() jitter averages out
  if (ext_ts_count == PPQN) {
    uint32_t span = now - ext_ts[ext_ts_idx];
    if (span) ext_bpm_x10 = (uint16_t)(600000000UL / span);
  } else {
    ext_ts_count++;
  }
  ext_ts[ext_ts_idx] = now;
  ext_ts_idx = (ext_ts_idx + 1) % PPQN;

  if (!extMode()) return;                    // INT mode: just measured
  if (hostGrid() && !host_running) return;   // host stopped: clock only
  processTick();
}

void realtimeIn(uint8_t b) {
  switch (b) {
    case 0xF8: extClockTick(); break;
    case 0xFA: if (hostGrid()) hostStart(true);  break;   // Start
    case 0xFB: if (hostGrid()) hostStart(false); break;   // Continue
    case 0xFC: if (hostGrid() && host_running) hostStop(); break;
  }
}

void songPosition(uint16_t sixteenths) {
  if (hostGrid() && !host_running) loop_tick = (uint32_t)sixteenths * (PPQN / 4);
}

// ---------- SysEx out ----------
bool sxQueuePacket(uint8_t cin, uint8_t b0, uint8_t b1, uint8_t b2) {
  uint8_t next = (sx_tx_head + 1) & (SX_TX_PKTS - 1);
  if (next == sx_tx_tail) return false;
  sx_tx[sx_tx_head][0] = cin;
  sx_tx[sx_tx_head][1] = b0;
  sx_tx[sx_tx_head][2] = b1;
  sx_tx[sx_tx_head][3] = b2;
  sx_tx_head = next;
  return true;
}

// msg is the complete message, F0 ... F7. Queued whole or not at all
// (the editor simply asks again).
void sendSysex(const uint8_t* m, uint8_t n) {
  if (!TinyUSBDevice.mounted() || n < 2) return;
  uint8_t used = (sx_tx_head - sx_tx_tail) & (SX_TX_PKTS - 1);
  uint8_t need = (n + 2) / 3;
  if ((uint8_t)(SX_TX_PKTS - 1 - used) < need) return;
  uint8_t i = 0;
  while (n - i > 3) { sxQueuePacket(0x04, m[i], m[i + 1], m[i + 2]); i += 3; }
  switch (n - i) {
    case 1: sxQueuePacket(0x05, m[i], 0, 0); break;
    case 2: sxQueuePacket(0x06, m[i], m[i + 1], 0); break;
    case 3: sxQueuePacket(0x07, m[i], m[i + 1], m[i + 2]); break;
  }
}

uint8_t midiGet(uint8_t item);   // defined with the MIDI menu

uint16_t paramValue(uint8_t p) {
  switch (p) {
    case PARAM_BPM:     return bpm;
    case PARAM_CLK_SRC: return clk_src;
    case PARAM_WAIT:    return wait_bars;
    case PARAM_STOPQ:   return stop_quant ? 1 : 0;
    case PARAM_DIV1:    return clk_div[0];
    case PARAM_DIV2:    return clk_div[1];
    case PARAM_USB_OUT: return midiGet(MI_USB);
    case PARAM_LED:     return led_level;
    case PARAM_SAVER:   return saver_options[saver_idx];
    case PARAM_EXT_TPT: return ext_tpt_host ? 1 : 0;
  }
  return 0;
}

void sendState() {
  uint8_t m[48];
  uint8_t n = 0;
  m[n++] = 0xF0; m[n++] = SYSEX_MFR; m[n++] = SYSEX_ID1; m[n++] = SYSEX_ID2;
  m[n++] = RPL_STATE;
  m[n++] = SYSEX_PROTO;
  for (uint8_t p = 0; p < PARAM_COUNT; p++) {
    uint16_t v = paramValue(p);
    m[n++] = v & 0x7F;
    m[n++] = (v >> 7) & 0x7F;
  }
  m[n++] = (running[0]   ? 0x01 : 0) | (running[1]   ? 0x02 : 0) |
           (pending[0]   ? 0x04 : 0) | (pending[1]   ? 0x08 : 0) |
           (stop_pend[0] ? 0x10 : 0) | (stop_pend[1] ? 0x20 : 0) |
           (save_pending ? 0x40 : 0);
  m[n++] = (extPresent() ? 0x01 : 0) | (host_running ? 0x02 : 0) |
           (usb_running  ? 0x04 : 0);
  uint16_t eb = extPresent() ? ext_bpm_x10 : 0;
  m[n++] = eb & 0x7F;
  m[n++] = (eb >> 7) & 0x7F;
  uint32_t lt = loop_tick;
  m[n++] = (uint8_t)((lt / PPQN) % 4);                 // beat in bar
  m[n++] = (uint8_t)((lt / (PPQN * 4)) % wait_bars);   // bar in WAIT LEN
  m[n++] = 0xF7;
  sendSysex(m, n);
}

void sendInfo() {
  uint8_t m[24];
  uint8_t n = 0;
  m[n++] = 0xF0; m[n++] = SYSEX_MFR; m[n++] = SYSEX_ID1; m[n++] = SYSEX_ID2;
  m[n++] = RPL_INFO;
  m[n++] = SYSEX_PROTO;
  for (const char* s = FW_VERSION; *s && n < sizeof(m) - 1; s++) m[n++] = (uint8_t)*s & 0x7F;
  m[n++] = 0xF7;
  sendSysex(m, n);
}

// ---------- SysEx in ----------
void requestPlayStop(uint8_t i, uint8_t action) {   // action: 0 toggle, 1 start, 2 stop
  bool active = running[i] || pending[i];
  bool start  = (action == 0) ? !active : (action == 1);
  if (start && !active)      play_req[i] = true;
  else if (!start && active) {
    // An explicit "stop" shouldn't cancel a quantised stop that's waiting
    if (!(action == 2 && stop_pend[i])) stop_req[i] = true;
  }
}

void midiApply(uint8_t item, uint8_t idx);   // defined with the MIDI menu
void factoryReset();

uint8_t findIndex(const uint8_t* opts, uint8_t n, uint16_t v) {
  for (uint8_t i = 0; i < n; i++) if (opts[i] == v) return i;
  return 0xFF;
}

void setParam(uint8_t p, uint16_t v) {
  uint8_t idx;
  switch (p) {
    case PARAM_BPM:
      if (v >= BPM_MIN && v <= BPM_MAX && v != bpm) {
        bpm = v;
        bpm_changed    = true;
        bpm_save_armed = true;
        bpm_changed_ms = millis();
      }
      break;
    case PARAM_CLK_SRC: setClockSource((uint8_t)v); break;
    case PARAM_WAIT:
      if ((idx = findIndex(wait_options, N_WAIT, v)) != 0xFF) midiApply(MI_WAIT, idx);
      break;
    case PARAM_STOPQ:   midiApply(MI_STOPQ, v ? 1 : 0); break;
    case PARAM_DIV1:
      if ((idx = findIndex(div_options, N_DIV, v)) != 0xFF) midiApply(MI_DIV1, idx);
      break;
    case PARAM_DIV2:
      if ((idx = findIndex(div_options, N_DIV, v)) != 0xFF) midiApply(MI_DIV2, idx);
      break;
    case PARAM_USB_OUT: if (v <= 1) midiApply(MI_USB, (uint8_t)v); break;
    case PARAM_LED:
      if (v < N_LED_LEVELS && v != led_level) { led_level = v; save_pending = true; }
      break;
    case PARAM_SAVER:
      if ((idx = findIndex(saver_options, N_SAVER, v)) != 0xFF && idx != saver_idx) {
        saver_idx = idx;
        save_pending = true;
      }
      break;
    case PARAM_EXT_TPT: setExtTransport(v != 0); break;
  }
  ui_dirty = true;
}

void handleSysex(const uint8_t* d, uint8_t n) {   // d excludes F0 / F7
  if (n < 4 || d[0] != SYSEX_MFR || d[1] != SYSEX_ID1 || d[2] != SYSEX_ID2) return;
  switch (d[3]) {
    case CMD_GET_INFO:  sendInfo();  return;
    case CMD_GET_STATE: sendState(); return;
    case CMD_SET:
      if (n >= 7) setParam(d[4], (uint16_t)d[5] | ((uint16_t)d[6] << 7));
      sendState();
      return;
    case CMD_TRANSPORT:
      if (n >= 6 && d[5] <= 2) {
        if (d[4] == 0 || d[4] == 2) requestPlayStop(0, d[5]);
        if (d[4] == 1 || d[4] == 2) requestPlayStop(1, d[5]);
        wakeScreen();
      }
      sendState();
      return;
    case CMD_FACTORY_RESET:
      if (n >= 7 && d[4] == 'D' && d[5] == 'E' && d[6] == 'F') factoryReset();
      sendState();
      return;
  }
}

// Byte-stream parser fed from USB-MIDI packets. Real-time bytes are acted
// on immediately, even in the middle of a SysEx message.
void midiInByte(uint8_t b) {
  static int8_t  spp_n = -1;
  static uint8_t spp_b[2];
  if (b >= 0xF8) { realtimeIn(b); return; }
  if (b == 0xF0) { sx_rx_on = true; sx_rx_len = 0; sx_rx_ovf = false; spp_n = -1; return; }
  if (b == 0xF7) {
    if (sx_rx_on && !sx_rx_ovf) handleSysex(sx_rx, sx_rx_len);
    sx_rx_on = false;
    return;
  }
  if (b & 0x80) { sx_rx_on = false; spp_n = (b == 0xF2) ? 0 : -1; return; }
  if (sx_rx_on) {
    if (sx_rx_len < SX_RX_MAX) sx_rx[sx_rx_len++] = b;
    else sx_rx_ovf = true;
    return;
  }
  if (spp_n >= 0) {
    spp_b[spp_n++] = b;
    if (spp_n == 2) { songPosition((uint16_t)spp_b[0] | ((uint16_t)spp_b[1] << 7)); spp_n = -1; }
  }
}

// ---------- USB MIDI RX ----------
void processUSB_RX() {
  if (!TinyUSBDevice.mounted()) return;
  // Bytes carried per Code Index Number (USB-MIDI 1.0, table 4-1)
  static const uint8_t cin_len[16] = { 0, 0, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1 };
  uint8_t packet[4];
  while (usb_midi.readPacket(packet)) {
    uint8_t len = cin_len[packet[0] & 0x0F];
    for (uint8_t i = 0; i < len; i++) midiInByte(packet[1 + i]);
  }
}

// ---------- USB-clock housekeeping (loop context) ----------
void extService() {
  // Drop a stale tempo reading so the screen / editor show "no clock"
  if (ext_bpm_x10 && !extPresent()) ext_bpm_x10 = 0;
  {
    // Whole BPM for the main screen, with ~0.7 BPM hysteresis so it
    // doesn't flicker between two values
    int32_t diff = (int32_t)ext_bpm_x10 - (int32_t)shown_ext_bpm * 10;
    if (ext_bpm_x10 == 0 ? shown_ext_bpm != 0 : (shown_ext_bpm == 0 || diff > 7 || diff < -7)) {
      shown_ext_bpm = (ext_bpm_x10 + 5) / 10;
      if (extMode()) ui_dirty = true;
    }
  }
  if (!extMode()) return;

  // No ticks run in USB mode without a clock, so apply here what the
  // tick would normally pick up
  if (usb_out_req >= 0) { usb_out_channel = (uint8_t)usb_out_req; usb_out_req = -1; }

  if (hostGrid()) {
    if (host_running && (!TinyUSBDevice.mounted() || (millis() - ext_last_ms) > EXT_LOST_MS)) {
      hostStop();   // host vanished without sending Stop
    }
    if (!host_running) processIdleRequests();
  } else if (!extPresent()) {
    processIdleRequests();
  }
}

// ---------- USB MIDI TX (drains ring buffer in loop context) ----------
// Real-time bytes go out as explicit 4-byte USB-MIDI packets with
// CIN 0xF ("single byte"). TinyUSB's byte-stream writer packs F8/FA/FC
// as CIN 0x5 ("system common / SysEx end"), which strict hosts drop.
void processUSB_TX() {
  static bool was_mounted = false;
  bool mounted = TinyUSBDevice.mounted();
  if (mounted && !was_mounted) usb_resync = true;   // fresh host: resend Start
  was_mounted = mounted;

  if (!mounted) {
    usb_tx_tail = usb_tx_head;   // flush
    sx_tx_tail  = sx_tx_head;
    return;
  }
  while (usb_tx_head != usb_tx_tail) {
    uint8_t b = usb_tx_ring[usb_tx_tail];
    uint8_t pkt[4] = { CIN_SINGLE_BYTE, b, 0, 0 };   // cable 0
    if (!usb_midi.writePacket(pkt)) {               // TinyUSB FIFO full
      usb_pkts_retry++;
      return;                                       // retry next loop
    }
    usb_pkts_sent++;
    usb_tx_tail = (usb_tx_tail + 1) & (USB_TX_RING_SIZE - 1);
  }
  // SysEx replies go after clock bytes. Packets are self-contained, so a
  // clock byte may legally land between two SysEx packets.
  while (sx_tx_head != sx_tx_tail) {
    if (!usb_midi.writePacket(sx_tx[sx_tx_tail])) return;
    sx_tx_tail = (sx_tx_tail + 1) & (SX_TX_PKTS - 1);
  }
}

// MONITOR test note: C4 on channel 1, every second, while enabled.
void processUSB_TestNote() {
  static uint32_t last_on = 0;
  static bool note_on = false;
  if (screen != Screen::DIAG || !test_note_enabled || !TinyUSBDevice.mounted()) {
    if (note_on && TinyUSBDevice.mounted()) {
      uint8_t off[4] = { CIN_NOTE_OFF, 0x80, 60, 0 };
      usb_midi.writePacket(off);
    }
    note_on = false;
    return;
  }
  uint32_t now = millis();
  if (!note_on && (now - last_on) >= 1000) {
    uint8_t on[4] = { CIN_NOTE_ON, 0x90, 60, 100 };
    if (usb_midi.writePacket(on)) { note_on = true; last_on = now; }
  } else if (note_on && (now - last_on) >= 200) {
    uint8_t off[4] = { CIN_NOTE_OFF, 0x80, 60, 0 };
    if (usb_midi.writePacket(off)) note_on = false;
  }
}

// ---------- Boot animation (~1.8 s) ----------
// 1. ".axs instruments" fades in   2. four beat bars count in with the
// beat LED   3. "DCLK" wipes in left to right, both channel LEDs flash.
// delay() runs yield(), which services TinyUSB, so enumeration carries on.
void setContrast(uint8_t c) {
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(c);
}

void textBounds(const char* s, uint8_t size, int16_t& x, int16_t y, uint16_t& w) {
  int16_t x1, y1; uint16_t h;
  display.setTextSize(size);
  display.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
  x = (display.width() - w) / 2 - x1;
}

void bootAnimation() {
  display.setFont(&Picopixel);
  display.setTextColor(SSD1306_WHITE);
  int16_t x; uint16_t w;

  // 1. Maker name fades in
  display.clearDisplay();
  textBounds(".axs instruments", 2, x, 20, w);
  display.setCursor(x, 20);
  display.print(".axs instruments");
  setContrast(0);
  display.display();
  for (int c = 0; c <= 0xCF; c += 9) { setContrast(c); delay(12); }
  setContrast(0xCF);
  delay(450);

  // 2. Count-in: four bars drop in on the beat (same bars as the main screen)
  const int bar_w = display.width() / 4;
  for (int b = 0; b < 4; b++) {
    display.clearDisplay();
    for (int i = 0; i <= b; i++) {
      display.fillRect(i * bar_w + 2, (i == b) ? 31 : 28, bar_w - 4, 4, SSD1306_WHITE);
    }
    display.display();
    analogWrite(PIN_LED_BEAT, b == 0 ? 255 : 140);
    delay(60);
    analogWrite(PIN_LED_BEAT, 0);
    delay(90);
  }

  // 3. Product name wipes in above the bars
  textBounds("DCLK", 4, x, 22, w);
  for (int step = 1; step <= 10; step++) {
    display.clearDisplay();
    display.setTextSize(4);
    display.setCursor(x, 22);
    display.print("DCLK");
    int16_t shown = x + (int16_t)((w + 4) * step / 10);
    display.fillRect(shown, 0, display.width() - shown, 26, SSD1306_BLACK);
    for (int i = 0; i < 4; i++) display.fillRect(i * bar_w + 2, 28, bar_w - 4, 4, SSD1306_WHITE);
    display.display();
    delay(25);
  }
  analogWrite(PIN_LED_STATUS[0], 120);
  analogWrite(PIN_LED_STATUS[1], 120);
  delay(120);
  analogWrite(PIN_LED_STATUS[0], 0);
  analogWrite(PIN_LED_STATUS[1], 0);
  delay(380);
}

void applyTempo() {
  uint32_t us = 60000000UL / ((uint32_t)bpm * PPQN);
  tickTimer->setOverflow(us, MICROSEC_FORMAT);
}

// ---------- Screen saver ----------
// Returns true if the screen was asleep (the input should then only wake it).
bool wakeScreen() {
  last_activity = millis();
  if (screen_dimmed) {            // still readable: undim and let the input act
    screen_dimmed = false;
    display.dim(false);
  }
  if (screen_asleep) {
    screen_asleep = false;
    display.ssd1306_command(SSD1306_DISPLAYON);
    ui_dirty = true;
    return true;
  }
  return false;
}

// Timed out + clock running  -> dim (main screen stays readable)
// Timed out + all stopped     -> panel off, BEAT LED breathes (standby)
void processScreenSaver() {
  uint8_t mins = saver_options[saver_idx];
  if (screen_asleep || mins == 0) return;
  if (millis() - last_activity < (uint32_t)mins * 60000UL) return;

  // Leave any half-finished edit cleanly and return to the main screen
  if (screen != Screen::MAIN) {
    if (screen == Screen::LED_EDIT) led_level = led_edit_orig;
    screen = Screen::MAIN;
    ui_dirty = true;
  }

  bool active = running[0] || running[1] || pending[0] || pending[1];
  if (active) {
    if (!screen_dimmed) {
      screen_dimmed = true;
      display.dim(true);
    }
  } else {
    if (screen_dimmed) {
      screen_dimmed = false;
      display.dim(false);       // restore contrast for when it wakes
    }
    screen_asleep = true;
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}

// ---------- MIDI menu values ----------
uint8_t midiOptionCount(uint8_t item) {
  switch (item) {
    case MI_WAIT:  return N_WAIT;
    case MI_DIV1:
    case MI_DIV2:  return N_DIV;
    default:       return 2;
  }
}

uint8_t midiGet(uint8_t item) {
  switch (item) {
    case MI_WAIT:  return indexOf(wait_options, N_WAIT, wait_bars);
    case MI_DIV1:  return indexOf(div_options, N_DIV, clk_div[0]);
    case MI_DIV2:  return indexOf(div_options, N_DIV, clk_div[1]);
    case MI_STOPQ: return stop_quant ? 1 : 0;
    case MI_USB:   return (usb_out_req >= 0) ? (uint8_t)usb_out_req : usb_out_channel;
    case MI_SRC:   return clk_src;
    case MI_TPT:   return ext_tpt_host ? 0 : 1;
  }
  return 0;
}

void midiValueText(uint8_t item, uint8_t idx, char* buf, size_t n) {
  switch (item) {
    case MI_WAIT:  snprintf(buf, n, "%u", wait_options[idx]); break;
    case MI_DIV1:
    case MI_DIV2:  snprintf(buf, n, "/%u", div_options[idx]); break;
    case MI_STOPQ: snprintf(buf, n, "%s", idx ? "ON" : "OFF"); break;
    case MI_USB:   snprintf(buf, n, "%s", idx ? "CH2" : "CH1"); break;
    case MI_SRC:   snprintf(buf, n, "%s", idx ? "USB" : "INT"); break;
    case MI_TPT:   snprintf(buf, n, "%s", idx ? "FREE" : "HOST"); break;
    default:       buf[0] = 0;
  }
}

void midiApply(uint8_t item, uint8_t idx) {
  if (idx == midiGet(item)) return;   // unchanged
  switch (item) {
    case MI_WAIT:  wait_bars = wait_options[idx]; updateTiming(); break;
    case MI_DIV1:  clk_div[0] = div_options[idx]; break;
    case MI_DIV2:  clk_div[1] = div_options[idx]; break;
    case MI_STOPQ: stop_quant = (idx != 0); break;
    case MI_USB:   usb_out_req = (int8_t)idx; break;   // applied on next tick (or extService)
    case MI_SRC:   setClockSource(idx); break;
    case MI_TPT:   setExtTransport(idx == 0); break;
  }
  save_pending = true;
}

// ---------- Encoder ISR ----------
void onEncoder() {
  static uint8_t state = 0;
  static int8_t acc = 0;
  static const int8_t table[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
  state = ((state << 2) | (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B)) & 0x0F;
  acc += table[state];
  if (acc >= 4 || acc <= -4) {
    encoder_delta += (acc > 0 ? 1 : -1);
    acc = 0;
  }
}

// ---------- Encoder processing ----------
void processEncoder() {
  int16_t delta;
  noInterrupts();
  delta = encoder_delta;
  encoder_delta = 0;
  interrupts();
  if (delta == 0) return;
  if (wakeScreen()) return;   // first input only wakes the screen

  switch (screen) {
    case Screen::MAIN: {
      if (extMode()) break;           // tempo comes from USB
      int16_t v = (int16_t)bpm + delta;
      if (v < (int16_t)BPM_MIN) v = BPM_MIN;
      if (v > (int16_t)BPM_MAX) v = BPM_MAX;
      if (v != (int16_t)bpm) {
        bpm = (uint16_t)v;
        bpm_changed    = true;
        ui_dirty       = true;
        bpm_save_armed = true;        // restart the idle timer
        bpm_changed_ms = millis();
      }
      break;
    }
    case Screen::SETTINGS:
      menu_sel = wrapAdd(menu_sel, delta, N_SETTINGS);
      ui_dirty = true;
      break;
    case Screen::LED_VIEW:
      break;
    case Screen::LED_EDIT:
      led_level = wrapAdd(led_level, delta, N_LED_LEVELS);   // live preview
      ui_dirty  = true;
      break;
    case Screen::MIDI_VIEW:
      midi_menu_sel = wrapAdd(midi_menu_sel, delta, MI_COUNT);
      ui_dirty = true;
      break;
    case Screen::MIDI_EDIT:
      edit_idx = wrapAdd(edit_idx, delta, midiOptionCount(midi_menu_sel));
      ui_dirty = true;
      break;
    case Screen::SAVER_EDIT:
      edit_idx = wrapAdd(edit_idx, delta, N_SAVER);
      ui_dirty = true;
      break;
    case Screen::DIAG:
      break;
  }
}

// ---------- Buttons ----------
void onEncoderButton() {
  if (wakeScreen()) return;
  switch (screen) {
    case Screen::MAIN:
      break;
    case Screen::DIAG:
      test_note_enabled = !test_note_enabled;
      break;
    case Screen::ABOUT:
      screen = Screen::SETTINGS;
      break;
    case Screen::SETTINGS:
      switch (menu_sel) {
        case SI_LEDS:
          screen = Screen::LED_VIEW;
          break;
        case SI_MIDI:
          screen = Screen::MIDI_VIEW;
          midi_menu_sel = 0;
          break;
        case SI_SCREEN:
          edit_idx = saver_idx;
          screen = Screen::SAVER_EDIT;
          break;
        case SI_MONITOR:
          test_note_enabled = false;
          screen = Screen::DIAG;
          break;
        case SI_ABOUT:
          screen = Screen::ABOUT;
          break;
      }
      break;
    case Screen::LED_VIEW:
      led_edit_orig = led_level;
      screen = Screen::LED_EDIT;
      break;
    case Screen::LED_EDIT:
      save_pending = true;
      screen = Screen::LED_VIEW;
      break;
    case Screen::MIDI_VIEW:
      edit_idx = midiGet(midi_menu_sel);
      screen = Screen::MIDI_EDIT;
      break;
    case Screen::MIDI_EDIT:
      midiApply(midi_menu_sel, edit_idx);
      screen = Screen::MIDI_VIEW;
      break;
    case Screen::SAVER_EDIT:
      if (edit_idx != saver_idx) { saver_idx = edit_idx; save_pending = true; }
      screen = Screen::SETTINGS;
      break;
  }
  ui_dirty = true;
}

void pollEncoderButton() {
  static uint32_t lastDebounceTime = 0;
  static bool buttonState = HIGH;
  static bool lastReading = HIGH;

  bool reading = digitalRead(PIN_ENC_BTN);
  if (reading != lastReading) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > 30) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        onEncoderButton();
      }
    }
  }
  lastReading = reading;
}

void onSettingsButton() {
  if (wakeScreen()) return;
  switch (screen) {
    case Screen::MAIN:
      screen   = Screen::SETTINGS;
      menu_sel = 0;
      break;
    case Screen::SETTINGS:
      screen   = Screen::MAIN;
      break;
    case Screen::DIAG:
      test_note_enabled = false;
      screen   = Screen::SETTINGS;
      break;
    case Screen::ABOUT:
      screen   = Screen::SETTINGS;
      break;
    case Screen::LED_VIEW:
      screen   = Screen::SETTINGS;
      break;
    case Screen::LED_EDIT:
      led_level = led_edit_orig;
      screen   = Screen::LED_VIEW;
      break;
    case Screen::MIDI_VIEW:
      screen   = Screen::SETTINGS;
      break;
    case Screen::MIDI_EDIT:
      screen   = Screen::MIDI_VIEW;     // cancel: nothing was applied
      break;
    case Screen::SAVER_EDIT:
      screen   = Screen::SETTINGS;      // cancel
      break;
  }
  ui_dirty = true;
}

void pollSettingsButton() {
  static uint32_t lastDebounceTime = 0;
  static bool buttonState = HIGH;
  static bool lastReading = HIGH;

  bool reading = digitalRead(PIN_BTN_SETTINGS);
  if (reading != lastReading) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > 30) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        onSettingsButton();
      }
    }
  }
  lastReading = reading;
}

// Output buttons: press = play / stop (acts immediately, even if the
// screen is asleep).
void pollButtons() {
  static uint32_t lastDebounceTime[2] = {0, 0};
  static bool buttonState[2] = {HIGH, HIGH};
  static bool lastReading[2] = {HIGH, HIGH};

  for (int i = 0; i < 2; i++) {
    bool reading = digitalRead(PIN_BTN[i]);
    if (reading != lastReading[i]) {
      lastDebounceTime[i] = millis();
    }
    if ((millis() - lastDebounceTime[i]) > 30) {
      if (reading != buttonState[i]) {
        buttonState[i] = reading;
        if (buttonState[i] == LOW) {
          wakeScreen();
          if (running[i] || pending[i]) stop_req[i] = true;
          else                          play_req[i] = true;
        }
      }
    }
    lastReading[i] = reading;
  }
}

// ---------- Status LEDs ----------
constexpr uint16_t FADE_PERIOD_MS = 1400;

void updateStatusLeds() {
  const uint8_t led_base  = led_levels[led_level].base;
  const uint8_t led_flash = led_levels[led_level].flash;
  uint32_t t    = millis() % FADE_PERIOD_MS;
  uint32_t tick = loop_tick;
  bool any_running = running[0] || running[1];

  uint8_t phase = 0;
  if (any_running) {
    phase = (uint8_t)((tick + PPQN - 1) % PPQN);
  }

  if (any_running) {
    uint16_t peak = (uint16_t)led_base + led_flash;
    if (peak > 255) peak = 255;
    uint8_t beat_duty = (uint32_t)peak * (PPQN - 1 - phase) / (PPQN - 1);
    analogWrite(PIN_LED_BEAT, beat_duty);
  } else if (screen_asleep && !pending[0] && !pending[1]) {
    // Standby: soft breathe, one cycle per bar at the current tempo
    uint32_t period = 240000UL / bpm;                 // 4 beats in ms
    uint32_t pt     = millis() % period;
    uint32_t half   = period / 2;
    uint32_t tri    = (pt < half) ? (pt * 255 / half) : ((period - pt) * 255 / half);
    uint16_t peak   = led_base / 3;
    if (peak < 8) peak = 8;                           // still visible at low LED setting
    analogWrite(PIN_LED_BEAT, (uint8_t)(((tri * tri) / 255) * peak / 255));
  } else {
    analogWrite(PIN_LED_BEAT, 0);
  }

  uint8_t flash = 0;
  if (any_running) {
    flash = (uint32_t)led_flash * (PPQN - 1 - phase) / (PPQN - 1);
  }

  for (int i = 0; i < 2; i++) {
    uint8_t duty;
    if (running[i]) {
      uint16_t v = (uint16_t)led_base + flash;
      duty = (v > 255) ? 255 : (uint8_t)v;
      // Quantised stop waiting: blink on the beat (on for the first half)
      if (stop_pend[i] && phase >= PPQN / 2) duty /= 6;
    } else if (pending[i]) {
      uint32_t half = FADE_PERIOD_MS / 2;
      uint32_t tri  = (t < half) ? (t * 255 / half) : ((FADE_PERIOD_MS - t) * 255 / half);
      duty = ((tri * tri) / 255) * led_base / 255;
    } else {
      duty = 0;
    }
    analogWrite(PIN_LED_STATUS[i], duty);
  }
}

// ---------- Clock readback for DIAG ----------
void readClocks(bool& hse, uint32_t& sys_khz, uint32_t& usb_khz) {
  uint32_t cfg  = RCC->PLLCFGR;
  hse           = (cfg & RCC_PLLCFGR_PLLSRC) != 0;
  uint32_t src  = hse ? HSE_VALUE : HSI_VALUE;
  uint32_t m    = cfg & RCC_PLLCFGR_PLLM;
  uint32_t n    = (cfg & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
  uint32_t q    = (cfg & RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos;
  uint32_t vco_khz = (m && q) ? (src / 1000 / m) * n : 0;
  usb_khz = q ? vco_khz / q : 0;
  sys_khz = HAL_RCC_GetSysClockFreq() / 1000;
}

// ---------- Display ----------
void drawRightAligned(const char* s, int16_t y, int16_t rightMargin = 0) {
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
  display.setCursor(display.width() - rightMargin - w - x1, y);
  display.print(s);
}

void drawUI() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setFont(&Picopixel);

  switch (screen) {
    case Screen::MAIN: {
      display.setTextSize(4);
      display.setCursor(6, 22);
      if (extMode()) {
        display.print("EXT: ");
        if (shown_ext_bpm) display.print(shown_ext_bpm);
        else               display.print("--");
      } else {
        display.print("BPM: ");
        display.print(bpm);
      }

      int active_bar = -1;
      if (running[0] || running[1]) {
        active_bar = (loop_tick / PPQN) % 4;
      }

      int bar_w = display.width() / 4;
      for (int i = 0; i < 4; i++) {
        int y_pos = 28;
        if (i == active_bar) {
          y_pos = 31;
        }
        display.fillRect(i * bar_w + 2, y_pos, bar_w - 4, 4, SSD1306_WHITE);
      }
      break;
    }

    // Two-line paged list
    case Screen::SETTINGS: {
      display.setTextSize(3);
      uint8_t top = menu_sel & ~1;
      for (uint8_t row = 0; row < 2; row++) {
        uint8_t item = top + row;
        if (item >= N_SETTINGS) break;
        display.setCursor(0, row ? 30 : 12);
        display.print(item == menu_sel ? "> " : "  ");
        display.print(settings_labels[item]);
      }
      break;
    }

    case Screen::LED_VIEW: {
      display.setTextSize(3);
      display.setCursor(0, 26);
      display.print(save_pending ? "LEDS*" : "LEDS");
      drawRightAligned(led_levels[led_level].name, 26);
      break;
    }

    case Screen::LED_EDIT: {
      display.setTextSize(3);
      display.setCursor(0, 26);
      display.print("LEDS >");
      drawRightAligned(led_levels[led_level].name, 26);
      break;
    }

    case Screen::MIDI_VIEW: {
      display.setTextSize(3);
      uint8_t top = midi_menu_sel & ~1;
      for (uint8_t row = 0; row < 2; row++) {
        uint8_t item = top + row;
        if (item >= MI_COUNT) break;
        int16_t y = row ? 30 : 12;
        display.setCursor(0, y);
        if (item == midi_menu_sel) display.print(">");
        display.print(midi_labels[item]);
        char val[8];
        midiValueText(item, midiGet(item), val, sizeof(val));
        drawRightAligned(val, y);
      }
      break;
    }

    case Screen::MIDI_EDIT: {
      display.setTextSize(3);
      display.setCursor(0, 12);
      display.print(">");
      display.print(midi_labels[midi_menu_sel]);
      display.setCursor(0, 30);
      display.print(midi_menu_sel == MI_WAIT ? "BARS >" : "SET >");
      char val[8];
      midiValueText(midi_menu_sel, edit_idx, val, sizeof(val));
      drawRightAligned(val, 30);
      break;
    }

    case Screen::SAVER_EDIT: {
      display.setTextSize(3);
      display.setCursor(0, 12);
      display.print(">SAVER");
      display.setCursor(0, 30);
      display.print("SET >");
      char val[8];
      if (saver_options[edit_idx] == 0) snprintf(val, sizeof(val), "OFF");
      else snprintf(val, sizeof(val), "%uM", saver_options[edit_idx]);
      drawRightAligned(val, 30);
      break;
    }

    // ---------- MONITOR (USB diagnostics, default 6x8 font) ----------
    // HOST = host has configured the device, LINK = bus active (not
    // suspended), MIDI = MIDI interface open. CLK = USB transport state.
    // HSE/HSI = clock source; SYS / USB in MHz (USB must read 48.0).
    // TX = packets sent, FULL = USB buffer full (retried), DROP = lost.
    case Screen::DIAG: {
      bool hse; uint32_t sys_khz, usb_khz;
      readClocks(hse, sys_khz, usb_khz);
      char line[24];

      display.setFont();          // built-in 5x7 font
      display.setTextSize(1);

      snprintf(line, sizeof(line), "HOST:%c LINK:%c MIDI:%c",
               TinyUSBDevice.mounted() ? 'Y' : 'N',
               tud_ready() ? 'Y' : 'N',
               tud_midi_mounted() ? 'Y' : 'N');
      display.setCursor(0, 0);   display.print(line);

      snprintf(line, sizeof(line), "%s SYS:%lu USB:%lu.%lu",
               hse ? "HSE" : "HSI",
               (unsigned long)(sys_khz / 1000),
               (unsigned long)(usb_khz / 1000),
               (unsigned long)((usb_khz % 1000) / 100));
      display.setCursor(0, 8);   display.print(line);

      snprintf(line, sizeof(line), "TX:%lu CLK:%s",
               (unsigned long)usb_pkts_sent,
               usb_running ? "RUN" : "STOP");
      display.setCursor(0, 16);  display.print(line);

      snprintf(line, sizeof(line), "FULL:%lu DROP:%lu %s",
               (unsigned long)usb_pkts_retry,
               (unsigned long)usb_ring_drops,
               test_note_enabled ? "NOTE" : "");
      display.setCursor(0, 24);  display.print(line);
      break;
    }

    // ---------- ABOUT ----------
    case Screen::ABOUT: {
      char fw[16];
      snprintf(fw, sizeof(fw), "fw v%s", FW_VERSION);
      display.setTextSize(3);
      display.setCursor(0, 15);
      display.print("DCLK");
      display.setTextSize(2);
      drawRightAligned(fw, 15);
      display.setCursor(0, 31);
      display.print(".axs instruments");
      break;
    }
  }
  displayStartFlush();
}

// ---------- Non-blocking display flush ----------
// display.display() holds the CPU for ~12 ms (512 bytes of I2C at 400 kHz),
// long enough to bunch up incoming USB clock ticks. Instead the frame is sent
// in small I2C transactions, one per loop() pass (< 1 ms each), so USB is
// serviced in between. Same command sequence as Adafruit_SSD1306::display().
constexpr uint8_t  OLED_ADDR   = 0x3C;
constexpr uint8_t  DISP_CHUNK  = 24;                    // data bytes per transaction
constexpr uint16_t DISP_BYTES  = 128 * 32 / 8;

void displayStartFlush() {
  display.ssd1306_command(SSD1306_PAGEADDR);
  display.ssd1306_command(0);
  display.ssd1306_command(0xFF);
  display.ssd1306_command(SSD1306_COLUMNADDR);
  display.ssd1306_command(0);
  display.ssd1306_command(127);
  disp_pos  = 0;
  disp_busy = true;
}

void displayService() {
  if (!disp_busy) return;
  const uint8_t* buf = display.getBuffer();
  uint16_t n = DISP_BYTES - disp_pos;
  if (n > DISP_CHUNK) n = DISP_CHUNK;
  Wire.beginTransmission(OLED_ADDR);
  Wire.write((uint8_t)0x40);                            // data stream
  Wire.write(buf + disp_pos, n);
  Wire.endTransmission();
  disp_pos += n;
  if (disp_pos >= DISP_BYTES) disp_busy = false;
}

// ---------- USB bring-up ----------
// Single-port, MIDI-only device (no CDC serial alongside).
void usbInit() {
  // stm32duino has no built-in TinyUSB integration, so the stack is started
  // manually. begin() connects to the bus straight away, so detach while
  // the descriptors are built, then attach once they're final.
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
  TinyUSBDevice.detach();
  TinyUSBDevice.clearConfiguration();   // drop default CDC: plain MIDI device

  TinyUSBDevice.setManufacturerDescriptor(".axs instruments");
  TinyUSBDevice.setProductDescriptor("DCLK");
  usb_midi.setStringDescriptor("DCLK MIDI");
  usb_midi.begin();

  delay(10);
  TinyUSBDevice.attach();
}

// ---------- Settings load / save ----------
// Uses the core's buffered EEPROM API: EEPROM.put() erases and rewrites the
// whole flash sector once per changed byte, the buffer does it once per save.
void loadSettings() {
  PersistedSettings s;
  eeprom_buffer_fill();
  uint8_t* p = (uint8_t*)&s;
  for (size_t i = 0; i < sizeof(s); i++) p[i] = eeprom_buffered_read_byte(EEPROM_SETTINGS_ADDR + i);
  if (s.magic != EEPROM_MAGIC) return;   // keep defaults

  led_level = (s.led_level < N_LED_LEVELS) ? s.led_level : 1;
  wait_bars = wait_options[indexOf(wait_options, N_WAIT, s.wait_bars)];
  usb_out_channel = (s.usb_out_channel > 1) ? 0 : s.usb_out_channel;
  for (int i = 0; i < 2; i++) clk_div[i] = div_options[indexOf(div_options, N_DIV, s.clk_div[i])];
  stop_quant = s.stop_quant != 0;
  saver_idx  = (s.saver_idx < N_SAVER) ? s.saver_idx : 2;
  uint16_t b = (uint16_t)s.bpm_lo | ((uint16_t)s.bpm_hi << 8);
  if (b >= BPM_MIN && b <= BPM_MAX) bpm = b;   // else keep the default
  clk_src      = (s.clk_src == CLK_USB) ? CLK_USB : CLK_INT;
  ext_tpt_host = (s.ext_tpt != 0);             // 0 = FREE, else (incl. erased) HOST
}

// Web editor: back to the defaults (loop context, stops all clocks)
void factoryReset() {
  tickTimer->pause();
  stopAll();
  clk_src         = CLK_INT;
  ext_tpt_host    = true;
  led_level       = 1;
  wait_bars       = 4;
  updateTiming();
  stop_quant      = false;
  clk_div[0]      = clk_div[1] = 1;
  usb_out_channel = 0;
  usb_out_req     = -1;
  saver_idx       = 2;
  bpm             = 120;
  applyTempo();
  tickTimer->resume();
  bpm_changed     = true;
  save_pending    = true;
}

void saveSettings() {
  PersistedSettings ns = {
    led_level, wait_bars, usb_out_channel,
    { clk_div[0], clk_div[1] },
    (uint8_t)stop_quant, saver_idx, EEPROM_MAGIC,
    (uint8_t)(bpm & 0xFF), (uint8_t)(bpm >> 8),
    clk_src, (uint8_t)(ext_tpt_host ? 1 : 0)
  };
  eeprom_buffer_fill();
  const uint8_t* p = (const uint8_t*)&ns;
  bool changed = false;
  for (size_t i = 0; i < sizeof(ns); i++) {
    if (eeprom_buffered_read_byte(EEPROM_SETTINGS_ADDR + i) != p[i]) {
      eeprom_buffered_write_byte(EEPROM_SETTINGS_ADDR + i, p[i]);
      changed = true;
    }
  }
  if (changed) eeprom_buffer_flush();   // one sector erase + program
}

// ---------- Setup / loop ----------
void setup() {
  loadSettings();
  updateTiming();

  usbInit();        // early, so enumeration isn't delayed

  MidiOut1.begin(31250);
  MidiOut2.begin(31250);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_BTN[0], INPUT_PULLUP);
  pinMode(PIN_BTN[1], INPUT_PULLUP);
  pinMode(PIN_ENC_BTN, INPUT_PULLUP);
  pinMode(PIN_BTN_SETTINGS, INPUT_PULLUP);
  attachInterrupt(PIN_ENC_A, onEncoder, CHANGE);
  attachInterrupt(PIN_ENC_B, onEncoder, CHANGE);

  pinMode(PIN_LED_BEAT, OUTPUT);
  pinMode(PIN_LED_STATUS[0], OUTPUT);
  pinMode(PIN_LED_STATUS[1], OUTPUT);
  analogWriteFrequency(1000);
  analogWrite(PIN_LED_BEAT, 0);
  analogWrite(PIN_LED_STATUS[0], 0);
  analogWrite(PIN_LED_STATUS[1], 0);

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(400000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(DISPLAY_ROTATION);   // applies to every screen, incl. boot animation

  bootAnimation();

  // Finish waiting for USB enumeration if it isn't done yet
  uint32_t usb_start = millis();
  while (!TinyUSBDevice.mounted() && (millis() - usb_start) < 3000) {
    TinyUSBDevice.task();
    delay(1);
  }

  last_activity = millis();

  applyTempo();
  tickTimer->setInterruptPriority(0, 0);
  tickTimer->attachInterrupt(onTick);
  if (clk_src == CLK_INT) tickTimer->resume();
  else                    tickTimer->pause();   // USB mode: ticks come from the host
}

void loop() {
  TinyUSBDevice.task();
  processUSB_RX();
  processUSB_TX();
  processUSB_TestNote();
  extService();

  pollButtons();
  pollEncoderButton();
  pollSettingsButton();
  processEncoder();
  updateStatusLeds();
  processScreenSaver();

  if (bpm_changed) {
    bpm_changed = false;
    applyTempo();
    ui_dirty = true;
  }

  // Trigger a redraw when the active beat bar changes
  static int last_active_bar = -2;
  int current_active_bar = -1;
  if (running[0] || running[1]) {
    current_active_bar = (loop_tick / PPQN) % 4;
  }
  if (current_active_bar != last_active_bar) {
    last_active_bar = current_active_bar;
    ui_dirty = true;
  }

  // MONITOR screen refreshes its counters 4x per second
  static uint32_t last_diag = 0;
  if (screen == Screen::DIAG && (millis() - last_diag) >= 250) {
    last_diag = millis();
    ui_dirty = true;
  }

  // BPM idle long enough -> queue a save
  if (bpm_save_armed && (millis() - bpm_changed_ms) >= BPM_SAVE_DELAY_MS) {
    bpm_save_armed = false;
    save_pending   = true;
  }

  // Flash writes stall the CPU, so only save while every output is stopped.
  // A BPM change made while playing is saved as soon as everything stops.
  // Following a host, armed outputs are idle until the host starts.
  const bool idle = !running[0] && !running[1] &&
                    (hostGrid() ? !host_running : (!pending[0] && !pending[1]));
  if (save_pending && usb_out_req < 0 && idle && !disp_busy) {
    save_pending = false;

    analogWrite(PIN_LED_BEAT,      0);
    analogWrite(PIN_LED_STATUS[0], 0);
    analogWrite(PIN_LED_STATUS[1], 0);

    saveSettings();
    ui_dirty = true;
  }

  if (ui_dirty && !screen_asleep && !disp_busy) {
    ui_dirty = false;
    drawUI();
  }
  displayService();
}