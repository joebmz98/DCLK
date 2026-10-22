#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/Picopixel.h>
#include <EEPROM.h>
#include <Adafruit_TinyUSB.h>

// ---------- Config ----------
constexpr uint8_t  PPQN       = 24;
constexpr uint16_t BPM_MIN = 30, BPM_MAX = 300;

// ---------- LED brightness ----------
constexpr uint8_t LED_MAX_PWM  = 178;   // 70% of 255
constexpr uint8_t FLASH_AMOUNT = 76;    // 30% of 255

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
HardwareTimer* tickTimer = new HardwareTimer(TIM2);

// ---------- USB MIDI ----------
Adafruit_USBD_MIDI usb_midi;

// ---------- USB TX ring buffer (ISR-safe SPSC) ----------
constexpr uint8_t USB_TX_RING_SIZE = 32;          // must be power of 2
volatile uint8_t usb_tx_ring[USB_TX_RING_SIZE];
volatile uint8_t usb_tx_head = 0;
volatile uint8_t usb_tx_tail = 0;

inline bool usb_tx_enqueue(uint8_t b) {
  uint8_t next = (usb_tx_head + 1) & (USB_TX_RING_SIZE - 1);
  if (next == usb_tx_tail) return false;          // full
  usb_tx_ring[usb_tx_head] = b;
  usb_tx_head = next;
  return true;
}

// ---------- Persistent settings (emulated EEPROM in flash) ----------
struct PersistedSettings {
  uint8_t led_max;
  uint8_t wait_bars;
  uint8_t usb_out_channel;
  uint8_t magic;
};

constexpr uint16_t EEPROM_SETTINGS_ADDR = 0;
constexpr uint8_t  EEPROM_MAGIC         = 0xA9;   // bumped: layout changed

// Allowed options
constexpr uint8_t wait_options[] = {1, 2, 4, 8, 16};

// ---------- State ----------
volatile uint16_t bpm = 120;
volatile bool bpm_changed = true;
volatile bool ui_dirty = true;

volatile bool running[2]  = { false, false };
volatile bool pending[2]  = { false, false };
volatile bool play_req[2] = { false, false };
volatile bool stop_req[2] = { false, false };
volatile uint32_t loop_tick = 0;
volatile uint32_t loop_ticks = 384; // dynamically calculated

volatile uint8_t led_max = 107;
volatile uint8_t wait_bars = 4;

volatile uint8_t usb_out_channel = 0;   // 0 = CH1, 1 = CH2

bool save_pending = false;

// ---------- UI state (loop() context only) ----------
enum class Screen : uint8_t {
  MAIN, SETTINGS,
  LED_VIEW, LED_EDIT,
  MIDI_VIEW, MIDI_WAIT_EDIT,
  MIDI_USB_OUT_EDIT
};
Screen  screen        = Screen::MAIN;
uint8_t menu_sel      = 0;
uint8_t midi_menu_sel = 0;   // 0=BARS, 1=USB OUT

// Temporary storage for cancel actions
uint8_t led_edit_orig = 0;
uint8_t wait_bars_orig = 0;
uint8_t usb_out_channel_orig = 0;

volatile int16_t encoder_delta = 0;

// ---------- Helpers ----------
void updateTiming() {
  constexpr uint32_t ticks_per_bar = PPQN * 4;
  loop_ticks = (uint32_t)wait_bars * ticks_per_bar;
  if (loop_ticks == 0) loop_ticks = 1;
}

// Send a single byte to one physical output (and to USB if this is the mirrored channel)
void sendMIDIByte(uint8_t idx, uint8_t b) {
  midi[idx]->write(b);
  if (idx == usb_out_channel) {
    usb_tx_enqueue(b);              // USB TX happens in loop()
  }
}

// ---------- Core tick (driven by internal timer) ----------
void processTick() {
  // 1. Stops (immediate)
  for (int i = 0; i < 2; i++) {
    if (stop_req[i]) {
      stop_req[i] = false;
      if (running[i]) sendMIDIByte(i, 0xFC);   // MIDI Stop
      running[i] = false;
      pending[i] = false;
      ui_dirty = true;
    }
  }

  // 2. Play requests
  for (int i = 0; i < 2; i++) {
    if (play_req[i]) {
      play_req[i] = false;
      pending[i] = true;
      ui_dirty = true;
    }
  }

  // 3. Start pending outputs
  bool any = running[0] || running[1];
  if (!any && (pending[0] || pending[1])) loop_tick = 0;
  bool boundary = !any || (loop_tick % loop_ticks == 0);

  for (int i = 0; i < 2; i++) {
    if (pending[i] && boundary) {
      pending[i] = false;
      running[i] = true;
      sendMIDIByte(i, 0xFA);   // MIDI Start
      ui_dirty = true;
    }
  }

  // 4. Send clock to running outputs
  for (int i = 0; i < 2; i++) {
    if (running[i]) sendMIDIByte(i, 0xF8);
  }

  // 5. Advance loop counter
  if (running[0] || running[1]) {
    loop_tick++;
  }
}

// ---------- Timer ISR ----------
void onTick() {
  processTick();
}

// ---------- USB MIDI RX (drained and ignored) ----------
void processUSB_RX() {
  if (!TinyUSBDevice.mounted()) return;
  uint8_t packet[4];
  // Drain the USB packet buffer so it never overflows.
  while (usb_midi.readPacket(packet)) {
    // USB MIDI IN is not used – internal timer is always the master clock.
  }
}

// ---------- USB MIDI TX (drains ring buffer in loop context) ----------
void processUSB_TX() {
  if (!TinyUSBDevice.mounted()) {
    usb_tx_tail = usb_tx_head;   // flush
    return;
  }
  while (usb_tx_head != usb_tx_tail) {
    uint8_t b = usb_tx_ring[usb_tx_tail];
    // write() is the portable API for single-byte real-time messages.
    // writePacket() has inconsistent return semantics across Adafruit_TinyUSB
    // versions (some return 1, some return 4), which silently broke the TX
    // loop and meant START / CLOCK / STOP never reached the USB device.
    if (usb_midi.write(b) != 1) break;   // USB TX buffer full – retry next loop
    usb_tx_tail = (usb_tx_tail + 1) & (USB_TX_RING_SIZE - 1);
  }
}

void applyTempo() {
  uint32_t us = 60000000UL / ((uint32_t)bpm * PPQN);
  tickTimer->setOverflow(us, MICROSEC_FORMAT);
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

  switch (screen) {
    case Screen::MAIN: {
      int16_t v = (int16_t)bpm + delta;
      if (v < (int16_t)BPM_MIN) v = BPM_MIN;
      if (v > (int16_t)BPM_MAX) v = BPM_MAX;
      if (v != (int16_t)bpm) {
        bpm = (uint16_t)v;
        bpm_changed = true;
        ui_dirty    = true;
      }
      break;
    }
    case Screen::SETTINGS: {
      int16_t v = (int16_t)menu_sel + delta;
      while (v < 0) v += 2;
      v %= 2;
      menu_sel = (uint8_t)v;
      ui_dirty = true;
      break;
    }
    case Screen::LED_VIEW:
      break;
    case Screen::LED_EDIT: {
      int16_t v = (int16_t)led_max + delta * 5;
      if (v < 0)              v = 0;
      if (v > LED_MAX_PWM)    v = LED_MAX_PWM;
      led_max  = (uint8_t)v;
      ui_dirty = true;
      break;
    }
    case Screen::MIDI_VIEW: {
      int16_t v = (int16_t)midi_menu_sel + delta;
      while (v < 0) v += 2;
      v %= 2;
      midi_menu_sel = (uint8_t)v;
      ui_dirty = true;
      break;
    }
    case Screen::MIDI_WAIT_EDIT: {
      int idx = 0;
      for (int i = 0; i < 5; i++) if (wait_options[i] == wait_bars) idx = i;
      idx = (idx + delta) % 5;
      if (idx < 0) idx += 5;
      wait_bars = wait_options[idx];
      updateTiming();
      ui_dirty = true;
      break;
    }
    case Screen::MIDI_USB_OUT_EDIT: {
      usb_out_channel = (usb_out_channel == 0) ? 1 : 0;
      ui_dirty = true;
      break;
    }
  }
}

// ---------- Buttons ----------
void onEncoderButton() {
  switch (screen) {
    case Screen::MAIN:
      break;
    case Screen::SETTINGS:
      if (menu_sel == 0) {
        screen = Screen::LED_VIEW;
      } else if (menu_sel == 1) {
        screen = Screen::MIDI_VIEW;
        midi_menu_sel = 0;
      }
      ui_dirty = true;
      break;
    case Screen::LED_VIEW:
      led_edit_orig = led_max;
      screen = Screen::LED_EDIT;
      ui_dirty = true;
      break;
    case Screen::LED_EDIT:
      save_pending = true;
      screen = Screen::LED_VIEW;
      ui_dirty = true;
      break;
    case Screen::MIDI_VIEW:
      switch (midi_menu_sel) {
        case 0: wait_bars_orig = wait_bars;     screen = Screen::MIDI_WAIT_EDIT;     break;
        case 1: usb_out_channel_orig = usb_out_channel; screen = Screen::MIDI_USB_OUT_EDIT; break;
      }
      ui_dirty = true;
      break;
    case Screen::MIDI_WAIT_EDIT:
      save_pending = true;
      screen = Screen::MIDI_VIEW;
      ui_dirty = true;
      break;
    case Screen::MIDI_USB_OUT_EDIT:
      save_pending = true;
      screen = Screen::MIDI_VIEW;
      ui_dirty = true;
      break;
  }
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
  switch (screen) {
    case Screen::MAIN:
      screen   = Screen::SETTINGS;
      menu_sel = 0;
      ui_dirty = true;
      break;
    case Screen::SETTINGS:
      screen   = Screen::MAIN;
      ui_dirty = true;
      break;
    case Screen::LED_VIEW:
      screen   = Screen::SETTINGS;
      ui_dirty = true;
      break;
    case Screen::LED_EDIT:
      led_max  = led_edit_orig;
      screen   = Screen::LED_VIEW;
      ui_dirty = true;
      break;
    case Screen::MIDI_VIEW:
      screen   = Screen::SETTINGS;
      ui_dirty = true;
      break;
    case Screen::MIDI_WAIT_EDIT:
      wait_bars = wait_bars_orig;
      updateTiming();
      screen   = Screen::MIDI_VIEW;
      ui_dirty = true;
      break;
    case Screen::MIDI_USB_OUT_EDIT:
      usb_out_channel = usb_out_channel_orig;
      screen   = Screen::MIDI_VIEW;
      ui_dirty = true;
      break;
  }
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
  uint32_t t    = millis() % FADE_PERIOD_MS;
  uint32_t tick = loop_tick;
  bool any_running = running[0] || running[1];

  uint8_t phase = 0;
  if (any_running) {
    phase = (uint8_t)((tick + PPQN - 1) % PPQN);
  }

  if (any_running) {
    uint16_t peak = (uint16_t)led_max + FLASH_AMOUNT;
    if (peak > 255) peak = 255;
    uint8_t beat_duty = (uint32_t)peak * (PPQN - 1 - phase) / (PPQN - 1);
    analogWrite(PIN_LED_BEAT, beat_duty);
  } else {
    analogWrite(PIN_LED_BEAT, 0);
  }

  uint8_t flash = 0;
  if (any_running) {
    flash = (uint32_t)FLASH_AMOUNT * (PPQN - 1 - phase) / (PPQN - 1);
  }

  for (int i = 0; i < 2; i++) {
    uint8_t duty;
    if (running[i]) {
      uint16_t v = (uint16_t)led_max + flash;
      duty = (v > 255) ? 255 : (uint8_t)v;
    } else if (pending[i]) {
      uint32_t half = FADE_PERIOD_MS / 2;
      uint32_t tri  = (t < half) ? (t * 255 / half) : ((FADE_PERIOD_MS - t) * 255 / half);
      duty = ((tri * tri) / 255) * led_max / 255;
    } else {
      duty = 0;
    }
    analogWrite(PIN_LED_STATUS[i], duty);
  }
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
      display.print("BPM: ");
      display.print(bpm);

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

    case Screen::SETTINGS:
      display.setTextSize(3);
      display.setCursor(0, 12);
      display.print(menu_sel == 0 ? "> LEDS" : "  LEDS");
      display.setCursor(0, 30);
      display.print(menu_sel == 1 ? "> MIDI" : "  MIDI");
      break;

    case Screen::LED_VIEW: {
      display.setTextSize(3);
      display.setCursor(0, 26);
      display.print(save_pending ? "LEDS*" : "LEDS");
      char pct[8];
      snprintf(pct, sizeof(pct), "%u%%", (uint16_t)led_max * 100 / LED_MAX_PWM);
      drawRightAligned(pct, 26);
      break;
    }

    case Screen::LED_EDIT: {
      display.setTextSize(3);
      display.setCursor(0, 26);
      display.print("LEDS >");
      char pct[8];
      snprintf(pct, sizeof(pct), "%u%%", (uint16_t)led_max * 100 / LED_MAX_PWM);
      drawRightAligned(pct, 26);
      break;
    }

    // ---------- MIDI menu: exactly two items, both visible ----------
    case Screen::MIDI_VIEW: {
      display.setTextSize(3);

      display.setCursor(0, 12);
      display.print(midi_menu_sel == 0 ? ">BARS" : " BARS");
      char barsBuf[5];
      snprintf(barsBuf, sizeof(barsBuf), "%u", wait_bars);
      drawRightAligned(barsBuf, 12);

      display.setCursor(0, 30);
      display.print(midi_menu_sel == 1 ? ">OUT" : " OUT");
      drawRightAligned(usb_out_channel == 0 ? "CH1" : "CH2", 30);
      break;
    }

    case Screen::MIDI_WAIT_EDIT: {
      display.setTextSize(3);
      display.setCursor(0, 26);
      display.print("BARS >");
      char buf[8];
      snprintf(buf, sizeof(buf), "%u", wait_bars);
      drawRightAligned(buf, 26);
      break;
    }

    case Screen::MIDI_USB_OUT_EDIT: {
      display.setTextSize(3);
      display.setCursor(0, 26);
      display.print("OUT >");
      drawRightAligned(usb_out_channel == 0 ? "CH1" : "CH2", 26);
      break;
    }
  }
  display.display();
}

// ---------- Setup / loop ----------
void setup() {
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

  // --- USB MIDI setup ---
  // Descriptors must be set BEFORE begin() so they take effect during enumeration.
  TinyUSBDevice.setManufacturerDescriptor(".axs instruments");
  TinyUSBDevice.setProductDescriptor("DCLK - Dual MIDI Clock");
  usb_midi.setStringDescriptor("DCLK MIDI");
  usb_midi.begin();

  // Brief wait for USB enumeration (non-blocking with task())
  uint32_t usb_start = millis();
  while (!TinyUSBDevice.mounted() && (millis() - usb_start) < 3000) {
    TinyUSBDevice.task();
    delay(1);
  }

  // --- Load persisted settings ---
  EEPROM.begin();
  PersistedSettings s;
  EEPROM.get(EEPROM_SETTINGS_ADDR, s);
  if (s.magic == EEPROM_MAGIC) {
    led_max = s.led_max;
    if (led_max > LED_MAX_PWM) led_max = LED_MAX_PWM;

    wait_bars = s.wait_bars;
    bool valid_wb = false;
    for (uint8_t wb : wait_options) if (wb == wait_bars) valid_wb = true;
    if (!valid_wb) wait_bars = 4;

    usb_out_channel = s.usb_out_channel;
    if (usb_out_channel > 1) usb_out_channel = 0;
  } else {
    led_max = 107;
    wait_bars = 4;
    usb_out_channel = 0;
  }
  updateTiming();

  applyTempo();
  tickTimer->setInterruptPriority(0, 0);
  tickTimer->attachInterrupt(onTick);
  tickTimer->resume();
}

void loop() {
  TinyUSBDevice.task();
  processUSB_RX();
  processUSB_TX();

  pollButtons();
  pollEncoderButton();
  pollSettingsButton();
  processEncoder();
  updateStatusLeds();

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

  if (save_pending && !running[0] && !running[1] &&
                      !pending[0] && !pending[1]) {
    save_pending = false;

    analogWrite(PIN_LED_BEAT,      0);
    analogWrite(PIN_LED_STATUS[0], 0);
    analogWrite(PIN_LED_STATUS[1], 0);

    PersistedSettings s;
    EEPROM.get(EEPROM_SETTINGS_ADDR, s);
    if (s.magic != EEPROM_MAGIC || s.led_max != led_max ||
        s.wait_bars != wait_bars ||
        s.usb_out_channel != usb_out_channel) {
      PersistedSettings ns = { led_max, wait_bars, usb_out_channel, EEPROM_MAGIC };
      EEPROM.put(EEPROM_SETTINGS_ADDR, ns);
    }
    ui_dirty = true;
  }

  if (ui_dirty) {
    ui_dirty = false;
    drawUI();
  }
}