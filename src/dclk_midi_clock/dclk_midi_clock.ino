#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/Picopixel.h>
#include <EEPROM.h>

// ---------- Config ----------
constexpr uint8_t  PPQN       = 24;
constexpr uint32_t LOOP_TICKS = PPQN * 4 * 4;  // 4 bars of 4/4 = 384
constexpr uint16_t BPM_MIN = 30, BPM_MAX = 300;

// ---------- LED brightness ----------
// User-facing 100% = 70% PWM duty (deliberately dimmer than full white).
// On each beat while a channel is running, status LEDs jump up by
// FLASH_AMOUNT (20% of full PWM range) then fade back to led_max.
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

// ---------- Persistent settings (emulated EEPROM in flash) ----------
struct PersistedSettings {
  uint8_t led_max;   // 0..LED_MAX_PWM
  // uint8_t midi_channel;   // future
  // uint8_t midi_clock_div; // future
  uint8_t magic;     // marker: 0xA5 if the block has been written at least once
};

constexpr uint16_t EEPROM_SETTINGS_ADDR = 0;
constexpr uint8_t  EEPROM_MAGIC         = 0xA5;

// ---------- State ----------
volatile uint16_t bpm = 120;
volatile bool bpm_changed = true;
volatile bool ui_dirty = true;

volatile bool running[2]  = { false, false };
volatile bool pending[2]  = { false, false };
volatile bool play_req[2] = { false, false };
volatile bool stop_req[2] = { false, false };
volatile uint32_t loop_tick = 0;

volatile uint8_t led_max = 107;  // 60% of LED_MAX_PWM; overwritten by EEPROM load

// EEPROM write is deferred until MIDI is idle: the flash sector erase
// blocks the CPU (and hence the TIM2 clock ISR) for ~1–4 s on the F411.
bool save_pending = false;

// ---------- UI state (loop() context only) ----------
enum class Screen : uint8_t { MAIN, SETTINGS, LED_VIEW, LED_EDIT };
Screen  screen        = Screen::MAIN;
uint8_t menu_sel      = 0;   // 0 = LEDS, 1 = MIDI
uint8_t led_edit_orig = 0;   // value saved when entering LED_EDIT

constexpr uint8_t MENU_COUNT = 2;

// ISR-produced encoder delta, consumed by loop()
volatile int16_t encoder_delta = 0;

// ---------- Timer ISR: one call per MIDI clock tick ----------
void onTick() {
  static bool beatLit = false;

  // 1. Stops (immediate)
  for (int i = 0; i < 2; i++) {
    if (stop_req[i]) {
      stop_req[i] = false;
      if (running[i]) midi[i]->write(0xFC);  // MIDI Stop
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
  bool boundary = !any || (loop_tick % LOOP_TICKS == 0);

  for (int i = 0; i < 2; i++) {
    if (pending[i] && boundary) {
      pending[i] = false;
      running[i] = true;
      midi[i]->write(0xFA);  // MIDI Start
      ui_dirty = true;
    }
  }

  // 4. Send clock to running outputs
  for (int i = 0; i < 2; i++) {
    if (running[i]) midi[i]->write(0xF8);
  }

  // 5. Beat LED + advance loop counter
  if (running[0] || running[1]) {
    uint8_t phase = loop_tick % PPQN;
    if (phase == 0 && !beatLit) {
      analogWrite(PIN_LED_BEAT, led_max);
      beatLit = true;
    } else if (phase == 4 && beatLit) {
      analogWrite(PIN_LED_BEAT, 0);
      beatLit = false;
    }
    loop_tick++;
  } else if (beatLit) {
    analogWrite(PIN_LED_BEAT, 0);
    beatLit = false;
  }
}

void applyTempo() {
  uint32_t us = 60000000UL / ((uint32_t)bpm * PPQN);
  tickTimer->setOverflow(us, MICROSEC_FORMAT);
}

// ---------- Encoder ISR (accumulates a delta) ----------
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

// Consume encoder delta in the current screen's context
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
      while (v < 0) v += MENU_COUNT;
      v %= MENU_COUNT;
      menu_sel = (uint8_t)v;
      ui_dirty = true;
      break;
    }
    case Screen::LED_VIEW:
      break;  // no-op
    case Screen::LED_EDIT: {
      int16_t v = (int16_t)led_max + delta * 5;   // 5-step granularity
      if (v < 0)              v = 0;
      if (v > LED_MAX_PWM)    v = LED_MAX_PWM;
      led_max  = (uint8_t)v;
      ui_dirty = true;
      break;
    }
  }
}

// ---------- Encoder push button (PB13) ----------
void onEncoderButton() {
  switch (screen) {
    case Screen::MAIN:
      break;  // unused on main screen
    case Screen::SETTINGS:
      if (menu_sel == 0) {          // "LEDS"
        screen   = Screen::LED_VIEW;
        ui_dirty = true;
      }
      // "MIDI" item — TODO
      break;
    case Screen::LED_VIEW:
      led_edit_orig = led_max;      // remember for cancel
      screen   = Screen::LED_EDIT;
      ui_dirty = true;
      break;
    case Screen::LED_EDIT:
      // Confirm / save: mark pending. The flash write is deferred to loop()
      // and only performed once nothing is playing, so the MIDI clock ISR
      // is not starved by the multi-second sector erase.
      save_pending = true;
      screen   = Screen::LED_VIEW;
      ui_dirty = true;
      break;
  }
}

void pollEncoderButton() {
  static uint32_t lastMs   = 0;
  static bool     lastState = true;
  bool s = digitalRead(PIN_ENC_BTN);
  if (!s && lastState && millis() - lastMs > 25) {
    lastMs = millis();
    onEncoderButton();
  }
  lastState = s;
}

// ---------- Settings button (PB14): open menu / back one level ----------
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
      led_max  = led_edit_orig;     // cancel edit (no EEPROM write)
      screen   = Screen::LED_VIEW;
      ui_dirty = true;
      break;
  }
}

void pollSettingsButton() {
  static uint32_t lastMs   = 0;
  static bool     lastState = true;
  bool s = digitalRead(PIN_BTN_SETTINGS);
  if (!s && lastState && millis() - lastMs > 25) {
    lastMs = millis();
    onSettingsButton();
  }
  lastState = s;
}

// ---------- MIDI channel buttons ----------
void pollButtons() {
  static uint32_t lastMs[2] = { 0, 0 };
  static bool lastState[2] = { true, true };
  for (int i = 0; i < 2; i++) {
    bool s = digitalRead(PIN_BTN[i]);
    if (!s && lastState[i] && millis() - lastMs[i] > 25) {
      lastMs[i] = millis();
      if (running[i] || pending[i]) stop_req[i] = true;
      else                          play_req[i] = true;
    }
    lastState[i] = s;
  }
}

// ---------- Status LEDs ----------
constexpr uint16_t FADE_PERIOD_MS = 1400;  // pending (WAIT) triangle period

void updateStatusLeds() {
  uint32_t t   = millis() % FADE_PERIOD_MS;
  uint32_t tick = loop_tick;                       // atomic 32-bit read
  bool any_running = running[0] || running[1];

  // Beat-synced flash: peak at phase 0, linear fade to 0 at end of beat.
  // Only computed when something is running — the beat is meaningful only
  // then, and loop_tick doesn't advance otherwise.
  uint8_t flash = 0;
  if (any_running) {
    uint8_t phase = tick % PPQN;                   // 0..23
    flash = (uint32_t)FLASH_AMOUNT * (PPQN - 1 - phase) / (PPQN - 1);
  }

  for (int i = 0; i < 2; i++) {
    uint8_t duty;
    if (running[i]) {
      // Base + beat flash, capped at 255.
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
// Draw a string whose rightmost ink pixel is `rightMargin` px from the display edge.
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
    case Screen::MAIN:
      display.setTextSize(4);
      display.setCursor(6, 26);
      display.print("BPM: ");
      display.print(bpm);
      break;

    case Screen::SETTINGS:
      display.setTextSize(3);
      display.setCursor(0, 12);
      display.print(menu_sel == 0 ? "> LEDS <" : "  LEDS");
      display.setCursor(0, 30);
      display.print(menu_sel == 1 ? "> MIDI <" : "  MIDI");
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

  // --- Load persisted settings from emulated EEPROM ---
  EEPROM.begin();
  PersistedSettings s;
  EEPROM.get(EEPROM_SETTINGS_ADDR, s);
  if (s.magic == EEPROM_MAGIC) {
    led_max = s.led_max;  // first boot keeps the 107 default
  }

  applyTempo();
  tickTimer->setInterruptPriority(0, 0);
  tickTimer->attachInterrupt(onTick);
  tickTimer->resume();
}

void loop() {
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

  // Deferred EEPROM write: only when nothing is playing or pending.
  // The flash sector erase blocks the CPU for ~1-4 s on the F411, which
  // would starve the TIM2 clock ISR and cause a visible MIDI gap.
  if (save_pending && !running[0] && !running[1] &&
                      !pending[0] && !pending[1]) {
    save_pending = false;

    // Explicitly force all LEDs off before the freeze. The PWM timers
    // keep running during the flash erase, so any non-zero duty left
    // here would stay visible for the whole 1-4 s block.
    analogWrite(PIN_LED_BEAT,      0);
    analogWrite(PIN_LED_STATUS[0], 0);
    analogWrite(PIN_LED_STATUS[1], 0);

    PersistedSettings s;
    EEPROM.get(EEPROM_SETTINGS_ADDR, s);
    if (s.magic != EEPROM_MAGIC || s.led_max != led_max) {
      PersistedSettings ns = { led_max, EEPROM_MAGIC };
      EEPROM.put(EEPROM_SETTINGS_ADDR, ns);
    }
    ui_dirty = true;   // refresh screen to clear the "*"
  }

  if (ui_dirty) {
    ui_dirty = false;
    drawUI();
  }
}