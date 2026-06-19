/*
 * Raw nRF52840 I2S tone test  -- proof-of-life for the MAX98357A + speaker on
 * the Seeed XIAO nRF52840 Sense.
 *
 * WHY THIS EXISTS: Zephyr's i2s_nrfx driver configures the peripheral perfectly
 * but its clock-picker chooses MCK=div63 / RATIO=32X for 16 kHz -- the fragile
 * edge case that won't actually clock on this silicon. This sketch sets the I2S
 * registers by hand with the clock Nordic's own LE Audio team uses for 16 kHz:
 *   MCKFREQ = 32MDIV21 (1.523 MHz),  RATIO = 96X  ->  LRCK = 15873 Hz.
 * Same output rate the driver wanted, but a healthy MCK and a ratio well clear
 * of 32X. If this plays a tone, the amp/speaker/wiring are good and nrfx-direct
 * is the way -- then we port this exact config into the real Zephyr firmware.
 *
 * BOARD: Seeed XIAO nRF52840 Sense  (FQBN Seeeduino:nrf52:xiaonRF52840Sense).
 *
 * WIRING (msfujino's confirmed-working XIAO I2S pins -- [UNIT], board-specific):
 *   amp BCLK <- D1 / P0.03      amp LRC  <- D3 / P0.29
 *   amp DIN  <- D4 / P0.04      amp SD   <- D7 / P1.12  (HIGH = enabled)
 *   MCK is routed to D0 / P0.02 but the MAX98357A ignores it -- leave unconnected.
 *   amp VIN <- 3V3 (or BAT),  amp GND <- GND.
 *
 * Open Serial Monitor at 115200; you should hear a ~440 Hz tone immediately.
 */
#include <Adafruit_TinyUSB.h>   // required for Serial with the TinyUSB stack
#include <Arduino.h>
#include <nrf.h>
#include <math.h>

// --- audio params ---
static const float    FS    = 15873.0f;   // 32 MHz / 21 / 96  (actual LRCK)
static const float    TONE  = 440.0f;     // test tone [Hz]
static const int      N     = 512;        // frames per DMA buffer (stereo)
static const int16_t  AMP    = 8000;      // tone amplitude (~-12 dBFS, safe)

// Two ping-pong buffers: int16 L,R interleaved. N frames = N*4 bytes = N words.
static int16_t buf[2][N * 2];
static float   phase = 0.0f;
static volatile uint8_t  give  = 1;        // which buffer to queue next (ISR-owned)
static volatile uint32_t txptrupd_count = 0;

static void fillBuf(int16_t *b) {
  const float inc = 2.0f * (float)M_PI * TONE / FS;
  for (int i = 0; i < N; i++) {
    int16_t v = (int16_t)(sinf(phase) * AMP);
    phase += inc;
    if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
    b[2 * i]     = v;   // left
    b[2 * i + 1] = v;   // right
  }
}

/* I2S ISR: the nRF52840 double-buffer STOPS after one buffer unless the next
 * TXD.PTR is reloaded with microsecond latency -- which only an interrupt can
 * guarantee (a polling loop delayed by Serial underruns and halts after 1).
 * Clear the event FIRST (+dummy read) to avoid the documented lockup, then
 * hand the peripheral the next buffer. */
extern "C" void I2S_IRQHandler(void) {
  if (NRF_I2S->EVENTS_TXPTRUPD) {
    NRF_I2S->EVENTS_TXPTRUPD = 0;
    (void)NRF_I2S->EVENTS_TXPTRUPD;       // dummy read: flush the clear (nRF event quirk)

    fillBuf(buf[give]);
    NRF_I2S->TXD.PTR = (uint32_t)buf[give];
    give ^= 1;
    txptrupd_count++;
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) { }
  Serial.println("Raw nRF52840 I2S tone test  (MCK=div21, RATIO=96X, ~15873 Hz)");

  // Force the 32 MHz crystal (HFXO) on -- I2S derives MCK from HFCLK; this
  // removes any doubt about it running on the internal RC instead.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED) { }
  Serial.println("HFXO (32 MHz crystal) running");

  // amp SD on D7 = P1.12 -> drive HIGH to enable the MAX98357A
  NRF_P1->DIRSET = (1u << 12);
  NRF_P1->OUTSET = (1u << 12);

  // --- pin routing (all on port 0; PSEL = pin number, bit31=0 -> connected) ---
  NRF_I2S->PSEL.MCK   = 2;            // D0 / P0.02
  NRF_I2S->PSEL.SCK   = 3;            // D1 / P0.03  (BCLK)
  NRF_I2S->PSEL.LRCK  = 29;           // D3 / P0.29
  NRF_I2S->PSEL.SDOUT = 4;            // D4 / P0.04  (data)
  NRF_I2S->PSEL.SDIN  = 0x80000000u;  // disconnected (TX only)

  // --- format + clock (the hand-picked, known-good combo) ---
  NRF_I2S->CONFIG.MODE     = I2S_CONFIG_MODE_MODE_Master;
  NRF_I2S->CONFIG.RXEN     = I2S_CONFIG_RXEN_RXEN_Disabled;
  NRF_I2S->CONFIG.TXEN     = I2S_CONFIG_TXEN_TXEN_Enabled;
  NRF_I2S->CONFIG.MCKEN    = I2S_CONFIG_MCKEN_MCKEN_Enabled;
  NRF_I2S->CONFIG.MCKFREQ  = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV21;  // 1.523 MHz
  NRF_I2S->CONFIG.RATIO    = I2S_CONFIG_RATIO_RATIO_96X;           // /96 -> 15873 Hz
  NRF_I2S->CONFIG.SWIDTH   = I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
  NRF_I2S->CONFIG.ALIGN    = I2S_CONFIG_ALIGN_ALIGN_Left;
  NRF_I2S->CONFIG.FORMAT   = I2S_CONFIG_FORMAT_FORMAT_I2S;
  NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Stereo;

  fillBuf(buf[0]);
  fillBuf(buf[1]);

  NRF_I2S->RXTXD.MAXCNT = N;            // N 32-bit words per buffer
  NRF_I2S->TXD.PTR      = (uint32_t)buf[0];
  NRF_I2S->EVENTS_TXPTRUPD = 0;

  // Interrupt-driven buffer reload (the proven XIAO recipe). We override the
  // weak I2S_IRQHandler symbol directly (no NVIC_SetVector -- that writes the
  // flash-resident vector table on this core and faults).
  NRF_I2S->INTENSET = I2S_INTENSET_TXPTRUPD_Msk;
  NVIC_SetPriority(I2S_IRQn, 3);
  NVIC_ClearPendingIRQ(I2S_IRQn);
  NVIC_EnableIRQ(I2S_IRQn);

  NRF_I2S->ENABLE       = 1;
  NRF_I2S->TASKS_START  = 1;            // buf[0] first; the ISR feeds the rest

  Serial.println("I2S started (IRQ-driven).");
  Serial.println("BARE-BOARD SELF-TEST: nothing needs to be wired -- just watch the verdict.");
}

static uint32_t last_report_ms = 0;
static bool     verdict_printed = false;

void loop() {
  // The ISR does all the buffer work; loop() just reports. At 15873 Hz /
  // 512-frame buffers the count climbs ~31/sec when the I2S truly clocks.
  uint32_t now = millis();
  if (now - last_report_ms >= 1000) {
    last_report_ms = now;
    Serial.print("TXPTRUPD total: ");
    Serial.println(txptrupd_count);
  }

  // One-shot PASS/FAIL after 3 s -- the whole point of the bare-board test.
  if (!verdict_printed && now > 3000) {
    verdict_printed = true;
    Serial.println("------------------------------------------------------------");
    if (txptrupd_count > 5) {
      Serial.print(">>> PASS: I2S SUSTAINS on this board (count=");
      Serial.print(txptrupd_count);
      Serial.println("). The peripheral WORKS -- the previous XIAO was the problem.");
      Serial.println(">>> I2S is back on the table; MAX98357A is fine. Re-wire the amp to hear sound.");
    } else {
      Serial.print(">>> FAIL: I2S STUCK (count=");
      Serial.print(txptrupd_count);
      Serial.println("). A fresh board doesn't sustain either => systemic, not a dead unit.");
      Serial.println(">>> Lean toward PWM audio for output.");
    }
    Serial.println("------------------------------------------------------------");
  }
}
