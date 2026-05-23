// ATtiny85 (bare, no bootloader) – DRV8212 Single-Channel Forward PWM Motor Driver
//
// HARDWARE
//   MCU    : Bare ATtiny85 @ 8 MHz internal oscillator (ISP-flashed, no bootloader)
//   Driver : DRV8212 breakout, two-input variant (IN1/IN2 → OUT1/OUT2)
//            DRV8837 clone acceptable for prototyping – identical control interface
//   Motor  : 3 V, 1 A DC motor
//            Note: DRV8212 contains integrated body diode recirculation paths.
//            An external Schottky across the motor terminals (not the bridge)
//            protects VM from back-EMF spikes on disconnect – optional but prudent.
//   Control: 10 kΩ linear pot with integral SPST power switch
//            Switch cuts the entire supply – firmware needs no sleep logic.
//
// PINOUT (ATtiny85 physical DIP-8 pin → port bit)
//   Pin 5  PB0 / OC0A  →  DRV8212 IN1   (PWM output)
//   Pin 6  PB1         →  DRV8212 IN2   (held LOW = forward / coast on 0 PWM)
//   Pin 7  PB2 / ADC1  →  Pot wiper     (add 100 nF cap wiper-to-GND)
//
// DRV8212 TRUTH TABLE (two-input variant, one H-bridge channel)
//   IN1   IN2   →  Output
//   PWM   LOW   →  Forward PWM   (fast decay)
//   LOW   LOW   →  Coast         (both outputs float)
//   HIGH  HIGH  →  Brake         (not used here)
//
// POWER
//   Li-ion (+) → ATtiny85 VCC (pin 8)
//   Li-ion (–) → ATtiny85 GND (pin 4)
//   DRV8212 VM → same Li-ion (+) rail
//   All GNDs tied together.
//   DRV8212 nSLEEP → tie directly to VCC on the board (no internal pullup).
//
// PWM
//   Timer0, Fast PWM, OC0A non-inverting, prescaler = 1
//   f_PWM = F_CPU / (1 * 256) = 8 MHz / 256 = 31.25 kHz  (Fast PWM, 8-bit single-slope)
//   Timer0 runs continuously in IDLE sleep – PWM output is unaffected between bursts.
//
// OVERSAMPLING
//   64 samples using ADC Noise Reduction sleep between conversions.
//   Sum fits in uint16_t (max = 64 * 1023 = 65472, no overflow).
//   Right-shifted by 3 → 13-bit effective result (0–8184).
//   Note: actual ENOB gain depends on sufficient analog dither at the ADC input.
//   With a filtered pot and stable VCC, gain is primarily noise reduction rather
//   than true 13-bit accuracy. Still beneficial; comment reflects reality.
//   Mapped to OCR0A (0–255). Hysteresis of ±2 PWM counts suppresses residual flutter.
//
// POWER MANAGEMENT
//   Between oversample bursts (~243 ms): SLEEP_MODE_IDLE
//     CPU halted. Timer0 keeps running → PWM output uninterrupted. ~2 mA.
//   During oversample burst (~6.7 ms): SLEEP_MODE_ADC (ADC Noise Reduction)
//     CPU and Timer0 halted between conversions. ADC clock runs. ~0.5 mA.
//     Timer0 halt causes a ~6.7 ms PWM gap per 250 ms cycle (2.7% of time).
//     Motor inertia bridges this gap cleanly at normal operating speeds.
//   WDT fires every 250 ms in interrupt-only mode to pace oversample bursts.
//   Average current: ~2 mA vs ~3.3 mA always-on. Modest but real saving.
//   For maximum battery life at the cost of ~6.7 ms PWM gaps every 250 ms,
//   replace IDLE sleep with POWER_DOWN and disable ADC between bursts.
//
// NOTE: No Arduino framework. Uses direct AVR-libc only.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// ---------------------------------------------------------------------------
// Pin assignments (ATtiny85 PB port bits)
// ---------------------------------------------------------------------------
#define PIN_IN1    PB0   // OC0A – PWM to DRV8212 IN1
#define PIN_IN2    PB1   // static LOW (forward direction)
#define ADC_CH     1     // ADC1 on PB2 (physical pin 7)

// ---------------------------------------------------------------------------
// Oversampling
// ---------------------------------------------------------------------------
// N=64 → +3 bits → 13-bit effective resolution. uint16_t holds 64×1023=65472.
#define OS_SAMPLES  64
#define OS_SHIFT    3      // sum >> 3 reduces 16-bit accumulator to 13-bit result
#define OS_MAX      8184   // 1023 * 64 >> 3  (13-bit full-scale)

// ---------------------------------------------------------------------------
// Dead zones (scaled from 10-bit thresholds to 13-bit space)
//   POT_DEAD_LOW  = round(100  * 8184 / 1023) = 800
//   POT_DEAD_HIGH = round(1011 * 8184 / 1023) = 8088
// ---------------------------------------------------------------------------
#define POT_DEAD_LOW   800
#define POT_DEAD_HIGH  8088

// ---------------------------------------------------------------------------
// Hysteresis: OCR0A only updates when new PWM value differs by more than
// this many counts (0–255 scale). Suppresses last-bit flutter.
// ---------------------------------------------------------------------------
#define HYST_COUNTS  2

// ---------------------------------------------------------------------------
// WDT period between oversample bursts.
// WDTO_250MS → ~4 Hz update rate, ~2 mA average current.
// WDTO_500MS → ~2 Hz, slightly lower average current, slightly slower response.
// ---------------------------------------------------------------------------
#define WDT_PERIOD  WDTO_250MS

// ---------------------------------------------------------------------------
// Flag set by WDT ISR. Consumed and cleared atomically in main loop.
// ---------------------------------------------------------------------------
static volatile uint8_t wdt_fired = 0;

// ---------------------------------------------------------------------------
// WDT ISR – fires every WDT_PERIOD, wakes CPU from IDLE sleep.
// ---------------------------------------------------------------------------
ISR(WDT_vect) {
    wdt_fired = 1;
}

// ---------------------------------------------------------------------------
// ADC complete ISR – wakes CPU from ADC Noise Reduction sleep.
// Result is read directly from ADC register in mainline after wake;
// no shared variable needed, eliminating any read-modify race.
// ---------------------------------------------------------------------------
ISR(ADC_vect) {
    // intentionally empty – wake only
}

// ---------------------------------------------------------------------------
// Configure WDT for interrupt-only mode (no system reset) at WDT_PERIOD.
// Uses timed write sequence per ATtiny85 datasheet §8.4.
// ---------------------------------------------------------------------------
static void wdt_init(void) {
    cli();
    wdt_reset();
    WDTCR = (1 << WDCE) | (1 << WDE);            // enter configuration mode
    WDTCR = (1 << WDIE) | (WDT_PERIOD & 0x27);   // interrupt-only + period
    sei();
}

// ---------------------------------------------------------------------------
// 64-sample oversampled ADC read using ADC Noise Reduction sleep between
// conversions. Returns 13-bit result (0–8184).
//
// ADC Noise Reduction mode halts the CPU and most peripherals (including
// Timer0) during each conversion, reducing switching noise on the ADC supply.
// Each conversion takes ~104 µs at 125 kHz ADC clock. Total burst ~6.7 ms.
// ---------------------------------------------------------------------------
static uint16_t readADC_oversampled(void) {
    uint16_t sum = 0;

    ADCSRA |= (1 << ADIE);    // enable ADC-complete interrupt (required to wake)

    for (uint8_t i = 0; i < OS_SAMPLES; i++) {
        ADCSRA |= (1 << ADSC);           // start conversion

        // Atomically enable sleep and enter ADC Noise Reduction mode.
        // cli/sei bracketing prevents a race where the ADC-complete ISR
        // fires between sleep_enable() and sleep_cpu(), which would leave
        // the CPU sleeping with no pending interrupt to wake it.
        set_sleep_mode(SLEEP_MODE_ADC);
        cli();
        sleep_enable();
        sei();
        sleep_cpu();                      // woken by ADC_vect (empty ISR)
        sleep_disable();

        sum += ADC;                       // read result directly – no shared state
    }

    ADCSRA &= ~(1 << ADIE);   // disable ADC-complete interrupt between bursts

    return sum >> OS_SHIFT;
}

// ---------------------------------------------------------------------------
// Minimal integer map – avoids overflow via int32_t intermediate.
// ---------------------------------------------------------------------------
static inline int32_t map_val(int32_t x,
                               int32_t in_min, int32_t in_max,
                               int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ---------------------------------------------------------------------------
int main(void) {

    // Disable digital input buffer on ADC pin – saves a few µA
    DIDR0 |= (1 << ADC1D);

    // Set IN1 and IN2 as outputs
    DDRB |= (1 << PIN_IN1) | (1 << PIN_IN2);

    // IN2 held LOW throughout – selects forward / coast mode on DRV8212
    PORTB &= ~(1 << PIN_IN2);

    // --- Timer0: Fast PWM, non-inverting on OC0A, prescaler = 1 ----------
    // WGM01|WGM00 = Fast PWM mode (8-bit single-slope)
    // COM0A1      = clear OC0A on compare match, set at BOTTOM (non-inv)
    // CS00        = clk/1  →  f_PWM = 8 MHz / 256 = 31.25 kHz
    TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0A1);
    TCCR0B = (1 << CS00);
    OCR0A  = 0;   // Motor off until pot is read

    // --- ADC: VCC reference, right-adjust result, channel 1 --------------
    // REFS0 = 0  →  VCC reference (correct for pot ratiometric to VCC)
    // MUX   = ADC_CH (ADC1 / PB2)
    // Prescaler /64  →  8 MHz / 64 = 125 kHz (within 50–200 kHz ADC spec)
    ADMUX  = (ADC_CH & 0x07);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);

    // Throwaway conversion: first result after ADC enable is unreliable
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));

    wdt_init();

    uint8_t last_pwm = 0;

    // ---------------------------------------------------------------------------
    // Main loop
    //   Sleeps in IDLE between WDT wakeups – Timer0 runs, PWM uninterrupted.
    //   On WDT wakeup: runs 64-sample ADC NR oversample burst (~6.7 ms).
    //   Updates OCR0A only if PWM value moved outside hysteresis band.
    // ---------------------------------------------------------------------------
    while (1) {

        // Sleep in IDLE until WDT fires. Timer0 keeps running → PWM unaffected.
        // Atomically arm sleep to prevent race if WDT fires before sleep_cpu().
        set_sleep_mode(SLEEP_MODE_IDLE);
        cli();
        sleep_enable();
        sei();
        sleep_cpu();                      // woken by WDT_vect (or any other interrupt)
        sleep_disable();

        // Guard against spurious wakeups (ADC-complete leaking through, etc.)
        if (!wdt_fired) continue;
        cli();
        wdt_fired = 0;
        sei();

        uint16_t os = readADC_oversampled();

        uint8_t pwm;
        if (os <= POT_DEAD_LOW) {
            pwm = 0;
        } else if (os >= POT_DEAD_HIGH) {
            pwm = 255;
        } else {
            pwm = (uint8_t) map_val(os, POT_DEAD_LOW, POT_DEAD_HIGH, 1, 254);
        }

        // Update OCR0A only if outside hysteresis band.
        // Cast to int16_t handles subtraction safely across the 0 boundary.
        if ((int16_t)pwm - (int16_t)last_pwm > HYST_COUNTS ||
            (int16_t)last_pwm - (int16_t)pwm > HYST_COUNTS) {
            OCR0A    = pwm;
            last_pwm = pwm;
        }
    }

    return 0;   // never reached
}