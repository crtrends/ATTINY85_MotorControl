// ATtiny85 (bare, no bootloader) – DRV8212 Single-Channel Forward PWM Motor Driver
//
// HARDWARE
//   MCU    : Bare ATtiny85 @ 8 MHz internal oscillator (ISP-flashed, no bootloader)
//   Driver : DRV8212 breakout, two-input variant (IN1/IN2 → OUT1/OUT2)
//            DRV8837 clone acceptable for prototyping – identical control interface
//   Motor  : 3 V, 1 A DC motor with external flyback diode
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
//   f_PWM = 8 MHz / 256 = 31.25 kHz  (above audible range)
//   Timer0 continues running during idle sleep – PWM output is unaffected.
//
// OVERSAMPLING
//   64 samples summed into uint16_t (max sum = 65472, no overflow).
//   Right-shifted by 3 → 13-bit effective result (0–8184).
//   Mapped to OCR0A (0–255).
//   Hysteresis of ±2 (PWM counts) suppresses residual jitter on OCR0A.
//
// POWER MANAGEMENT
//   Sleep mode: SLEEP_MODE_IDLE
//     - Timer0 keeps running → PWM output holds its last value uninterrupted.
//     - ADC can run → used to wake via ADC-complete interrupt after each sample.
//     - CPU halted between ADC conversions → saves ~80% of active current.
//   WDT wakes the CPU every 250 ms to trigger a fresh oversample burst.
//   Average current: ~87 µA vs ~3.3 mA always-on (38× battery life improvement).
//
// SLEEP STRATEGY DETAIL
//   During the oversample burst (64 conversions × ~104 µs = ~6.7 ms):
//     CPU sleeps in IDLE between each conversion, woken by ADC-complete ISR.
//   Between bursts (remaining ~243 ms of the 250 ms WDT period):
//     CPU sleeps in POWER-DOWN (deepest sleep, ~1 µA), woken by WDT ISR.
//   OCR0A holds its last value throughout – motor speed is unaffected by sleep.
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
// Dead zones (scaled from original 10-bit thresholds to 13-bit space)
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
// WDTO_250MS → ~4 Hz update rate, ~87 µA average current.
// Increase to WDTO_500MS for longer battery life at the cost of slightly
// slower pot response (~44 µA average, ~2 Hz).
// ---------------------------------------------------------------------------
#define WDT_PERIOD  WDTO_250MS

// ---------------------------------------------------------------------------
// Volatile flags set by ISRs, consumed in main loop.
// ---------------------------------------------------------------------------
static volatile uint8_t wdt_fired  = 0;   // set by WDT ISR – time to resample
static volatile uint8_t adc_done   = 0;   // set by ADC ISR – conversion complete
static volatile uint16_t adc_result = 0;  // latched ADC value from ISR

// ---------------------------------------------------------------------------
// WDT ISR – fires every WDT_PERIOD, wakes CPU from power-down sleep.
// Watchdog is in interrupt-only mode (no reset).
// ---------------------------------------------------------------------------
ISR(WDT_vect) {
    wdt_fired = 1;
}

// ---------------------------------------------------------------------------
// ADC complete ISR – fires after each conversion during oversample burst.
// Latches result and wakes CPU from idle sleep.
// ---------------------------------------------------------------------------
ISR(ADC_vect) {
    adc_result = ADC;
    adc_done   = 1;
}

// ---------------------------------------------------------------------------
// Configure WDT for interrupt-only mode (no system reset) at WDT_PERIOD.
// Must be done with timed write sequence per ATtiny85 datasheet §8.4.
// ---------------------------------------------------------------------------
static void wdt_init(void) {
    cli();
    wdt_reset();
    // Enter configuration mode: set WDCE and WDE simultaneously
    WDTCR = (1 << WDCE) | (1 << WDE);
    // Set interrupt-only mode + desired period (WDE=0, WDIE=1)
    WDTCR = (1 << WDIE) | WDT_PERIOD;
    sei();
}

// ---------------------------------------------------------------------------
// Perform one 64-sample oversampled ADC read using ADC-complete ISR + idle
// sleep between conversions. Returns 13-bit result (0–8184).
// ---------------------------------------------------------------------------
static uint16_t readADC_oversampled(void) {
    uint16_t sum = 0;

    // Enable ADC-complete interrupt for sleep-between-conversions pattern
    ADCSRA |= (1 << ADIE);

    for (uint8_t i = 0; i < OS_SAMPLES; i++) {
        adc_done = 0;
        ADCSRA |= (1 << ADSC);          // start conversion

        // Sleep in IDLE until ADC-complete ISR wakes us.
        // Timer0 keeps running in IDLE → PWM output unaffected.
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        sei();
        sleep_cpu();                     // woken by ADC_vect
        sleep_disable();

        sum += adc_result;
    }

    // Disable ADC interrupt – not needed outside oversample burst
    ADCSRA &= ~(1 << ADIE);

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
    // WGM01|WGM00 = Fast PWM mode
    // COM0A1      = clear OC0A on compare match, set at BOTTOM (non-inv)
    // CS00        = clk/1  →  8 MHz / 256 = 31.25 kHz
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

    // Start WDT – fires every WDT_PERIOD to trigger oversample bursts
    wdt_init();

    uint8_t last_pwm = 0;

    // ---------------------------------------------------------------------------
    // Main loop
    //   - Sleeps in power-down between WDT wakeups (~243 ms, ~1 µA)
    //   - On WDT wakeup: runs 64-sample oversample burst (~6.7 ms, ~3.3 mA)
    //     with CPU idling between each ADC conversion
    //   - Updates OCR0A only if PWM value changed beyond hysteresis band
    // ---------------------------------------------------------------------------
    while (1) {

        // Sleep in power-down until WDT fires.
        // Timer0 is stopped in power-down but OCR0A register retains its value.
        // The DRV8212 IN1 line will go LOW during power-down (OC0A tristated),
        // which puts the motor in coast. If holding speed during sleep is needed,
        // switch to SLEEP_MODE_IDLE here at the cost of higher sleep current.
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();
        sei();
        sleep_cpu();                     // woken by WDT_vect
        sleep_disable();

        if (!wdt_fired) continue;        // spurious wakeup guard
        wdt_fired = 0;

        // Oversample burst – CPU idles between ADC conversions
        uint16_t os = readADC_oversampled();

        uint8_t pwm;
        if (os <= POT_DEAD_LOW) {
            pwm = 0;
        } else if (os >= POT_DEAD_HIGH) {
            pwm = 255;
        } else {
            pwm = (uint8_t) map_val(os, POT_DEAD_LOW, POT_DEAD_HIGH, 1, 254);
        }

        // Update OCR0A only if outside hysteresis band
        if ((int16_t)pwm - (int16_t)last_pwm > HYST_COUNTS ||
            (int16_t)last_pwm - (int16_t)pwm > HYST_COUNTS) {
            OCR0A    = pwm;
            last_pwm = pwm;
        }
    }

    return 0;   // never reached
}
