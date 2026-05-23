// ATtiny85 (bare, no bootloader) – DRV8212 Single-Channel Forward PWM Motor Driver
//
// ARDUINO IDE SETUP
//   Board package : "ATtiny" by David A. Mellis
//                   Add via File → Preferences → Additional Boards Manager URLs:
//                   https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json
//   Tools → Board     : ATtiny25/45/85
//   Tools → Processor : ATtiny85
//   Tools → Clock     : 8 MHz (internal)
//   Tools → Programmer: Arduino as ISP
//   First flash only  : Tools → Burn Bootloader
//                       (sets fuses, writes no bootloader)
//   Every flash after : Sketch → Upload Using Programmer
//
// HARDWARE
//   MCU    : Bare ATtiny85 @ 8 MHz internal oscillator
//   Driver : DRV8212 breakout, two-input variant (IN1/IN2 → OUT1/OUT2)
//            DRV8837 clone acceptable for prototyping
//   Motor  : 3 V DC motor
//   Control: 10 kΩ linear pot with integral SPST power switch
//
// PINOUT (ATtiny85 physical DIP-8 pin → port bit)
//   Pin 5  PB0 / OC0A  → DRV8212 IN1   (PWM output)
//   Pin 6  PB1         → DRV8212 IN2   (held LOW = forward / coast)
//   Pin 7  PB2 / ADC1  → Pot wiper     (100 nF wiper-to-GND recommended)
//
// DRV8212 TRUTH TABLE (two-input variant)
//   IN1   IN2   →  Output
//   PWM   LOW   →  Forward PWM   (fast decay)
//   LOW   LOW   →  Coast
//   HIGH  HIGH  →  Brake         (unused)
//
// POWER
//   Li-ion (+) → ATtiny85 VCC (pin 8)
//   Li-ion (–) → ATtiny85 GND (pin 4)
//   DRV8212 VM → same Li-ion (+) rail
//   All grounds common.
//   DRV8212 nSLEEP tied directly to VCC.
//
// PWM
//   Timer0 Fast PWM, OC0A non-inverting, prescaler = 1
//   f_PWM = F_CPU / (1 × 256)
//         = 8 MHz / 256
//         = 31.25 kHz
//
//   Timer0 runs continuously.
//   PWM output is never interrupted by sleep.
//
// CONTROL LOOP
//   Timer1 generates a periodic interrupt at ~62.5 Hz.
//   Main loop sleeps in IDLE mode between updates.
//   Timer0 continues running during IDLE sleep.
//
// ADC OVERSAMPLING
//   16 samples averaged per update.
//   Oversampling improves noise reduction and may improve effective
//   resolution if sufficient analog noise/dither exists.
//
// POWER
//   CPU sleeps in IDLE between update intervals.
//   Timer0 PWM remains active continuously.
//   ADC disabled outside conversion burst to reduce current.
//
// NOTE
//   Timer0 is reconfigured from Arduino-core defaults.
//   millis(), delay(), tone(), and analogWrite() timing assumptions
//   are therefore invalid and intentionally unused.

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------
#define PIN_IN1    PB0
#define PIN_IN2    PB1
#define ADC_CH     1

// ---------------------------------------------------------------------------
// Oversampling
// ---------------------------------------------------------------------------
#define OS_SAMPLES  16
#define OS_SHIFT    2

// 16 × 1023 >> 2 = 4092
#define OS_MAX      4092

// ---------------------------------------------------------------------------
// Pot dead zones (12-bit oversampled space)
// ---------------------------------------------------------------------------
#define POT_DEAD_LOW   400
#define POT_DEAD_HIGH  4047

// ---------------------------------------------------------------------------
// PWM hysteresis
// ---------------------------------------------------------------------------
#define HYST_COUNTS    1

// ---------------------------------------------------------------------------
// Update rate
//
// Timer1:
//   8 MHz / 64 prescaler = 125 kHz
//   OCR1C = 1999
//
//   125000 / (1999 + 1) = 62.5 Hz
// ---------------------------------------------------------------------------
static volatile uint8_t update_flag = 0;

// ---------------------------------------------------------------------------
// Timer1 compare ISR
// ---------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
    update_flag = 1;
}

// ---------------------------------------------------------------------------
// Minimal integer map()
// ---------------------------------------------------------------------------
static inline uint8_t map_u16_to_u8(
    uint16_t x,
    uint16_t in_min,
    uint16_t in_max,
    uint8_t out_min,
    uint8_t out_max)
{
    return (uint32_t)(x - in_min) *
           (out_max - out_min) /
           (in_max - in_min) +
           out_min;
}

// ---------------------------------------------------------------------------
// Single ADC conversion
//
// Uses IDLE sleep so Timer0 PWM continues uninterrupted.
// ---------------------------------------------------------------------------
static uint16_t adc_read_sleep(void)
{
    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC))
    {
        set_sleep_mode(SLEEP_MODE_IDLE);

        cli();
        sleep_enable();
        sei();

        sleep_cpu();

        sleep_disable();
    }

    return ADC;
}

// ---------------------------------------------------------------------------
// Oversampled ADC read
// ---------------------------------------------------------------------------
static uint16_t readADC_oversampled(void)
{
    uint16_t sum = 0;

    // Enable ADC
    ADCSRA |= (1 << ADEN);

    // Throwaway conversion after enabling ADC
    adc_read_sleep();

    for (uint8_t i = 0; i < OS_SAMPLES; i++)
    {
        sum += adc_read_sleep();
    }

    // Disable ADC outside burst to save current
    ADCSRA &= ~(1 << ADEN);

    return (sum >> OS_SHIFT);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup(void)
{
    // Disable digital input buffer on ADC pin
    DIDR0 |= (1 << ADC1D);

    // -----------------------------------------------------------------------
    // GPIO
    // -----------------------------------------------------------------------
    DDRB |= (1 << PIN_IN1) | (1 << PIN_IN2);

    // IN2 LOW = forward direction
    PORTB &= ~(1 << PIN_IN2);

    // -----------------------------------------------------------------------
    // Timer0 PWM
    // -----------------------------------------------------------------------
    TCCR0A =
        (1 << WGM01) |
        (1 << WGM00) |
        (1 << COM0A1);

    TCCR0B =
        (1 << CS00);

    OCR0A = 0;

    // -----------------------------------------------------------------------
    // ADC
    //
    // VCC reference
    // ADC1 input
    // ADC clock = 8 MHz / 64 = 125 kHz
    // -----------------------------------------------------------------------
    ADMUX =
        (ADC_CH & 0x07);

    ADCSRA =
        (1 << ADPS2) |
        (1 << ADPS1);

    // -----------------------------------------------------------------------
    // Timer1 periodic interrupt (~62.5 Hz)
    //
    // ATtiny85 Timer1 is high-speed and slightly unusual.
    // CTC mode with OCR1C as TOP.
    // -----------------------------------------------------------------------
    TCCR1 =
        (1 << CTC1) |
        (1 << CS12) |
        (1 << CS11);

    GTCCR = 0;

    OCR1C = 1999;
    OCR1A = 1999;

    TIMSK |= (1 << OCIE1A);

    sei();
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop(void)
{
    static uint8_t last_pwm = 0;

    // Sleep until Timer1 update interrupt fires
    while (!update_flag)
    {
        set_sleep_mode(SLEEP_MODE_IDLE);

        cli();
        sleep_enable();
        sei();

        sleep_cpu();

        sleep_disable();
    }

    cli();
    update_flag = 0;
    sei();

    // -----------------------------------------------------------------------
    // Read potentiometer
    // -----------------------------------------------------------------------
    uint16_t os = readADC_oversampled();

    // -----------------------------------------------------------------------
    // Map to PWM
    // -----------------------------------------------------------------------
    uint8_t pwm;

    if (os <= POT_DEAD_LOW)
    {
        pwm = 0;
    }
    else if (os >= POT_DEAD_HIGH)
    {
        pwm = 255;
    }
    else
    {
        pwm = map_u16_to_u8(
            os,
            POT_DEAD_LOW,
            POT_DEAD_HIGH,
            1,
            254);
    }

    // -----------------------------------------------------------------------
    // Hysteresis suppresses PWM flutter
    // -----------------------------------------------------------------------
    if ((int16_t)pwm - (int16_t)last_pwm > HYST_COUNTS ||
        (int16_t)last_pwm - (int16_t)pwm > HYST_COUNTS)
    {
        OCR0A = pwm;
        last_pwm = pwm;
    }
}