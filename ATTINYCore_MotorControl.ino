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
#define OS_SAMPLES     16
#define OS_SHIFT       2

// ---------------------------------------------------------------------------
// Pot dead zones (12-bit oversampled space)
// ---------------------------------------------------------------------------
#define POT_DEAD_LOW   400
#define POT_DEAD_HIGH  4047

// ---------------------------------------------------------------------------
// PWM hysteresis
// ---------------------------------------------------------------------------
#define PWM_HYST       2

// ---------------------------------------------------------------------------
// Timer1 update rate
//
// ATtiny85 Timer1 is 8-bit.
//
// Timer1 clock:
//   8 MHz / 256 prescaler = 31.25 kHz
//
// OCR1C = 255:
//
//   31250 / 256 = 122.07 Hz interrupt rate
//
// Software divides by 2:
//   ~61 Hz control loop update rate
// ---------------------------------------------------------------------------
static volatile uint8_t update_flag = 0;

// ---------------------------------------------------------------------------
// Timer1 Compare A ISR
// ---------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
    static uint8_t divider = 0;

    divider ^= 1;

    if (divider)
    {
        update_flag = 1;
    }
}

// ---------------------------------------------------------------------------
// Empty ADC ISR
//
// Used only to wake CPU from sleep during ADC conversion.
// ---------------------------------------------------------------------------
ISR(ADC_vect)
{
}

// ---------------------------------------------------------------------------
// Integer mapping helper
// ---------------------------------------------------------------------------
static inline uint8_t map_u16_to_u8(uint16_t x,
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
// Sleep once in IDLE mode
//
// Timer0 PWM continues running during IDLE sleep.
// cli/sei ordering prevents AVR sleep race condition.
// ---------------------------------------------------------------------------
static inline void sleep_idle_once(void)
{
    set_sleep_mode(SLEEP_MODE_IDLE);

    cli();
    sleep_enable();
    sei();

    sleep_cpu();

    sleep_disable();
}

// ---------------------------------------------------------------------------
// Perform one ADC conversion while sleeping in IDLE mode
//
// IDLE sleep keeps Timer0 PWM running continuously.
// ADC interrupt wakes CPU on conversion completion.
// ---------------------------------------------------------------------------
static uint16_t adc_read_once_sleep(void)
{
    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC))
    {
        sleep_idle_once();
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

    // Enable ADC-complete interrupt
    ADCSRA |= (1 << ADIE);

    // Throwaway conversion after ADC enable
    adc_read_once_sleep();

    // Oversample burst
    for (uint8_t i = 0; i < OS_SAMPLES; i++)
    {
        sum += adc_read_once_sleep();
    }

    // Disable ADC interrupt
    ADCSRA &= ~(1 << ADIE);

    // Disable ADC outside conversion burst
    ADCSRA &= ~(1 << ADEN);

    return (sum >> OS_SHIFT);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup(void)
{
    cli();

    // Disable digital input buffer on ADC pin
    DIDR0 |= (1 << ADC1D);

    // -----------------------------------------------------------------------
    // GPIO
    // -----------------------------------------------------------------------
    DDRB |= (1 << PIN_IN1) | (1 << PIN_IN2);

    // IN1 low initially
    PORTB &= ~(1 << PIN_IN1);

    // IN2 low permanently (forward/coast mode)
    PORTB &= ~(1 << PIN_IN2);

    // -----------------------------------------------------------------------
    // Timer0 Fast PWM
    //
    // Fast PWM, non-inverting, clk/1
    //
    // f_PWM = 8 MHz / 256 = 31.25 kHz
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
    // Timer1 periodic interrupt
    //
    // Timer1:
    //   clock = 8 MHz / 256 = 31.25 kHz
    //   TOP   = 255
    //
    // Interrupt frequency:
    //   31250 / 256 = 122 Hz
    //
    // ISR divides by 2:
    //   ~61 Hz control updates
    // -----------------------------------------------------------------------
    TCCR1 =
        (1 << CTC1) |
        (1 << CS13);

    OCR1C = 255;
    OCR1A = 255;

    TIMSK |= (1 << OCIE1A);

    sei();
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop(void)
{
    static uint8_t last_pwm = 0;

    // -----------------------------------------------------------------------
    // Sleep until next control-loop update
    // -----------------------------------------------------------------------
    while (!update_flag)
    {
        sleep_idle_once();
    }

    cli();
    update_flag = 0;
    sei();

    // -----------------------------------------------------------------------
    // Read potentiometer
    // -----------------------------------------------------------------------
    uint16_t os = readADC_oversampled();

    // -----------------------------------------------------------------------
    // Map ADC value to PWM
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
    // Hysteresis suppresses last-bit flutter
    // -----------------------------------------------------------------------
    if ((int16_t)pwm - (int16_t)last_pwm > PWM_HYST ||
        (int16_t)last_pwm - (int16_t)pwm > PWM_HYST)
    {
        OCR0A = pwm;
        last_pwm = pwm;
    }
}