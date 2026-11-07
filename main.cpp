/*
 * Autonomous mecanum car - bare-metal port for ATmega328PB
 * No Arduino core: avr-libc only.
 *
 * Assumptions
 *   - 16 MHz external crystal (fuses set accordingly)
 *   - Arduino-Uno-style wiring (same D-pin numbers as the original sketch)
 *   - Keyestudio mecanum base board: STC8G1K08 motor controller reached over a
 *     bit-banged I2C bus on D3 (SDA) / D2 (SCL), registers 1..8 = PWM 0..255
 *
 * Build
 *   avr-g++ -mmcu=atmega328pb -DF_CPU=16000000UL -Os -std=gnu++17 -Wall -o car.elf main.cpp
 *   avr-objcopy -O ihex -R .eeprom car.elf car.hex
 *   avrdude -c <programmer> -p m328pb -U flash:w:car.hex:i
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

// ═════════════════════════════════════════════════════════════════════════════
// Configuration (same values as the Arduino sketch)
// ═════════════════════════════════════════════════════════════════════════════

// Distance thresholds [cm]
#define DIST_EMERGENCY   15
#define DIST_AVOID       10
#define DIST_WARN        50
#define DIST_CLEAR       60

// Speeds (PWM 0..255)
#define SPEED_CRUISE     65
#define SPEED_APPROACH   50
#define SPEED_MANEUVER   45
#define SPEED_SLOW       35
#define SPEED_BACK       43

#define TURN_BOOST       18

// Timing
#define BACK_DURATION_MS    700
#define TURN_DURATION_MS    450
#define STUCK_TIMEOUT_MS   4000
#define STUCK_DELTA        2.5f
#define LOOP_DELAY_MS         5

#define ECHO_TIMEOUT_US   30000u   // same as pulseIn(..., 30000)

// Motor controller I2C address (7-bit). VERIFY: take it from Writebyte() in
// MecanumCar_v2.cpp, or run an I2C scan on the board.
#define MOTOR_I2C_ADDR     0x30
#define I2C_HALF_BIT_US       5     // ~100 kHz bit-banged bus

// ═════════════════════════════════════════════════════════════════════════════
// GPIO
// Arduino pin -> ATmega328PB port mapping
//   D12 = PB4 TRIG_FRONT    D13 = PB5 ECHO_FRONT
//   D7  = PD7 TRIG_LEFT     D6  = PD6 ECHO_LEFT
//   D8  = PB0 TRIG_RIGHT    D11 = PB3 ECHO_RIGHT
//   D3  = PD3 I2C SDA       D2  = PD2 I2C SCL
// ═════════════════════════════════════════════════════════════════════════════

struct GpioPin {
    volatile uint8_t* port;
    volatile uint8_t* ddr;
    volatile uint8_t* pin;
    uint8_t           mask;
};

static inline void pinHigh  (const GpioPin& p) { *p.port |=  p.mask; }
static inline void pinLow   (const GpioPin& p) { *p.port &= ~p.mask; }
static inline bool pinRead  (const GpioPin& p) { return (*p.pin & p.mask) != 0; }
static inline void pinOutput(const GpioPin& p) { *p.ddr  |=  p.mask; }
static inline void pinInput (const GpioPin& p) { *p.ddr  &= ~p.mask; *p.port &= ~p.mask; }

struct Ultrasonic {
    GpioPin trig;
    GpioPin echo;
};

static const Ultrasonic US_FRONT = { { &PORTB, &DDRB, &PINB, _BV(PB4) },
                                     { &PORTB, &DDRB, &PINB, _BV(PB5) } };
static const Ultrasonic US_LEFT  = { { &PORTD, &DDRD, &PIND, _BV(PD7) },
                                     { &PORTD, &DDRD, &PIND, _BV(PD6) } };
static const Ultrasonic US_RIGHT = { { &PORTB, &DDRB, &PINB, _BV(PB0) },
                                     { &PORTB, &DDRB, &PINB, _BV(PB3) } };

static const GpioPin I2C_SDA = { &PORTD, &DDRD, &PIND, _BV(PD3) };
static const GpioPin I2C_SCL = { &PORTD, &DDRD, &PIND, _BV(PD2) };

// ═════════════════════════════════════════════════════════════════════════════
// Timebase
//   Timer0: 1 ms tick -> millis()
//   Timer1: free-running at 2 MHz (0.5 us/tick) -> echo pulse measurement
// ═════════════════════════════════════════════════════════════════════════════

static volatile uint32_t g_millis = 0;

ISR(TIMER0_COMPA_vect) {
    g_millis++;
}

static void timebase_init() {
    TCCR0A = _BV(WGM01);               // CTC
    TCCR0B = _BV(CS01) | _BV(CS00);    // clk/64 = 250 kHz
    OCR0A  = 249;                      // 250 kHz / 250 = 1 kHz
    TIMSK0 = _BV(OCIE0A);

    TCCR1A = 0;                        // normal mode
    TCCR1B = _BV(CS11);                // clk/8 = 2 MHz
}

static uint32_t millis() {
    uint8_t sreg = SREG;
    cli();
    uint32_t m = g_millis;
    SREG = sreg;
    return m;
}

static void delay_ms(uint16_t ms) {
    while (ms--) _delay_ms(1);
}

// Equivalent of pulseIn(pin, HIGH, timeout): returns pulse width in us, 0 on timeout
static uint16_t pulseInHigh(const GpioPin& p, uint16_t timeout_us) {
    const uint16_t limit = timeout_us * 2u;          // in 0.5 us ticks (max 32767 us)
    const uint16_t t0    = TCNT1;

    while (pinRead(p))                               // wait for previous pulse to end
        if ((uint16_t)(TCNT1 - t0) >= limit) return 0;
    while (!pinRead(p))                              // wait for rising edge
        if ((uint16_t)(TCNT1 - t0) >= limit) return 0;

    const uint16_t rise = TCNT1;
    while (pinRead(p))                               // wait for falling edge
        if ((uint16_t)(TCNT1 - t0) >= limit) return 0;

    return (uint16_t)(TCNT1 - rise) / 2u;
}

// ═════════════════════════════════════════════════════════════════════════════
// UART0 - 9600 8N1, interrupt-driven TX buffer (behaves like Arduino Serial)
// ═════════════════════════════════════════════════════════════════════════════

#define TX_BUF_SIZE 64                                // must be a power of 2
static volatile uint8_t txBuf[TX_BUF_SIZE];
static volatile uint8_t txHead = 0;
static volatile uint8_t txTail = 0;

static void uart_init() {
    UCSR0A = _BV(U2X0);
    UBRR0  = 207;                                     // 9600 baud @ 16 MHz, U2X (0.2 % error)
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);               // 8 data, no parity, 1 stop
    UCSR0B = _BV(TXEN0);
}

ISR(USART0_UDRE_vect) {
    if (txHead == txTail) {
        UCSR0B &= ~_BV(UDRIE0);                       // buffer empty
        return;
    }
    UDR0   = txBuf[txTail];
    txTail = (txTail + 1) & (TX_BUF_SIZE - 1);
}

static void uart_putc(char c) {
    uint8_t next = (txHead + 1) & (TX_BUF_SIZE - 1);
    while (next == txTail) { }                        // buffer full: wait
    txBuf[txHead] = c;
    txHead = next;
    UCSR0B |= _BV(UDRIE0);
}

static void uart_print_P(const char* s) {
    char c;
    while ((c = pgm_read_byte(s++))) uart_putc(c);
}
#define PRINT(s)   uart_print_P(PSTR(s))
#define PRINTLN(s) uart_print_P(PSTR(s "\r\n"))

static void uart_newline() { uart_putc('\r'); uart_putc('\n'); }

static void uart_printUInt(uint32_t v) {
    char buf[10];
    uint8_t i = 0;
    do { buf[i++] = '0' + (v % 10); v /= 10; } while (v);
    while (i) uart_putc(buf[--i]);
}

// Two decimals, like Serial.print(float)
static void uart_printFloat(float v) {
    if (isnan(v)) { PRINT("nan"); return; }
    if (v < 0)    { uart_putc('-'); v = -v; }
    v += 0.005f;
    uint32_t ip   = (uint32_t)v;
    uint8_t  frac = (uint8_t)((v - (float)ip) * 100.0f);
    uart_printUInt(ip);
    uart_putc('.');
    uart_putc('0' + frac / 10);
    uart_putc('0' + frac % 10);
}

// ═════════════════════════════════════════════════════════════════════════════
// Bit-banged I2C master (open-drain style, internal pull-ups as backup)
// ═════════════════════════════════════════════════════════════════════════════

static inline void i2c_delay() { _delay_us(I2C_HALF_BIT_US); }

static inline void odRelease(const GpioPin& p) { *p.ddr &= ~p.mask; *p.port |= p.mask; }  // line high
static inline void odLow    (const GpioPin& p) { *p.port &= ~p.mask; *p.ddr |= p.mask; }  // line low

static void i2c_init() {
    odRelease(I2C_SDA);
    odRelease(I2C_SCL);
}

static void i2c_start() {
    odRelease(I2C_SDA); odRelease(I2C_SCL); i2c_delay();
    odLow(I2C_SDA);                         i2c_delay();
    odLow(I2C_SCL);                         i2c_delay();
}

static void i2c_stop() {
    odLow(I2C_SDA);     i2c_delay();
    odRelease(I2C_SCL); i2c_delay();
    odRelease(I2C_SDA); i2c_delay();
}

// Returns true if the slave ACKed
static bool i2c_writeByte(uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) {
        if (b & 0x80) odRelease(I2C_SDA); else odLow(I2C_SDA);
        b <<= 1;
        i2c_delay();
        odRelease(I2C_SCL); i2c_delay();
        odLow(I2C_SCL);
    }
    odRelease(I2C_SDA);                     // let the slave drive ACK
    i2c_delay();
    odRelease(I2C_SCL); i2c_delay();
    bool ack = !pinRead(I2C_SDA);
    odLow(I2C_SCL);
    i2c_delay();
    return ack;
}

static void motor_writeReg(uint8_t reg, uint8_t val) {
    i2c_start();
    i2c_writeByte(MOTOR_I2C_ADDR << 1);     // write
    i2c_writeByte(reg);
    i2c_writeByte(val);
    i2c_stop();
}

// ═════════════════════════════════════════════════════════════════════════════
// Mecanum drive (replaces MecanumCar_v2)
// Each wheel uses two PWM registers; channel mapping copied from the library:
//   Upper_L (FL): 3/4   Upper_R (FR): 1/2   Lower_L (BL): 7/8   Lower_R (BR): 5/6
// Forward = first channel 0, second channel PWM.
// ═════════════════════════════════════════════════════════════════════════════

enum Wheel : uint8_t { FL = 0, FR, BL, BR };

struct WheelCh { uint8_t a, b; };
static const WheelCh WHEEL_CH[4] = {
    { 3, 4 },   // FL
    { 1, 2 },   // FR
    { 7, 8 },   // BL
    { 5, 6 },   // BR
};

static uint8_t wheelSpeed[4] = { 80, 80, 80, 80 };   // library defaults

static void setWheel(Wheel w, bool forward, uint8_t pwm) {
    const WheelCh& ch = WHEEL_CH[w];
    if (forward) { motor_writeReg(ch.a, 0);   motor_writeReg(ch.b, pwm); }
    else         { motor_writeReg(ch.a, pwm); motor_writeReg(ch.b, 0);   }
}

static void car_stop() {
    setWheel(FL, false, 0); setWheel(BL, false, 0);
    setWheel(FR, false, 0); setWheel(BR, false, 0);
}

static void car_advance() {
    setWheel(FL, true, wheelSpeed[FL]); setWheel(BL, true, wheelSpeed[BL]);
    setWheel(FR, true, wheelSpeed[FR]); setWheel(BR, true, wheelSpeed[BR]);
}

static void car_back() {
    setWheel(FL, false, wheelSpeed[FL]); setWheel(BL, false, wheelSpeed[BL]);
    setWheel(FR, false, wheelSpeed[FR]); setWheel(BR, false, wheelSpeed[BR]);
}

static void car_rMove() {   // FL fwd, BL back, FR back, BR fwd
    setWheel(FL, true,  wheelSpeed[FL]); setWheel(BL, false, wheelSpeed[BL]);
    setWheel(FR, false, wheelSpeed[FR]); setWheel(BR, true,  wheelSpeed[BR]);
}

static void car_lMove() {   // FL back, BL fwd, FR fwd, BR back
    setWheel(FL, false, wheelSpeed[FL]); setWheel(BL, true,  wheelSpeed[BL]);
    setWheel(FR, true,  wheelSpeed[FR]); setWheel(BR, false, wheelSpeed[BR]);
}

static void car_init() {
    i2c_init();
    delay_ms(1000);                         // library Init() waits 1 s
    car_stop();
}

// ═════════════════════════════════════════════════════════════════════════════
// Application state
// ═════════════════════════════════════════════════════════════════════════════

static float dist_F, dist_L, dist_R;
static float prev_F = 0, prev_L = 0, prev_R = 0;
static uint32_t stuckTimer = 0;

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static uint8_t toPwm(int v) { return (uint8_t)clampi(v, 0, 255); }

// ── Sensors ─────────────────────────────────────────────────────────────────
static void sensors_init() {
    const Ultrasonic* s[] = { &US_FRONT, &US_LEFT, &US_RIGHT };
    for (const Ultrasonic* u : s) {
        pinLow(u->trig); pinOutput(u->trig);
        pinInput(u->echo);
    }
}

static float readSensor(const Ultrasonic& s) {
    pinLow(s.trig);  _delay_us(2);
    pinHigh(s.trig); _delay_us(10);
    pinLow(s.trig);
    float d = pulseInHigh(s.echo, ECHO_TIMEOUT_US) / 58.2f;
    if (d <= 0 || d > 300) d = 300;         // invalid / no echo
    return d;
}

static void readAllSensors() {
    dist_F = readSensor(US_FRONT);
    dist_L = readSensor(US_LEFT);
    dist_R = readSensor(US_RIGHT);
}

// ── Motion primitives ───────────────────────────────────────────────────────
static void applyWheels(int fl, int fr, int bl, int br) {
    wheelSpeed[FL] = toPwm(fl);
    wheelSpeed[FR] = toPwm(fr);
    wheelSpeed[BL] = toPwm(bl);
    wheelSpeed[BR] = toPwm(br);
    car_advance();
}

static void setAllSpeeds(int s) {
    uint8_t v = toPwm(s);
    wheelSpeed[FL] = wheelSpeed[FR] = wheelSpeed[BL] = wheelSpeed[BR] = v;
}

static void driveForward(int s)   { applyWheels(s, s, s, s); }
static void driveBack(int s)      { setAllSpeeds(s); car_back(); }
static void driveSlideRight(int s){ setAllSpeeds(s); car_rMove(); }
static void driveSlideLeft(int s) { setAllSpeeds(s); car_lMove(); }
static void stopCar()             { car_stop(); }

static void smoothTurnRight(int base) {
    int fast = clampi(base + TURN_BOOST, 0, 255);   // left wheels faster
    int slow = clampi(base - TURN_BOOST, 0, 255);   // right wheels slower
    applyWheels(fast, slow, fast, slow);
    PRINTLN(">> Smooth turn RIGHT");
}

static void smoothTurnLeft(int base) {
    int fast = clampi(base + TURN_BOOST, 0, 255);   // right wheels faster
    int slow = clampi(base - TURN_BOOST, 0, 255);   // left wheels slower
    applyWheels(slow, fast, slow, fast);
    PRINTLN(">> Smooth turn LEFT");
}

// ── Back escape ─────────────────────────────────────────────────────────────
static void escapeBack() {
    PRINTLN(">> Escape: reversing");

    uint32_t t = millis();
    while (millis() - t < BACK_DURATION_MS) {
        driveBack(SPEED_BACK);
        delay_ms(1);
    }
    stopCar();
    delay_ms(150);

    readAllSensors();
    PRINT(">> After reverse - L:"); uart_printFloat(dist_L);
    PRINT(" R:");                    uart_printFloat(dist_R); uart_newline();

    t = millis();
    while (millis() - t < TURN_DURATION_MS) {
        if (dist_R >= dist_L) smoothTurnRight(SPEED_MANEUVER);
        else                  smoothTurnLeft(SPEED_MANEUVER);
        delay_ms(1);
    }
    stopCar();
    delay_ms(150);

    stuckTimer = 0;
    prev_F = prev_L = prev_R = 0;
}

// ── Stuck detection ─────────────────────────────────────────────────────────
static bool isStuck() {
    bool noChange = (fabsf(dist_F - prev_F) < STUCK_DELTA &&
                     fabsf(dist_L - prev_L) < STUCK_DELTA &&
                     fabsf(dist_R - prev_R) < STUCK_DELTA);
    prev_F = dist_F;
    prev_L = dist_L;
    prev_R = dist_R;

    if (noChange) {
        if (stuckTimer == 0) stuckTimer = millis();
        if (millis() - stuckTimer >= STUCK_TIMEOUT_MS) return true;
    } else {
        stuckTimer = 0;
    }
    return false;
}

// ── Navigation ──────────────────────────────────────────────────────────────
static void navigate() {
    bool frontValid = (dist_F > 0 && dist_F < 299);
    bool leftValid  = (dist_L > 0 && dist_L < 299);
    bool rightValid = (dist_R > 0 && dist_R < 299);

    bool frontClear = (!frontValid || dist_F > DIST_WARN);
    bool leftClear  = (!leftValid  || dist_L > DIST_AVOID);
    bool rightClear = (!rightValid || dist_R > DIST_AVOID);

    // CASE 1: all clear - cruise
    if (frontClear && leftClear && rightClear) {
        driveForward((dist_F > DIST_CLEAR) ? SPEED_CRUISE : SPEED_APPROACH);
        return;
    }

    // CASE 2: front obstacle in warning range - early steer
    if (frontValid && dist_F <= DIST_WARN && dist_F > DIST_AVOID) {
        if (rightClear && (!leftClear || dist_R >= dist_L)) {
            PRINTLN(">> Early steer RIGHT");
            smoothTurnRight(SPEED_APPROACH);
        } else if (leftClear) {
            PRINTLN(">> Early steer LEFT");
            smoothTurnLeft(SPEED_APPROACH);
        } else {
            driveForward(SPEED_SLOW);
        }
        return;
    }

    // CASE 3: front obstacle close - stronger turn
    if (frontValid && dist_F <= DIST_AVOID) {
        if (rightClear && (!leftClear || dist_R >= dist_L)) {
            PRINTLN(">> Turn RIGHT (close)");
            smoothTurnRight(SPEED_MANEUVER);
        } else if (leftClear) {
            PRINTLN(">> Turn LEFT (close)");
            smoothTurnLeft(SPEED_MANEUVER);
        } else {
            driveForward(SPEED_SLOW);
        }
        return;
    }

    // CASE 4: side obstacle only - slide away
    if (leftValid && dist_L <= DIST_AVOID && rightClear) {
        PRINTLN(">> Left obstacle - slide RIGHT");
        driveSlideRight(SPEED_MANEUVER);
        return;
    }
    if (rightValid && dist_R <= DIST_AVOID && leftClear) {
        PRINTLN(">> Right obstacle - slide LEFT");
        driveSlideLeft(SPEED_MANEUVER);
        return;
    }

    // Both sides tight - straight, slowly
    driveForward(SPEED_SLOW);
}

// ═════════════════════════════════════════════════════════════════════════════
// main  (setup + loop)
// ═════════════════════════════════════════════════════════════════════════════

static void loopOnce() {
    readAllSensors();

    PRINT("F:");  uart_printFloat(dist_F);
    PRINT(" L:"); uart_printFloat(dist_L);
    PRINT(" R:"); uart_printFloat(dist_R); uart_newline();

    // Priority 1: emergency
    if (dist_F <= DIST_EMERGENCY || dist_L <= DIST_EMERGENCY || dist_R <= DIST_EMERGENCY) {
        escapeBack();
        return;
    }

    // Priority 2: stuck recovery
    if (isStuck()) {
        PRINTLN(">> Stuck detected - escaping");
        escapeBack();
        return;
    }

    // Priority 3: navigate
    navigate();

    delay_ms(LOOP_DELAY_MS);
}

int main() {
    timebase_init();
    uart_init();
    sensors_init();
    sei();

    car_init();
    car_stop();
    delay_ms(1000);

    PRINTLN("=== Autonomous Navigation Start ===");

    for (;;) {
        loopOnce();
    }
}
