/*
i960SxManagementEngine
Copyright (c) 2020-2022, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/// i960Sx management engine, based on atmega1284p with fuses set for:
/// - 20Mhz crystal
/// - D1 acts as CLKO
/// Language options:
/// - C++17
/// Board Platform: MightyCore
#include <Arduino.h>
constexpr auto EnableConsole = false;
enum class i960Pinout : int {
    PORT_B0 = 0,
    PORT_B1,
    PORT_B2,
    PORT_B3,
    PORT_B4,
    PORT_B5,
    PORT_B6,
    PORT_B7,
    PORT_D0,
    PORT_D1,
    PORT_D2,
    PORT_D3,
    PORT_D4,
    PORT_D5,
    PORT_D6,
    PORT_D7,
    PORT_C0,
    PORT_C1,
    PORT_C2,
    PORT_C3,
    PORT_C4,
    PORT_C5,
    PORT_C6,
    PORT_C7,
    PORT_A0,
    PORT_A1,
    PORT_A2,
    PORT_A3,
    PORT_A4,
    PORT_A5,
    PORT_A6,
    PORT_A7,
    Count,
    LED = PORT_B0,
    CLOCK_OUT= PORT_B1,
    WaitBoot960 = PORT_B3,
    INT960_0 = PORT_B4,
    INT960_1 = PORT_B5,
    INT960_2 = PORT_B6,
    INT960_3 = PORT_B7,
    RX0 = PORT_D0,
    RX1 = PORT_D1,
    MCU_READY = PORT_D2,
    FAIL960 = PORT_D3,
    DoCycle = PORT_D4,
    StartTransaction = PORT_D5,
    BurstNext = PORT_D6,
    EndTransaction = PORT_D7,
    RESET960 = PORT_C0,
    HLDA960 = PORT_C1,
    HOLD960 = PORT_C2,
    DEN = PORT_C3,
    BLAST = PORT_C4,
    LOCK960 = PORT_C5,
    Ready960 = PORT_C6,
    SuccessfulBoot = PORT_C7,
};

[[gnu::always_inline]]
inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWrite(static_cast<int>(ip), value);
}

[[gnu::always_inline]]
inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}
[[gnu::always_inline]]
inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
}
template<i960Pinout pin>
constexpr auto isValidPin960_v = static_cast<int>(pin) < static_cast<int>(i960Pinout::Count);
//static_assert(isValidPin<i960Pinout::CACHE_A0>, "The CACHE_A0 pin should be a valid pin!");
template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] inline volatile unsigned char& getAssociatedOutputPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number
#define Y(id) \
    X(id, 0): \
    X(id, 1): \
    X(id, 2): \
    X(id, 3): \
    X(id, 4): \
    X(id, 5): \
    X(id, 6): \
    X(id, 7)
        Y(A): return PORTA;
        Y(C): return PORTC;
        Y(D): return PORTD;
        Y(B): return PORTB;
#undef Y
#undef X

        default:
            return PORTA;
    }
}

template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] inline volatile unsigned char& getAssociatedDirectionPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number
#define Y(id) \
    X(id, 0): \
    X(id, 1): \
    X(id, 2): \
    X(id, 3): \
    X(id, 4): \
    X(id, 5): \
    X(id, 6): \
    X(id, 7)
        Y(A): return DDRA;
        Y(C): return DDRC;
        Y(D): return DDRD;
        Y(B): return DDRB;
#undef Y
#undef X

        default:
            return DDRA;
    }
}

template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] inline volatile unsigned char& getAssociatedInputPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number
#define Y(id) \
    X(id, 0): \
    X(id, 1): \
    X(id, 2): \
    X(id, 3): \
    X(id, 4): \
    X(id, 5): \
    X(id, 6): \
    X(id, 7)
        Y(A): return PINA;
        Y(C): return PINC;
        Y(D): return PIND;
        Y(B): return PINB;
#undef Y
#undef X
        default:
            return PINA;
    }
}
template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] constexpr decltype(auto) getPinMask() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number : return _BV ( P ## id ## number )
#define Y(id) \
    X(id, 0); \
    X(id, 1); \
    X(id, 2); \
    X(id, 3); \
    X(id, 4); \
    X(id, 5); \
    X(id, 6); \
    X(id, 7)
        Y(A);
        Y(C);
        Y(D);
        Y(B);
#undef Y
#undef X
        default:
            return 0xFF;
    }
}

template<i960Pinout pin, decltype(HIGH) value>
[[gnu::always_inline]]
inline void digitalWrite() noexcept {
    if constexpr (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
}
template<i960Pinout pin>
[[gnu::always_inline]]
inline void digitalWrite(decltype(HIGH) value) noexcept {
    // don't disable interrupts, that should be done outside this method
    if (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
}
template<i960Pinout pin>
[[gnu::always_inline]] inline void digitalWrite(bool level) noexcept {
    digitalWrite<pin>(level ? HIGH : LOW);
}

template<i960Pinout pin, decltype(HIGH) switchTo = LOW>
[[gnu::always_inline]]
inline void pulse() noexcept {
    // use the switch to value to compute what to revert to
    digitalWrite<pin, switchTo>();
    digitalWrite<pin, ((switchTo == LOW) ? HIGH : LOW)>();
}

template<i960Pinout pin>
[[gnu::always_inline]]
inline auto digitalRead() noexcept {
    return (getAssociatedInputPort<pin>() & getPinMask<pin>()) ? HIGH : LOW;
}

template<i960Pinout pin>
struct DigitalPin {
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr bool isInputPin() noexcept { return false; }
    static constexpr bool isOutputPin() noexcept { return false; }
    static constexpr auto isInputPullupPin() noexcept { return false; }
    static constexpr bool getDirection() noexcept { return false; }
    static constexpr auto getPin() noexcept { return pin; }
    static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }
};

#define DefOutputPin(pin, asserted, deasserted) \
    template<> \
    struct DigitalPin< pin > { \
    static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
        DigitalPin() = delete; \
        ~DigitalPin() = delete; \
        DigitalPin(const DigitalPin&) = delete; \
        DigitalPin(DigitalPin&&) = delete; \
        DigitalPin& operator=(const DigitalPin&) = delete; \
        DigitalPin& operator=(DigitalPin&&) = delete; \
        static constexpr auto isInputPin() noexcept { return false; } \
        static constexpr auto isOutputPin() noexcept { return true; } \
        static constexpr auto isInputPullupPin() noexcept { return false; } \
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return OUTPUT; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; }      \
        [[gnu::always_inline]] inline static void assertPin() noexcept { digitalWrite<pin,getAssertionState()>(); } \
        [[gnu::always_inline]] inline static void deassertPin() noexcept { digitalWrite<pin,getDeassertionState()>(); } \
        [[gnu::always_inline]] inline static void write(decltype(LOW) value) noexcept { digitalWrite<pin>(value); } \
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }          \
        template<decltype(LOW) switchTo = LOW>  \
        [[gnu::always_inline]]                  \
        inline static void pulse() noexcept {   \
            ::pulse<pin, switchTo>();           \
        }                                       \
        static void configure() noexcept { pinMode(pin, OUTPUT); } \
    }
#define DefInputPin(pin, asserted, deasserted) \
    template<> \
    struct DigitalPin< pin > { \
        static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
        DigitalPin() = delete; \
        ~DigitalPin() = delete; \
        DigitalPin(const DigitalPin&) = delete; \
        DigitalPin(DigitalPin&&) = delete; \
        DigitalPin& operator=(const DigitalPin&) = delete; \
        DigitalPin& operator=(DigitalPin&&) = delete; \
        static constexpr auto isInputPin() noexcept { return true; } \
        static constexpr auto isOutputPin() noexcept { return false; } \
        static constexpr auto isInputPullupPin() noexcept { return false; } \
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return INPUT; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; } \
        [[gnu::always_inline]] inline static auto read() noexcept { return digitalRead<pin>(); } \
        [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
        [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; } \
        static void configure() noexcept { pinMode(pin, INPUT); } \
    }

#define DefInputPullupPin(pin, asserted, deasserted) \
    template<> \
    struct DigitalPin< pin > { \
        static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
        DigitalPin() = delete; \
        ~DigitalPin() = delete; \
        DigitalPin(const DigitalPin&) = delete; \
        DigitalPin(DigitalPin&&) = delete; \
        DigitalPin& operator=(const DigitalPin&) = delete; \
        DigitalPin& operator=(DigitalPin&&) = delete; \
        static constexpr auto isInputPin() noexcept { return false; } \
        static constexpr auto isOutputPin() noexcept { return false; }   \
        static constexpr auto isInputPullupPin() noexcept { return true; } \
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return INPUT_PULLUP; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; } \
        [[gnu::always_inline]] inline static auto read() noexcept { return digitalRead<pin>(); } \
        [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
        [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; } \
        static void configure() noexcept { pinMode(pin, INPUT_PULLUP); } \
    }
DefOutputPin(i960Pinout::LED, HIGH, LOW);
DefInputPin(i960Pinout::MCU_READY, LOW, HIGH);
DefInputPin(i960Pinout::FAIL960, HIGH, LOW);
DefOutputPin(i960Pinout::SuccessfulBoot, HIGH, LOW);
DefInputPullupPin(i960Pinout::WaitBoot960, LOW, HIGH);
DefOutputPin(i960Pinout::DoCycle, LOW, HIGH);
DefOutputPin(i960Pinout::StartTransaction, LOW, HIGH);
DefOutputPin(i960Pinout::BurstNext, LOW, HIGH);
DefOutputPin(i960Pinout::EndTransaction, LOW, HIGH);
DefOutputPin(i960Pinout::Ready960, LOW, HIGH);
DefInputPin(i960Pinout::BLAST, LOW, HIGH);
DefInputPin(i960Pinout::DEN, LOW, HIGH);

DefOutputPin(i960Pinout::INT960_0, LOW, HIGH);
DefOutputPin(i960Pinout::INT960_1, HIGH, LOW);
DefOutputPin(i960Pinout::INT960_2, HIGH, LOW);
DefOutputPin(i960Pinout::INT960_3, LOW, HIGH);

DefOutputPin(i960Pinout::RESET960, LOW, HIGH);
DefOutputPin(i960Pinout::LOCK960, LOW, HIGH);
DefInputPin(i960Pinout::HLDA960, LOW, HIGH);
DefOutputPin(i960Pinout::HOLD960, HIGH, LOW);

#undef DefInputPin
#undef DefOutputPin
#undef DefInputPullupPin

template<i960Pinout ... pins>
[[gnu::always_inline]]
inline void configurePins() noexcept {
    (DigitalPin<pins>::configure(), ...);
}

template<i960Pinout ... pins>
[[gnu::always_inline]]
inline void deassertPins() noexcept {
    (DigitalPin<pins>::deassertPin(), ...);
}

template<i960Pinout pinId>
class PinAsserter {
public:
    static_assert(DigitalPin<pinId>::isOutputPin());
    PinAsserter() { DigitalPin<pinId>::assertPin(); }
    ~PinAsserter() { DigitalPin<pinId>::deassertPin(); }
};

[[noreturn]]
void checksumFailureHappened() {
    if constexpr (EnableConsole) {
        Serial.println(F("CHECKSUM FAILURE!"));
    }
    DigitalPin<i960Pinout::SuccessfulBoot>::deassertPin();
    DigitalPin<i960Pinout::LED>::assertPin();
    while(true) {
        delay(1000);
    }
}

volatile bool readyTriggered = false;
void
handleREADYTrigger() noexcept {
    readyTriggered = true;
}

// the setup routine runs once when you press reset:
void setup() {
    DigitalPin<i960Pinout::RESET960>::configure();
    {
        PinAsserter<i960Pinout::RESET960> holdInReset;
        DigitalPin<i960Pinout::WaitBoot960>::configure();
        delay(2000);
        if constexpr (EnableConsole) {
            Serial.begin(9600); // just turn on the serial console
            Serial.println(F("i960Sx Management Engine"));
        }
        configurePins<i960Pinout::DEN,
                i960Pinout::FAIL960,
                i960Pinout::HLDA960,
                i960Pinout::BLAST,
                i960Pinout::MCU_READY,
                i960Pinout::Ready960,
                i960Pinout::LOCK960,
                i960Pinout::HOLD960,
                i960Pinout::INT960_0,
                i960Pinout::INT960_1,
                i960Pinout::INT960_2,
                i960Pinout::INT960_3,
                i960Pinout::SuccessfulBoot,
                i960Pinout::DoCycle,
                i960Pinout::EndTransaction,
                i960Pinout::StartTransaction,
                i960Pinout::BurstNext,
                i960Pinout::LED>();
        deassertPins<i960Pinout::BurstNext,
                i960Pinout::DoCycle,
                i960Pinout::StartTransaction,
                i960Pinout::EndTransaction,
                i960Pinout::LOCK960,
                i960Pinout::HOLD960,
                i960Pinout::INT960_0,
                i960Pinout::INT960_1,
                i960Pinout::INT960_2,
                i960Pinout::INT960_3,
                i960Pinout::SuccessfulBoot,
                i960Pinout::Ready960,
                i960Pinout::LED>();
        attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::MCU_READY)), handleREADYTrigger, FALLING);
        // wait for the chipset to release control
        while (DigitalPin<i960Pinout::WaitBoot960>::isAsserted());
    }
    while (DigitalPin<i960Pinout::FAIL960>::read() == LOW) {
        if (DigitalPin<i960Pinout::DEN>::isAsserted()) {
            break;
        }
    }
    while (DigitalPin<i960Pinout::FAIL960>::read() == HIGH) {
        if (DigitalPin<i960Pinout::DEN>::isAsserted()) {
            break;
        }
    }
    DigitalPin<i960Pinout::SuccessfulBoot>::assertPin();
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::FAIL960)), checksumFailureHappened, RISING);
    if constexpr (EnableConsole) {
        Serial.println(F("Successful boot!"));
    }
}
// ----------------------------------------------------------------
// state machine
// ----------------------------------------------------------------
// The bootup process has a separate set of states
// TStart - Where we start
// TSystemTest - Processor performs self test
//
// TStart -> TSystemTest via FAIL being asserted
// TSystemTest -> Ti via FAIL being deasserted
//
// State machine will stay here for the duration
// State diagram based off of i960SA/SB Reference manual
// Basic Bus States
// Ti - Idle State
// Ta - Address State
// Td - Data State
// Tw - Wait State

// READY - ~READY asserted
// NOT READY - ~READY not asserted
// BURST - ~BLAST not asserted
// NO BURST - ~BLAST asserted
// NEW REQUEST - ~AS asserted
// NO REQUEST - ~AS not asserted when in

// Ti -> Ti via no request
// Ti -> Ta via new request
// on enter of Ta, set address state to false
// on enter of Td, burst is sampled
// Ta -> Td
// Td -> Ti after signaling ready and no burst (blast low)
// Td -> Td after signaling ready and burst (blast high)
// Ti -> TChecksumFailure if FAIL is asserted
// NOTE: Tw may turn out to be synthetic
[[nodiscard]] bool informCPU() noexcept {
    // you must scan the BLAST_ pin before pulsing ready, the cpu will change blast for the next transaction
    auto isBurstLast = DigitalPin<i960Pinout::BLAST>::isAsserted();
    DigitalPin<i960Pinout::Ready960>::pulse();
    return isBurstLast;
}

void loop() {
    // wait for DEN to go low
    while (DigitalPin<i960Pinout::DEN>::isDeasserted());
    DigitalPin<i960Pinout::StartTransaction>::pulse();  // tell the chipset to start the transaction

    do {
       DigitalPin<i960Pinout::DoCycle>::pulse(); // tell the chipset that a new transaction cycle is starting
       while (!readyTriggered);
       readyTriggered = false;
       if (informCPU()) {
           DigitalPin<i960Pinout::EndTransaction>::pulse(); // tell the chipset we are done with the transaction
           break;
       } else {
           // if we got here then it is a burst transaction. Let the chipset know to continue the current transaction
           DigitalPin<i960Pinout::BurstNext>::pulse();
       }
    } while (true);
    // okay now just loop back around and wait for the next data cycle
}

