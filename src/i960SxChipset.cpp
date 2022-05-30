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

/// i960Sx management engine, based on atmega4809 with fuses set for:
/// - C++17
/// Board Platform: MegaCoreX
#include <Arduino.h>

/// @todo implement a better configuration setup
constexpr auto EnableExternalClockSource = true; // true for the expanded processor card
constexpr auto EnableClockOutputPort = false; // false on the expanded processor card
constexpr auto EnableCommunicationChannel = true; // true on the expanded processor card
enum class i960Pinout : int {
    SRC0_TRIGGER_INT1 = PIN_PF0,
    SRC1_TRIGGER_INT1 = PIN_PF1,
    BUS_LOCKED = PIN_PF2,
    INT1 = PIN_PF3,
    LOCK = PIN_PF4,
    LOCK_REQUESTED = PIN_PF5,
    BOOT_SUCCESSFUL = PIN_PE0,
    DO_CYCLE = PIN_PE1,
    BURST_LAST_ME = PIN_PE2,
    IN_TRANSACTION = PIN_PE3,
    SRC0_TRIGGER_INT3 = PIN_PC0,
    SRC1_TRIGGER_INT3 = PIN_PC1,
    READY_IN = PIN_PC2,
    INT3 = PIN_PC3,
    CLK2 = PIN_PA0,
    SRC0_TRIGGER_INT0 = PIN_PA1,
    SRC1_TRIGGER_INT0 = PIN_PA7,
    INT0 = PIN_PA3,
    CLK = PIN_PA2,
    SRC0_TRIGGER_INT2 = PIN_PD0,
    SRC1_TRIGGER_INT2 = PIN_PD1,
    ME_BOOTED = PIN_PD2,
    INT2 = PIN_PD3,
    BLAST = PIN_PD4,
    DEN = PIN_PD5,
    FAIL = PIN_PD6,
    READY960 = PIN_PD7,
};

enum class PinStyle {
    Input,
    Output,
    InputPullup,
    Any,
};

template<PinStyle style>
constexpr decltype(OUTPUT) PinDirection_v = INPUT; // default to input
template<> constexpr auto PinDirection_v<PinStyle::Input> = INPUT;
template<> constexpr auto PinDirection_v<PinStyle::InputPullup> = INPUT_PULLUP;
template<> constexpr auto PinDirection_v<PinStyle::Output> = OUTPUT;

template<i960Pinout pin, PinStyle style, decltype(HIGH) asserted, decltype(HIGH) deasserted>
struct DigitalPin {
    static_assert(asserted != deasserted, "asserted cannot be equal to deasserted");
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr bool isBidirectionalPin() noexcept { return style == PinStyle::Any; }
    static constexpr bool isInputPin() noexcept { return style == PinStyle::Input || isBidirectionalPin(); }
    static constexpr bool isOutputPin() noexcept { return style == PinStyle::Output || isBidirectionalPin(); }
    static constexpr bool isInputPullupPin() noexcept { return style == PinStyle::InputPullup || isBidirectionalPin(); }
    static constexpr auto getDirection() noexcept { return PinDirection_v<style>; }
    static constexpr auto getPin() noexcept { return pin; }
    static void configure(decltype(OUTPUT) pinDirection = getDirection()) noexcept {
        ::pinMode(static_cast<int>(getPin()), pinDirection);
    }
    static auto read() noexcept {
        return digitalReadFast(static_cast<int>(pin));
    }
    static bool inputAsserted() noexcept { return read() == asserted; }
    static bool inputDeasserted() noexcept { return read() == deasserted; }
    static bool inputLow() noexcept { return read() == LOW; }
    static bool inputHigh() noexcept { return read() == HIGH; }
    static void write(decltype(HIGH) value) noexcept {
        digitalWriteFast(static_cast<int>(pin), value);
    }
    static void assertPin() noexcept { write(asserted); }
    static void deassertPin() noexcept { write(deasserted); }
    static void pulse() noexcept {
        assertPin();
        __builtin_avr_nops(2);
        deassertPin();
    }
};

template<i960Pinout pin, decltype(HIGH) asserted, decltype(HIGH) deasserted>
using OutputPin = DigitalPin<pin, PinStyle::Output, asserted, deasserted>;

template<i960Pinout pin, decltype(HIGH) asserted, decltype(HIGH) deasserted>
using InputPin = DigitalPin<pin, PinStyle::Input, asserted, deasserted>;
template<i960Pinout pin, decltype(HIGH) asserted, decltype(HIGH) deasserted>
using InputPullupPin = DigitalPin<pin, PinStyle::InputPullup, asserted, deasserted>;

template<i960Pinout pin, decltype(HIGH) asserted, decltype(HIGH) deasserted>
using BidirectionalPin = DigitalPin<pin, PinStyle::Any, asserted, deasserted>;

[[gnu::always_inline]]
inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWriteFast(static_cast<int>(ip), value);
}

[[gnu::always_inline]]
inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}
[[gnu::always_inline]]
inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
}
using DenPin = InputPin<i960Pinout::DEN, LOW, HIGH>;
using BlastPin = InputPin<i960Pinout::BLAST, LOW, HIGH>;
using FailPin = InputPin<i960Pinout::FAIL, HIGH, LOW>;
using Src0Trigger3Pin = InputPin<i960Pinout::SRC0_TRIGGER_INT3, LOW, HIGH>;
using Src1Trigger3Pin = InputPin<i960Pinout::SRC1_TRIGGER_INT3, LOW, HIGH>;
using Src0Trigger2Pin = InputPin<i960Pinout::SRC0_TRIGGER_INT2, LOW, HIGH>;
using Src1Trigger2Pin = InputPin<i960Pinout::SRC1_TRIGGER_INT2, LOW, HIGH>;
using Src0Trigger1Pin = InputPin<i960Pinout::SRC0_TRIGGER_INT1, LOW, HIGH>;
using Src1Trigger1Pin = InputPin<i960Pinout::SRC1_TRIGGER_INT1, LOW, HIGH>;
using Src0Trigger0Pin = InputPin<i960Pinout::SRC0_TRIGGER_INT0, LOW, HIGH>;
using Src1Trigger0Pin = InputPin<i960Pinout::SRC1_TRIGGER_INT0, LOW, HIGH>;
using LockRequestedPin = InputPin<i960Pinout::LOCK_REQUESTED, LOW, HIGH>;
using ReadyInputPin = InputPin<i960Pinout::READY_IN, LOW, HIGH>;

using BootSuccessfulPin = OutputPin<i960Pinout::BOOT_SUCCESSFUL, LOW, HIGH>;
using DoCyclePin = OutputPin<i960Pinout::DO_CYCLE, LOW, HIGH>;
using BurstLastME = OutputPin<i960Pinout::BURST_LAST_ME, LOW, HIGH>;
using InTransactionPin = OutputPin<i960Pinout::IN_TRANSACTION, LOW, HIGH>;
using Int0Pin = OutputPin<i960Pinout::INT0, HIGH, LOW>;
using Int1Pin = OutputPin<i960Pinout::INT1, HIGH, LOW>;
using Int2Pin = OutputPin<i960Pinout::INT2, HIGH, LOW>;
using Int3Pin = OutputPin<i960Pinout::INT3, HIGH, LOW>;
using BootedPin = OutputPin<i960Pinout::ME_BOOTED, HIGH, LOW>;
using ReadySyncPin = OutputPin<i960Pinout::READY960, LOW, HIGH>;
using BusLockedPin = OutputPin<i960Pinout::BUS_LOCKED, LOW, HIGH>;

using LockPin = BidirectionalPin<i960Pinout::LOCK, LOW, HIGH>;

template<typename ... pins>
[[gnu::always_inline]]
inline void configurePins() noexcept {
    (pins::configure(), ...);
}
void
setupPins() noexcept {
    configurePins<FailPin, BlastPin , DenPin ,
            LockRequestedPin , BusLockedPin ,
            Int0Pin , Int1Pin , Int2Pin, Int3Pin,
            InTransactionPin , DoCyclePin, BootSuccessfulPin , BurstLastME ,
            Src0Trigger0Pin , Src1Trigger0Pin ,
            Src0Trigger1Pin , Src1Trigger1Pin ,
            Src0Trigger2Pin , Src1Trigger2Pin ,
            Src0Trigger3Pin , Src1Trigger3Pin ,
            ReadySyncPin , ReadyInputPin >();
    // the lock pin is special as it is an open collector pin, we want to stay off of it as much as possible
    LockPin::configure(INPUT);
    // make all outputs deasserted
    Int0Pin::deassertPin();
    Int1Pin::deassertPin();
    Int2Pin::deassertPin();
    Int3Pin::deassertPin();
    BootedPin::deassertPin();
    ReadySyncPin :: deassertPin();
    BootSuccessfulPin :: deassertPin();
    InTransactionPin :: deassertPin();
    DoCyclePin :: deassertPin();
    BurstLastME :: deassertPin();
    BusLockedPin :: deassertPin();
}
template<i960Pinout pin, decltype(HIGH) value>
[[gnu::always_inline]]
inline void digitalWrite() noexcept {
    digitalWrite(pin, value);
}
template<i960Pinout pin>
[[gnu::always_inline]]
inline void digitalWrite(decltype(HIGH) value) noexcept {
    digitalWriteFast(static_cast<int>(pin), value);
}
template<i960Pinout pin>
[[gnu::always_inline]] inline void digitalWrite(bool level) noexcept {
    digitalWriteFast(static_cast<int>(pin), level ? HIGH : LOW);
}

template<i960Pinout pin, decltype(HIGH) asserted = LOW>
[[gnu::always_inline]]
inline void pulse() noexcept {
    static constexpr auto deasserted = asserted == LOW ? HIGH : LOW;
    // use the switch to value to compute what to revert to
    digitalWrite<pin, asserted>();
    __builtin_avr_nops(2);
    digitalWrite<pin, deasserted>();
}

template<i960Pinout pin>
[[gnu::always_inline]]
inline auto digitalRead() noexcept {
    return digitalReadFast(static_cast<int>(pin));
}



template<typename ... pins>
[[gnu::always_inline]]
inline void deassertPins() noexcept {
    (pins::deassertPin(), ...);
}

template<typename T>
class PinAsserter {
public:
    static_assert(T::isOutputPin());
    PinAsserter() { T::assertPin(); }
    ~PinAsserter() { T::deassertPin(); }
};

[[noreturn]]
void
handleChecksumFail() noexcept {
    BootSuccessfulPin :: deassertPin();
    while(true) {
        delay(1000);
    }
}

void
configureClockSource() noexcept {
    byte clkBits = 0;
    if constexpr (EnableExternalClockSource) {
        clkBits |= 0b0000'0011;
    }
    if constexpr (EnableClockOutputPort) {
        clkBits |= 0b1000'0000;
    }
    if constexpr (EnableExternalClockSource || EnableClockOutputPort) {
        CCP = 0xD8;
        CLKCTRL.MCLKCTRLA = clkBits;
        CCP = 0xD8;
    }
    if constexpr (EnableClockOutputPort) {
        CCP = 0xD8;
        CLKCTRL.MCLKCTRLA |= 0b0000'0010;
        CCP = 0xD8;
    }
}

// the setup routine runs once when you press reset:
void setup() {
    configureClockSource();
    // the booted pin is the reset pin conceptually
    BootedPin ::configure();
    BootedPin ::assertPin();
    setupPins();
    // no need to wait for the chipset to release control
    BootedPin::deassertPin();
    while (FailPin::inputLow()) {
        if (DenPin::inputAsserted()) {
            break;
        }
    }
    while (FailPin::inputHigh()) {
        if (DenPin::inputAsserted()) {
            break;
        }
    }
    BootSuccessfulPin::assertPin();
    // at this point, if we ever go from low to high again then we have a checksum failure
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::FAIL)), handleChecksumFail, RISING);
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

