/*
i960SxChipset
Copyright (c) 2020-2021, Joshua Scoggins
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

#ifndef ARDUINO_PINOUT_H
#define ARDUINO_PINOUT_H
#include <Arduino.h>
#include "MCUPlatform.h"
using Address = uint32_t;
/**
 * @brief Sx Load/Store styles that the processor will request
 */
enum class LoadStoreStyle : uint8_t {
    // based off of BE0,BE1 pins
#if 0
    Full16 = 0b00,
    Upper8 = 0b01,
    Lower8 = 0b10,
    None = 0b11,
#else
// no need to shift
    Full16 = 0b0000'0000,
    Upper8 = 0b0001'0000,
    Lower8 = 0b0010'0000,
    None = 0b0011'0000,
#endif
};
/// @todo fix this pinout for different targets
enum class i960Pinout : int {
    // PORT B
    Ready = TargetBoard::getReadyPin(),
    CLOCK_OUT = TargetBoard::getClockOutPin(),
    PSRAM_EN = TargetBoard::getPsramEnPin(),
    GPIOSelect = TargetBoard::getGpioSelectPin(),
    MOSI = TargetBoard::getMosiPin(),          // reserved
    MISO = TargetBoard::getMisoPin(),          // reserved
    SCK = TargetBoard::getSckPin(),          // reserved
    DEN_ = TargetBoard::getDenPin(),      // AVR Interrupt INT0
    // PD4 is not used by simple management card
    Reset960= TargetBoard::getReset960Pin(),
    Int0_ = TargetBoard::getInt0Pin(),
    // PD7 is not used by simple management card
    SPI_OFFSET0 = TargetBoard::getSpiOffset0Pin(),
    SPI_OFFSET1 = TargetBoard::getSpiOffset1Pin(),
    SPI_OFFSET2 = TargetBoard::getSpiOffset2Pin(),
    SD_EN = TargetBoard::getSdEnablePin(),      // output
    W_R_ = TargetBoard::getWrPin(),
    BA1 = TargetBoard::getBurstAddress1Pin(),
    BA2 = TargetBoard::getBurstAddress2Pin(),
    BA3 = TargetBoard::getBurstAddress3Pin(),
    BE0 = TargetBoard::getByteEnable0Pin(),
    BE1 = TargetBoard::getByteEnable1Pin(),
    BLAST_ = TargetBoard::getBlastPin(),     // input
    FAIL = TargetBoard::getFailPin(),         // input
};
constexpr bool isValidPin(i960Pinout pin) noexcept {
    return isValidPin<UnderlyingPinoutType>(static_cast<UnderlyingPinoutType>(pin));
}
constexpr auto attachedToIOExpander(i960Pinout pinout) noexcept {
    return attachedToIOExpander<UnderlyingPinoutType>(static_cast<UnderlyingPinoutType>(pinout));
}
template<i960Pinout pin>
constexpr bool attachedToIOExpander_v = attachedToIOExpander(pin);
inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWrite(static_cast<int>(ip), value);
}

inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}
inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
}
template<i960Pinout pin>
constexpr auto isValidPin960_v = isValidPin_v<static_cast<UnderlyingPinoutType >(pin)>;
//static_assert(isValidPin<i960Pinout::CACHE_A0>, "The CACHE_A0 pin should be a valid pin!");
template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedOutputPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (static_cast<UnderlyingPinoutType >(pin)) {
#define X(id, number) case UnderlyingPinoutType:: PORT_ ## id ## number
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
[[nodiscard]] inline volatile unsigned char& getAssociatedInputPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (static_cast<UnderlyingPinoutType >(pin)) {
#define X(id, number) case UnderlyingPinoutType:: PORT_ ## id ## number
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
[[nodiscard]] constexpr decltype(auto) getPinMask() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (static_cast<UnderlyingPinoutType >(pin)) {
#define X(id, number) case UnderlyingPinoutType:: PORT_ ## id ## number : return _BV ( P ## id ## number )
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

template<bool disableInterrupts>
class InterruptDisabler final {
public:
    InterruptDisabler() noexcept {
        if constexpr (disableInterrupts) {
            storage_ = SREG;
            cli();
        }
    }
    ~InterruptDisabler() noexcept {
        if constexpr (disableInterrupts) {
            SREG = storage_;
        }
    }
private:
    uint8_t storage_ = 0;
};

template<i960Pinout pin, bool disableInterrupts = true>
inline void pulse() noexcept {
    // save registers and do the pulse
    InterruptDisabler<disableInterrupts> disableTheInterrupts;
    auto& thePort = getAssociatedOutputPort<pin>();
    thePort ^= getPinMask<pin>();
    thePort ^= getPinMask<pin>();
}

template<i960Pinout pin, decltype(HIGH) value, bool disableInterrupts = true>
inline void digitalWrite() noexcept {
    InterruptDisabler<disableInterrupts> disableTheInterrupts;
    if constexpr (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
}
template<i960Pinout pin, bool disableInterrupts = true>
inline void digitalWrite(decltype(HIGH) value) noexcept {
    InterruptDisabler<disableInterrupts> disableTheInterrupts;
    if (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
}

template<i960Pinout pin>
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
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return OUTPUT; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; }      \
        template<bool disableInterrupts = true>                                         \
        inline static void assertPin() noexcept { digitalWrite<pin,getAssertionState(), disableInterrupts>(); } \
        template<bool disableInterrupts = true>                                         \
        inline static void deassertPin() noexcept { digitalWrite<pin,getDeassertionState(), disableInterrupts>(); } \
        template<bool disableInterrupts = true>                                         \
        inline static void write(decltype(LOW) value) noexcept { digitalWrite<pin, disableInterrupts>(value); } \
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }          \
        template<bool disableInterrupts = true> \
        inline static void pulse() noexcept {   \
            ::pulse<pin, disableInterrupts>();  \
        }                                       \
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
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return INPUT; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; } \
        inline static auto read() noexcept { return digitalRead<pin>(); } \
        inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
        inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; } \
    }
#define DefSPICSPin(pin) DefOutputPin(pin, LOW, HIGH)

DefSPICSPin(i960Pinout::GPIOSelect);
DefSPICSPin(i960Pinout::SD_EN);
DefSPICSPin(i960Pinout::PSRAM_EN);
DefOutputPin(i960Pinout::Reset960, LOW, HIGH);
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefInputPin(i960Pinout::BLAST_, LOW, HIGH);
DefInputPin(i960Pinout::W_R_, LOW, HIGH);
#undef DefSPICSPin
#undef DefInputPin
#undef DefOutputPin

template<typename ... Pins>
inline void setupPins(decltype(OUTPUT) direction, Pins ... pins) {
    (pinMode(pins, direction), ...);
}

template<typename ... Pins>
inline void digitalWriteBlock(decltype(HIGH) value, Pins ... pins) {
    (digitalWrite(pins, value), ...);
}

template<i960Pinout pinId>
class PinAsserter {
public:
    static_assert(DigitalPin<pinId>::isOutputPin());
    PinAsserter() { DigitalPin<pinId>::assertPin(); }
    ~PinAsserter() { DigitalPin<pinId>::deassertPin(); }
};


#endif //ARDUINO_PINOUT_H
