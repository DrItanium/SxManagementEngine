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

/// i960Sx chipset mcu, based on atmega1284p with fuses set for:
/// - 20Mhz crystal
/// - D1 acts as CLKO
/// Language options:
/// - C++17
/// Board Platform: MightyCore
#include "Pinout.h"


[[noreturn]]
void checksumFailureHappened() {
    DigitalPin<i960Pinout::SuccessfulBoot>::deassertPin();
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
        configurePins<i960Pinout::DEN,
                i960Pinout::AS,
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
                i960Pinout::BurstNext>();
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
                i960Pinout::Ready960>();
        // wait for the chipset to release control
        while (DigitalPin<i960Pinout::WaitBoot960>::isAsserted());
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

}

