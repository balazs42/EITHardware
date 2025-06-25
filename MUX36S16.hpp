#ifndef _MUX36S16_HPP_
#define _MUX36S16_HPP_

#include "Arduino.h"
#include "configurations.hpp"

// --- MUX36S16Pair Class ---
class MUX36S16Pair
{
private:
    int A0, A1, A2, A3; // The A0...A3 Select pins of the Multiplexers
    int enPin;          // Enable pin of the Multiplexers
public:
    MUX36S16Pair(int a0, int a1, int a2, int a3, int en) : A0(a0), A1(a1), A2(a2), A3(a3), enPin(en)
    {
        // Set the pin modes for the select pins and enable pin
        pinMode(MUX_S0, OUTPUT);
        pinMode(MUX_S1, OUTPUT);
        pinMode(MUX_S2, OUTPUT);
        pinMode(MUX_S3, OUTPUT);
        
        pinMode(MUX_ENABLE_PIN, OUTPUT);

        // HIGH enables both MUX to operate
        digitalWrite(MUX_ENABLE_PIN, HIGH); 
    }

    // Sets the specified channel as GND, CH+1 will be Excitation
    void setMuxInjectionPair(uint8_t injectingChannel)
    {
        // Only work with 0 to 16 as inputs
        if (injectingChannel < 0 || injectingChannel > 16)
            return;

        // If injecting channel is 16, we should revert it to 0, because we go from
        // 0 to 15 as indexing the injection channels
        uint8_t s = (injectingChannel == 16) ? 0 : injectingChannel; // s=0 for p=16 (pair 16-1), else s=p

        // Disable the mux, so the switch between channels doesn't cause an issue
        disable();

        // Set the select lines accordingly
        digitalWrite(MUX_S0, (s & 0x01) ? HIGH : LOW);
        digitalWrite(MUX_S1, ((s & 0x02) >> 1) ? HIGH : LOW);
        digitalWrite(MUX_S2, ((s & 0x04) >> 2) ? HIGH : LOW);
        digitalWrite(MUX_S3, ((s & 0x08) >> 3) ? HIGH : LOW);

        // Wait a microsecond so the signal switch sattles, 
        // and this way, we avoid applying VCC-GND short 
        // while shifthing adjecent channels
        delayMicroseconds(1);

        // Re-enable the muxs
        enable();
    }

    // Enables the multiplexers by writing high to EN pin
    void enable() { digitalWrite(MUX_ENABLE_PIN, HIGH); }

    // Disables the multiplxers by writing low to EN pin
    void disable() { digitalWrite(MUX_ENABLE_PIN, LOW); }
};

#endif /*_MUX36S16_HPP_*/