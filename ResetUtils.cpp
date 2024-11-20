// ResetUtils.cpp
#include "ResetUtils.h"
#include <Arduino.h>
#include <mbed.h> // Pour NVIC_SystemReset()

void resetNicla() {
    const int resetPin = 7; // D7 sur la Portenta H7 correspond au pin de réinitialisation de la Nicla Sense ME
    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, LOW);
    delay(10);
    digitalWrite(resetPin, HIGH);
    delay(100);
}

void resetPortenta() {
    NVIC_SystemReset();
}