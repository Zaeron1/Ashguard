// main.ino
#include <Arduino.h>
#include "Sensors.h"
#include "ResetUtils.h"

MySensorManager mySensorManager;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    delay(5000); // Permettre à la Nicla Sense ME de s'initialiser

    Serial.println("Initialisation de la communication avec la Nicla Sense ME...");
    if (!mySensorManager.initialize()) {
        Serial.println("Échec de l'initialisation. Arrêt du programme.");
        while (1);
    }

    Serial.println("Attente que les capteurs soient prêts...");
    if (!mySensorManager.areSensorsReady()) {
        Serial.println("Échec de l'initialisation des capteurs après plusieurs tentatives.");
        while (1);
    }

    Serial.println("Capteurs prêts !");
}

void loop() {
    mySensorManager.updateSensors();
    mySensorManager.printSensorData();

    if (mySensorManager.checkForZeroValues()) {
        Serial.println("Réinitialisation de la Nicla Sense ME...");
        resetNicla();

        Serial.println("Réinitialisation de la Portenta H7...");
        resetPortenta();
    }

    delay(1000);
}