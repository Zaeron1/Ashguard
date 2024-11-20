// Sensors.cpp
#include "Sensors.h"
#include <Arduino.h>

MySensorManager::MySensorManager()
    : tempSensor(SENSOR_ID_TEMP),
      gasSensor(SENSOR_ID_GAS),
      humSensor(SENSOR_ID_HUM),
      pressureSensor(SENSOR_ID_BARO),
      bsecSensor(SENSOR_ID_BSEC),
      accelerometer(SENSOR_ID_ACC),
      zeroCount(0) {}

bool MySensorManager::initialize() {
    // Code d'initialisation
    bool initialized = false;
    int initAttempts = 0;
    const int MAX_INIT_ATTEMPTS = 5;

    while (!initialized && initAttempts < MAX_INIT_ATTEMPTS) {
        if (BHY2Host.begin(false, NICLA_VIA_ESLOV)) {
            Serial.println("BHY2Host initialisé avec succès.");
            initialized = true;
        } else {
            Serial.println("Échec de l'initialisation de BHY2Host. Nouvelle tentative...");
            initAttempts++;
            delay(2000);
        }
    }
    if (!initialized) {
        Serial.println("Impossible d'initialiser BHY2Host après plusieurs tentatives.");
        return false;
    }

    delay(2000); // Permettre aux capteurs de s'initialiser

    // Configuration des capteurs
    tempSensor.configure(10, 0);
    gasSensor.configure(10, 0);
    humSensor.configure(10, 0);
    pressureSensor.configure(10, 0);
    bsecSensor.begin();
    accelerometer.configure(10, 0);

    return true;
}

bool MySensorManager::areSensorsReady() {
    bool sensorsReady = false;
    int attempts = 0;
    const int MAX_ATTEMPTS = 20;

    while (!sensorsReady && attempts < MAX_ATTEMPTS) {
        BHY2Host.update();

        temperature = tempSensor.value();
        gas = gasSensor.value();
        humidity = humSensor.value();
        pressure = pressureSensor.value();

        String bsecData = bsecSensor.toString();
        iaq = extractValue(bsecData, "iaq: ");

        if (isValid(temperature) && isValid(gas) && isValid(humidity) && isValid(pressure) && isValid(iaq)) {
            sensorsReady = true;
        } else {
            attempts++;
            delay(1000);
            Serial.println("Les capteurs ne sont pas encore prêts, nouvelle tentative...");
        }
    }
    return sensorsReady;
}

void MySensorManager::updateSensors() {
    BHY2Host.update();

    temperature = tempSensor.value();
    gas = gasSensor.value();
    humidity = humSensor.value();
    pressure = pressureSensor.value();

    String bsecData = bsecSensor.toString();

    iaq       = extractValue(bsecData, "iaq: ");
    iaq_s     = extractValue(bsecData, "iaq_s: ");
    b_voc_eq  = extractValue(bsecData, "b_voc_eq: ");
    co2_eq    = extractValue(bsecData, "co2_eq: ");
    accuracy  = (int)extractValue(bsecData, "accuracy: ");
    comp_t    = extractValue(bsecData, "comp_t: ");
    comp_h    = extractValue(bsecData, "comp_h: ");
    comp_g    = extractValue(bsecData, "comp_g: ");

    String accData = accelerometer.toString();
    extractXYZValues(accData, accX, accY, accZ);
}

void MySensorManager::printSensorData() {
    if (isValid(temperature)) {
        Serial.println("Température : " + String(temperature) + " °C");
    } else {
        Serial.println("Données de température non disponibles.");
    }

    if (isValid(gas)) {
        Serial.println("Gaz : " + String(gas) + " Ohms");
    } else {
        Serial.println("Données de gaz non disponibles.");
    }

    if (isValid(humidity)) {
        Serial.println("Humidité : " + String(humidity) + " %");
    } else {
        Serial.println("Données d'humidité non disponibles.");
    }

    if (isValid(pressure)) {
        Serial.println("Pression : " + String(pressure) + " hPa");
    } else {
        Serial.println("Données de pression non disponibles.");
    }

    if (isValid(iaq)) {
        Serial.println("=== Données BSEC ===");
        Serial.println("IAQ : " + String(iaq));
        Serial.println("IAQ Statique : " + String(iaq_s));
        Serial.println("Équivalent COV : " + String(b_voc_eq) + " ppm");
        Serial.println("Équivalent CO₂ : " + String(co2_eq) + " ppm");
        Serial.println("Précision : " + String(accuracy));
        Serial.println("Température compensée : " + String(comp_t) + " °C");
        Serial.println("Humidité compensée : " + String(comp_h) + " %");
        Serial.println("Résistance de gaz : " + String(comp_g) + " Ohms");
    } else {
        Serial.println("Données BSEC non disponibles.");
    }

    Serial.println("=== Données Accéléromètre ===");
    Serial.println("X : " + String(accX));
    Serial.println("Y : " + String(accY));
    Serial.println("Z : " + String(accZ));
}

bool MySensorManager::checkForZeroValues() {
    bool zeroDetected = false;
    if (iaq == 0 || iaq_s == 0 || b_voc_eq == 0 || co2_eq == 0 || comp_t == 0 || comp_h == 0 || comp_g == 0 ||
        temperature == 0 || gas == 0 || humidity == 0 || pressure == 0 ||
        accX == 0 || accY == 0 || accZ == 0) {
        zeroDetected = true;
    }

    if (zeroDetected) {
        zeroCount++;
        if (zeroCount >= 15) {
            Serial.println("Des valeurs nulles ont été détectées pendant plusieurs cycles.");
            zeroCount = 0;
            return true;
        }
    } else {
        zeroCount = 0;
    }
    return false;
}

bool MySensorManager::isValid(float value) {
    return !isnan(value);
}

float MySensorManager::extractValue(const String& data, const String& label) {
    int index = data.indexOf(label);
    if (index != -1) {
        index += label.length();
        int endIndex = data.indexOf(' ', index);
        if (endIndex == -1) {
            endIndex = data.length();
        }
        String valueStr = data.substring(index, endIndex);
        valueStr.trim();
        return valueStr.toFloat();
    } else {
        return NAN;
    }
}

void MySensorManager::extractXYZValues(const String& data, float& x, float& y, float& z) {
    x = extractValue(data, "X: ");
    y = extractValue(data, "Y: ");
    z = extractValue(data, "Z: ");
}