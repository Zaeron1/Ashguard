#include "Arduino_BHY2Host.h"
#include <mbed.h> // Pour NVIC_SystemReset()

// Déclaration des capteurs
Sensor tempSensor(SENSOR_ID_TEMP);
Sensor gasSensor(SENSOR_ID_GAS);
Sensor humSensor(SENSOR_ID_HUM);
Sensor pressureSensor(SENSOR_ID_BARO);
SensorBSEC bsec(SENSOR_ID_BSEC);
SensorXYZ accelerometer(SENSOR_ID_ACC);

// Fonction pour vérifier si une valeur est valide
bool isValid(float value) {
    return !isnan(value);
}

// Fonction pour extraire la valeur après une étiquette donnée
float extractValue(String data, String label) {
    int index = data.indexOf(label);
    if (index != -1) {
        index += label.length();
        int endIndex = data.indexOf(' ', index);
        if (endIndex == -1) {
            endIndex = data.length();
        }
        String valueStr = data.substring(index, endIndex);
        valueStr.trim(); // Supprime les espaces
        return valueStr.toFloat();
    } else {
        return NAN; // Retourne NaN si l'étiquette n'est pas trouvée
    }
}

// Fonction pour extraire les valeurs X, Y, Z de l'accéléromètre
void extractXYZValues(String data, float &x, float &y, float &z) {
    x = extractValue(data, "X: ");
    y = extractValue(data, "Y: ");
    z = extractValue(data, "Z: ");
}

// Fonction pour réinitialiser la Nicla Sense ME
void resetNicla() {
    const int resetPin = 7; // D7 sur la Portenta H7 correspond au pin de réinitialisation de la Nicla Sense ME

    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, LOW);   // Mettre le pin de réinitialisation à LOW
    delay(10);                     // Attendre 10 ms
    digitalWrite(resetPin, HIGH);  // Relâcher le pin de réinitialisation
    delay(100);                    // Attendre 100 ms pour que la Nicla se réinitialise
}

// Fonction pour réinitialiser la Portenta H7
void resetPortenta() {
    NVIC_SystemReset();
}

void setup() {
    Serial.begin(115200);

    // Attendre que le port série soit prêt (facultatif)
    while (!Serial);

    // Attendre un peu pour permettre à la Nicla Sense ME de s'initialiser
    delay(5000);

    // Initialisation de la communication avec la Nicla Sense ME
    Serial.println("Initialisation de la communication Nicla");

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
            delay(2000); // Attendre avant de réessayer
        }
    }

    if (!initialized) {
        Serial.println("Impossible d'initialiser BHY2Host après plusieurs tentatives.");
        while (1); // Arrêt si l'initialisation échoue
    }

    // Ajouter un délai pour permettre aux capteurs de s'initialiser
    delay(2000);

    // Configuration des capteurs
    tempSensor.configure(10,0); // Taux d'échantillonnage 10 Hz
    gasSensor.configure(10,0);
    humSensor.configure(10,0);
    pressureSensor.configure(10,0);
    bsec.begin();
    accelerometer.configure(10,0);

    // Attendre que les capteurs soient prêts
    Serial.println("Attente que les capteurs soient prêts...");

    bool sensorsReady = false;
    int attempts = 0;
    const int MAX_ATTEMPTS = 20; // Nombre maximum de tentatives
    while (!sensorsReady && attempts < MAX_ATTEMPTS) {
        BHY2Host.update(); // Mise à jour des capteurs

        float temperature = tempSensor.value();
        float gas = gasSensor.value();
        float hum = humSensor.value();
        float pressure = pressureSensor.value();

        // Récupérer la chaîne de données du capteur BSEC
        String bsecData = bsec.toString();

        // Extraire une valeur (par exemple, IAQ) pour vérifier si BSEC fonctionne
        float iaq = extractValue(bsecData, "iaq: ");

        // Vérifier si toutes les valeurs sont valides
        if (isValid(temperature) && isValid(gas) && isValid(hum) && isValid(pressure) && isValid(iaq)) {
            sensorsReady = true;
        } else {
            attempts++;
            delay(1000); // Attendre avant de réessayer
            Serial.println("Les capteurs ne sont pas encore prêts, nouvelle tentative...");
        }
    }

    if (!sensorsReady) {
        Serial.println("Échec de l'initialisation des capteurs après plusieurs tentatives.");
        while (1); // Arrêt du programme
    }

    Serial.println("Capteurs prêts !");
}

void loop() {
    static int zeroCount = 0;

    BHY2Host.update(); // Mise à jour des capteurs

    float temperature = tempSensor.value();
    float gas = gasSensor.value();
    float hum = humSensor.value();
    float pressure = pressureSensor.value();

    // Vérifier si les valeurs sont valides avant de les afficher
    if (isValid(temperature)) {
        Serial.println(String("temp info: ") + String(temperature) + " °C");
    } else {
        Serial.println("Données de température non disponibles.");
    }

    if (isValid(gas)) {
        Serial.println(String("gas info: ") + String(gas) + " Ohms");
    } else {
        Serial.println("Données de gaz non disponibles.");
    }

    if (isValid(hum)) {
        Serial.println(String("hum info: ") + String(hum) + " %");
    } else {
        Serial.println("Données d'humidité non disponibles.");
    }

    if (isValid(pressure)) {
        Serial.println(String("pressure info: ") + String(pressure) + " hPa");
    } else {
        Serial.println("Données de pression non disponibles.");
    }

    // Récupérer la chaîne de données du capteur BSEC
    String bsecData = bsec.toString();

    // Variables pour stocker les valeurs extraites
    float iaq, iaq_s, b_voc_eq, co2_eq, comp_t, comp_h, comp_g;
    int accuracy;

    // Extraire les valeurs de la chaîne
    iaq       = extractValue(bsecData, "iaq: ");
    iaq_s     = extractValue(bsecData, "iaq_s: ");
    b_voc_eq  = extractValue(bsecData, "b_voc_eq: ");
    co2_eq    = extractValue(bsecData, "co2_eq: ");
    accuracy  = (int)extractValue(bsecData, "accuracy: ");
    comp_t    = extractValue(bsecData, "comp_t: ");
    comp_h    = extractValue(bsecData, "comp_h: ");
    comp_g    = extractValue(bsecData, "comp_g: ");

    // Vérifier si les valeurs BSEC sont valides
    if (isValid(iaq)) {
        // Afficher les valeurs BSEC
        Serial.println("=== Données BSEC ===");
        Serial.print("IAQ : "); Serial.println(iaq);
        Serial.print("IAQ Statique : "); Serial.println(iaq_s);
        Serial.print("Équivalent COV : "); Serial.print(b_voc_eq); Serial.println(" ppm");
        Serial.print("Équivalent CO₂ : "); Serial.print(co2_eq); Serial.println(" ppm");
        Serial.print("Précision : "); Serial.println(accuracy);
        Serial.print("Température compensée : "); Serial.print(comp_t); Serial.println(" °C");
        Serial.print("Humidité compensée : "); Serial.print(comp_h); Serial.println(" %");
        Serial.print("Résistance de gaz : "); Serial.print(comp_g); Serial.println(" Ohms");
        Serial.println();
    } else {
        Serial.println("Données BSEC non disponibles.");
    }

    Serial.println(String("BSEC info: ") + bsecData);

    // Récupérer la chaîne de données de l'accéléromètre
    String accData = accelerometer.toString();

    // Variables pour stocker les valeurs X, Y, Z
    float accX, accY, accZ;

    // Extraire les valeurs X, Y, Z
    extractXYZValues(accData, accX, accY, accZ);

    // Afficher les valeurs de l'accéléromètre
    Serial.println("=== Données Accéléromètre ===");
    Serial.print("X : "); Serial.println(accX);
    Serial.print("Y : "); Serial.println(accY);
    Serial.print("Z : "); Serial.println(accZ);
    Serial.println();

    // Vérifier si une des variables est égale à zéro (sauf 'accuracy') pendant 3 cycles consécutifs
    bool zeroDetected = false;

    // Vérifier toutes les variables (sauf 'accuracy')
    if (iaq == 0 || iaq_s == 0 || b_voc_eq == 0 || co2_eq == 0 || comp_t == 0 || comp_h == 0 || comp_g == 0 ||
        temperature == 0 || gas == 0 || hum == 0 || pressure == 0 ||
        accX == 0 || accY == 0 || accZ == 0) {
        zeroDetected = true;
    }

    if (zeroDetected) {
        zeroCount++;
        if (zeroCount >= 15) {
            Serial.println("Une ou plusieurs variables sont égales à zéro pendant 3 cycles consécutifs.");
            Serial.println("Réinitialisation de la Nicla Sense ME...");
            resetNicla(); // Réinitialiser la Nicla via D7

            Serial.println("Réinitialisation de la Portenta H7...");
            resetPortenta(); // Réinitialiser la Portenta H7
        }
    } else {
        zeroCount = 0;
    }

    // Attendre avant la prochaine lecture
    delay(1000);
}