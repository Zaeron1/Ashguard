// Sensors.h
#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino_BHY2Host.h"

// Classe pour gérer les capteurs
class MySensorManager {
public:
    MySensorManager();
    bool initialize();
    bool areSensorsReady();
    void updateSensors();
    void printSensorData();
    bool checkForZeroValues();
    
private:
    Sensor tempSensor;
    Sensor gasSensor;
    Sensor humSensor;
    Sensor pressureSensor;
    SensorBSEC bsecSensor;
    SensorXYZ accelerometer;
    
    float temperature;
    float gas;
    float humidity;
    float pressure;
    
    float iaq;
    float iaq_s;
    float b_voc_eq;
    float co2_eq;
    int accuracy;
    float comp_t;
    float comp_h;
    float comp_g;
    
    float accX;
    float accY;
    float accZ;
    
    int zeroCount;
    
    bool isValid(float value);
    float extractValue(const String& data, const String& label);
    void extractXYZValues(const String& data, float& x, float& y, float& z);
};

#endif // SENSORS_H