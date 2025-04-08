// BsecSensor.h
#ifndef BSECSENSOR_H
#define BSECSENSOR_H

#include <Wire.h>
#include "bsec.h"

class BsecSensor {
public:
    BsecSensor();
    void begin();
    void update();
    void printData();
    void checkStatus();

    float getTemperature() const { return temperature; }
    int getHumidity() const { return humidity; }
    float getPressure() const { return pressure; }
    int getIAQ() const { return iaq; }
    int getCarbon() const { return co2Equivalent; }
    float getVOC() const { return breathVocEquivalent; }
    String getIAQStatus() const { return iaqStatus; }

private:
    Bsec iaqSensor;
    String output;
    float temperature;
    int humidity;
    float pressure;
    int iaq;
    int co2Equivalent;
    float breathVocEquivalent;
    String iaqStatus;

    void updateIAQStatus();
};

#endif // BSECSENSOR_H
