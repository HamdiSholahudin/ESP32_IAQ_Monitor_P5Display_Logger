// BsecSensor.cpp
#include "BsecSensor.h"

BsecSensor::BsecSensor() {
    temperature = 0.0;
    humidity = 0.0;
    pressure = 0.0;
    iaq = 0.0;
    co2Equivalent = 0.0;
    breathVocEquivalent = 0.0;
    iaqStatus = "Unknown";
}

void BsecSensor::begin() {
    Wire.begin();
    Serial.println(F("Starting..."));

    iaqSensor.begin(0x77, Wire);
    output = "BSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
    Serial.println(output);
    checkStatus();

    bsec_virtual_sensor_t sensorList[10] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };

    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    checkStatus();
}

void BsecSensor::update() {
    //unsigned long time_trigger = millis();
    if (iaqSensor.run()) { // If new data is available
        //output = String(time_trigger);
        output += ", " + String(iaqSensor.rawTemperature);
        output += ", " + String(iaqSensor.pressure);
        output += ", " + String(iaqSensor.rawHumidity);
        output += ", " + String(iaqSensor.gasResistance);
        output += ", " + String(iaqSensor.iaq);
        output += ", " + String(iaqSensor.iaqAccuracy);
        output += ", " + String(iaqSensor.temperature);
        output += ", " + String(iaqSensor.humidity);
        output += ", " + String(iaqSensor.staticIaq);
        output += ", " + String(iaqSensor.co2Equivalent);
        output += ", " + String(iaqSensor.breathVocEquivalent);
        //Serial.println(output);

        temperature = iaqSensor.temperature;
        humidity = iaqSensor.humidity;
        pressure = iaqSensor.pressure / 100.0;
        iaq = iaqSensor.staticIaq;
        co2Equivalent = iaqSensor.co2Equivalent;
        breathVocEquivalent = iaqSensor.breathVocEquivalent;

        updateIAQStatus();
    } else {
        checkStatus();
    }
}

void BsecSensor::printData() {
    Serial.print("Temperature = "); 
    Serial.print(temperature); 
    Serial.println(" *C");

    Serial.print("Humidity = "); 
    Serial.print(humidity); 
    Serial.println(" %");

    Serial.print("Pressure = "); 
    Serial.print(pressure); 
    Serial.println(" hPa");

    Serial.print("IAQ = "); 
    Serial.print(iaq); 
    Serial.println(" PPM");

    Serial.print("CO2 equiv = "); 
    Serial.print(co2Equivalent); 
    Serial.println(" PPM");

    Serial.print("Breath VOC = "); 
    Serial.print(breathVocEquivalent); 
    Serial.println(" PPM");

    Serial.println("IAQ Status = " + iaqStatus);
    Serial.println();
}

void BsecSensor::checkStatus() {
    if (iaqSensor.bsecStatus != BSEC_OK) {
        if (iaqSensor.bsecStatus < BSEC_OK) {
            output = "BSEC error code : " + String(iaqSensor.bsecStatus);
            Serial.println(output);
            for (;;);
        } else {
            output = "BSEC warning code : " + String(iaqSensor.bsecStatus);
            Serial.println(output);
        }
    }

    if (iaqSensor.bme68xStatus != BME68X_OK) {
        if (iaqSensor.bme68xStatus < BME68X_OK) {
            output = "BME688 error code : " + String(iaqSensor.bme68xStatus);
            Serial.println(output);
            for (;;);
        } else {
            output = "BME688 warning code : " + String(iaqSensor.bme68xStatus);
            Serial.println(output);
        }
    }
}

void BsecSensor::updateIAQStatus() {
    if ((iaq > 0)  && (iaq <= 50)) {
        iaqStatus = "Good";
    } else if ((iaq > 51)  && (iaq <= 100)) {
        iaqStatus = "Average";
    } else if ((iaq > 101)  && (iaq <= 150)) {
        iaqStatus = "Little Bad";
    } else if ((iaq > 151)  && (iaq <= 200)) {
        iaqStatus = "Bad";
    } else if ((iaq > 201)  && (iaq <= 300)) {
        iaqStatus = "Worse";
    } else if ((iaq > 301)  && (iaq <= 500)) {
        iaqStatus = "Very Bad";
    } else if ((iaq > 500)){
        iaqStatus = "Very Very Bad";
    }
}
