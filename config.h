#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// EEPROM
// ============================================================

#define CONFIG_EEPROM_ADDRESS 0x52

#define CONFIG_MAGIC   0x574C
#define CONFIG_VERSION 2

#define CONFIG_EEPROM_START 0x00

// ============================================================
// CZUJNIKI
// ============================================================

#define SENSOR_AHT20     0x01
#define SENSOR_BMP280    0x02
#define SENSOR_DS18B20   0x04
#define SENSOR_TEMT6000  0x08
#define SENSOR_INA3221   0x10

// ============================================================
// KONFIGURACJA
// ============================================================

struct __attribute__((packed)) WeatherConfig{
    //char nodeId[13];
    uint8_t nodeId;  // 0–255

    uint8_t nrfChannel;
    uint8_t nrfPower;
    uint8_t nrfDataRate;

    uint16_t sendIntervalMs;

    uint8_t sensorFlags;

    int16_t tempOffsetC100;
    int16_t humidityOffsetC100;
    int16_t pressureOffsetPa;

    uint32_t seaLevelPressurePa;
};

extern WeatherConfig config;

// ============================================================
// REKORD EEPROM
// ============================================================

struct __attribute__((packed)) ConfigRecord{
    uint16_t magic;
    uint8_t version;
    uint8_t length;

    WeatherConfig config;

    uint16_t crc;
};

// ============================================================
// API
// ============================================================

void configSetDefaults(WeatherConfig &cfg);

bool configLoad(WeatherConfig &cfg);

bool configSave(const WeatherConfig &cfg);

void configPrint(const WeatherConfig &cfg);

void configProcessCommand(char *command);

void configSend();

#endif