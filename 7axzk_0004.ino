#include <Wire.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <RF24.h>
#include <avr/wdt.h>

#include "config.h"

// ============================================================
// PINY
// ============================================================

#define DS18B20_PIN 2
#define TEMT6000_PIN A0

#define NRF_CE  9
#define NRF_CSN 10

// ============================================================
// I2C
// ============================================================

#define EEPROM_ADDRESS 0x52
#define AHT20_ADDRESS  0x38
#define INA3221_ADDRESS 0x41
#define BMP280_ADDRESS 0x77

// ============================================================
// INA3221
// ============================================================

#define INA3221_REG_CONFIG      0x00
#define INA3221_REG_SHUNT1      0x01
#define INA3221_REG_BUS1        0x02
#define INA3221_REG_SHUNT2      0x03
#define INA3221_REG_BUS2        0x04
#define INA3221_REG_SHUNT3      0x05
#define INA3221_REG_BUS3        0x06

#define SHUNT_RESISTOR 0.1f

// ============================================================
// STATUS
// ============================================================

#define STATUS_AHT20    0x01
#define STATUS_BMP280   0x02
#define STATUS_DS18B20  0x04
#define STATUS_TEMT6000 0x08
#define STATUS_INA3221  0x10

// ============================================================
// NRF
// ============================================================

#define WEATHER_HEADER  0xA5
#define WEATHER_VERSION 1

RF24 radio(NRF_CE, NRF_CSN);

const uint8_t RADIO_ADDRESS[6] = "WTHLS";

// ============================================================
// CZUJNIKI
// ============================================================

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

// ============================================================
// KONFIGURACJA
// ============================================================

//WeatherConfig config;

// ============================================================
// STATUS CZUJNIKÓW
// ============================================================

bool ahtOK = false;
bool bmpOK = false;
bool dsOK = false;
bool inaOK = false;
bool nrfOK = false;

// ============================================================
// SEKWENCJA PAKIETU
// ============================================================

uint8_t packetSequence = 0;

// ============================================================
// PAKIET NRF24
//
// MUSI MIEĆ DOKŁADNIE 32 BAJTY
// ============================================================

struct __attribute__((packed)) WeatherPacket{
    uint8_t header;
    uint8_t version;
    uint8_t sequence;

    int16_t  ahtTemp;
    uint16_t ahtHumidity;

    int16_t  bmpTemp;
    uint32_t bmpPressure;

    int16_t  dsTemp;

    uint16_t light;

    uint16_t ina1Voltage;
    int16_t  ina1Current;

    uint16_t ina2Voltage;
    int16_t  ina2Current;

    uint16_t ina3Voltage;
    int16_t  ina3Current;

    uint8_t status;

    uint8_t reserved;

    uint8_t crc;
};

static_assert(sizeof(WeatherPacket) == 32, "WeatherPacket must be exactly 32 bytes");

// ============================================================
// DANE POMIAROWE
// ============================================================

WeatherPacket packet;

// ============================================================
// CZAS
// ============================================================

unsigned long lastSend = 0;
unsigned long lastPrint = 0;

// ============================================================
// CRC8
// ============================================================

uint8_t weatherCRC(const uint8_t *data, uint8_t length){
    uint8_t crc = 0;

    for(uint8_t i = 0; i < length; i++){
        crc ^= data[i];

        for(uint8_t j = 0; j < 8; j++){
            if(crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }

    return crc;
}

// ============================================================
// INA3221
// ============================================================

uint16_t inaRead16(uint8_t reg){
    Wire.beginTransmission(INA3221_ADDRESS);
    Wire.write(reg);

    if(Wire.endTransmission(false) != 0)
        return 0;

    Wire.requestFrom((uint8_t)INA3221_ADDRESS, (uint8_t)2);

    if(Wire.available() < 2)
        return 0;

    uint16_t value = ((uint16_t)Wire.read() << 8);
    value |= Wire.read();

    return value;
}

void inaWrite16(uint8_t reg, uint16_t value){
    Wire.beginTransmission(INA3221_ADDRESS);

    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));

    Wire.endTransmission();
}

float inaBusVoltage(uint8_t channel){
    uint8_t reg;

    if(channel == 1)
        reg = INA3221_REG_BUS1;
    else if(channel == 2)
        reg = INA3221_REG_BUS2;
    else
        reg = INA3221_REG_BUS3;

    int16_t raw = (int16_t)inaRead16(reg);

    raw >>= 3;

    return raw * 0.008f;
}

float inaShuntVoltage(uint8_t channel){
    uint8_t reg;

    if(channel == 1)
        reg = INA3221_REG_SHUNT1;
    else if(channel == 2)
        reg = INA3221_REG_SHUNT2;
    else
        reg = INA3221_REG_SHUNT3;

    int16_t raw = (int16_t)inaRead16(reg);

    raw >>= 3;

    return raw * 0.00004f;
}

float inaCurrent(uint8_t channel){
    return inaShuntVoltage(channel) / SHUNT_RESISTOR;
}

// ============================================================
// NRF KONFIGURACJA
// ============================================================

void configureRadio(){
    Serial.println();
    Serial.println(F("[NRF24] konfiguracja"));

    if(!radio.begin()){
        Serial.println(F("[NRF24] BLAD - brak odpowiedzi"));
        nrfOK = false;
        return;
    }

    radio.setChannel(config.nrfChannel);

    switch(config.nrfPower){
        case 0:
            radio.setPALevel(RF24_PA_MIN);
            break;

        case 1:
            radio.setPALevel(RF24_PA_LOW);
            break;

        case 2:
            radio.setPALevel(RF24_PA_HIGH);
            break;

        default:
            radio.setPALevel(RF24_PA_MAX);
            break;
    }

    switch(config.nrfDataRate){
        case 0:
            radio.setDataRate(RF24_250KBPS);
            break;

        case 1:
            radio.setDataRate(RF24_1MBPS);
            break;

        default:
            radio.setDataRate(RF24_2MBPS);
            break;
    }

    radio.setAutoAck(true);
    radio.setRetries(5, 15);
    radio.setPayloadSize(sizeof(WeatherPacket));
    radio.openWritingPipe(RADIO_ADDRESS);
    radio.stopListening();

    nrfOK = true;

    Serial.println(F("[NRF24] OK"));
}

// ============================================================
// AHT20
// ============================================================

void initAHT20(){
    if(!(config.sensorFlags & SENSOR_AHT20)){
        Serial.println(F("[AHT20] WYLACZONY"));
        return;
    }

    ahtOK = aht.begin();

    if(ahtOK) Serial.println(F("[AHT20] OK"));
    else Serial.println(F("[AHT20] BLAD"));
}

// ============================================================
// BMP280
// ============================================================

void initBMP280(){
    if(!(config.sensorFlags & SENSOR_BMP280)){
        Serial.println(F("[BMP280] WYLACZONY"));
        return;
    }

    bmpOK = bmp.begin(BMP280_ADDRESS);

    if(bmpOK){
        bmp.setSampling(
            Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X2,
            Adafruit_BMP280::SAMPLING_X16,
            Adafruit_BMP280::FILTER_X16,
            Adafruit_BMP280::STANDBY_MS_500
        );

        Serial.println(F("[BMP280] OK"));
    }else{
        Serial.println(F("[BMP280] BLAD"));
    }
}

// ============================================================
// DS18B20
// ============================================================

void initDS18B20(){
    if(!(config.sensorFlags & SENSOR_DS18B20)){
        Serial.println(F("[DS18B20] WYLACZONY"));
        return;
    }

    ds18b20.begin();

    if(ds18b20.getDeviceCount() > 0){
        dsOK = true;
        Serial.print(F("[DS18B20] OK - czujnikow: "));
        Serial.println(ds18b20.getDeviceCount());
    }else{
        Serial.println(F("[DS18B20] BRAK CZUJNIKA"));
    }
}

// ============================================================
// INA3221
// ============================================================

void initINA3221(){
    if(!(config.sensorFlags & SENSOR_INA3221)){
        Serial.println(F("[INA3221] WYLACZONY"));
        return;
    }

    Wire.beginTransmission(INA3221_ADDRESS);

    if(Wire.endTransmission() != 0){
        Serial.println(F("[INA3221] BRAK"));
        return;
    }

    inaWrite16(INA3221_REG_CONFIG, 0x7127);

    inaOK = true;

    Serial.println(F("[INA3221] OK"));
}

// ============================================================
// INICJALIZACJA CZUJNIKÓW
// ============================================================

void initSensors(){
    Serial.println();
    Serial.println(F("=============================="));
    Serial.println(F(" INICJALIZACJA CZUJNIKOW"));
    Serial.println(F("=============================="));

    initAHT20();
    initBMP280();
    initDS18B20();
    initINA3221();

    Serial.println();
}

// ============================================================
// ODCZYT AHT20
// ============================================================

void readAHT20(){
    if(!ahtOK)
        return;

    sensors_event_t humidity;
    sensors_event_t temperature;

    aht.getEvent(&humidity, &temperature);

    float temp = temperature.temperature;
    float hum = humidity.relative_humidity;

    temp += config.tempOffsetC100 / 100.0f;
    hum += config.humidityOffsetC100 / 100.0f;

    packet.ahtTemp = (int16_t)(temp * 100.0f);
    packet.ahtHumidity = (uint16_t)(hum * 100.0f);
}

// ============================================================
// ODCZYT BMP280
// ============================================================

void readBMP280(){
    if(!bmpOK)
        return;

    float temp = bmp.readTemperature();
    float pressure = bmp.readPressure();

    temp += config.tempOffsetC100 / 100.0f;

    pressure += config.pressureOffsetPa;

    packet.bmpTemp = (int16_t)(temp * 100.0f);
    packet.bmpPressure = (uint32_t)pressure;
}

// ============================================================
// ODCZYT DS18B20
// ============================================================

void readDS18B20(){
    if(!dsOK)
        return;

    ds18b20.requestTemperatures();

    float temp = ds18b20.getTempCByIndex(0);

    if(temp == DEVICE_DISCONNECTED_C)
        return;

    temp += config.tempOffsetC100 / 100.0f;

    packet.dsTemp = (int16_t)(temp * 100.0f);
}

// ============================================================
// ODCZYT TEMT6000
// ============================================================

void readTEMT6000(){
    if(!(config.sensorFlags & SENSOR_TEMT6000))
        return;

    packet.light = analogRead(TEMT6000_PIN);
}

// ============================================================
// ODCZYT INA3221
// ============================================================

void readINA3221(){
    if(!inaOK)
        return;

    float bus1 = inaBusVoltage(1);
    float bus2 = inaBusVoltage(2);
    float bus3 = inaBusVoltage(3);

    float cur1 = inaCurrent(1);
    float cur2 = inaCurrent(2);
    float cur3 = inaCurrent(3);

    packet.ina1Voltage = (uint16_t)(bus1 * 1000.0f);
    packet.ina1Current = (int16_t)(cur1 * 1000.0f);

    packet.ina2Voltage = (uint16_t)(bus2 * 1000.0f);
    packet.ina2Current = (int16_t)(cur2 * 1000.0f);

    packet.ina3Voltage = (uint16_t)(bus3 * 1000.0f);
    packet.ina3Current = (int16_t)(cur3 * 1000.0f);
}

// ============================================================
// ZBUDUJ PAKIET
// ============================================================

void buildPacket(){
    memset(&packet, 0, sizeof(packet));

    packet.header = WEATHER_HEADER;
    packet.version = WEATHER_VERSION;
    packet.sequence = packetSequence++;

    if(ahtOK){
        readAHT20();
        packet.status |= STATUS_AHT20;
    }

    if(bmpOK){
        readBMP280();
        packet.status |= STATUS_BMP280;
    }

    if(dsOK){
        readDS18B20();
        packet.status |= STATUS_DS18B20;
    }

    if(config.sensorFlags & SENSOR_TEMT6000){
        readTEMT6000();
        packet.status |= STATUS_TEMT6000;
    }

    if(inaOK){
        readINA3221();
        packet.status |= STATUS_INA3221;
    }

    //packet.reserved = 0;

    packet.reserved = config.nodeId;

    packet.crc = weatherCRC((uint8_t *)&packet, sizeof(packet) - 1);
}

// ============================================================
// WYŚLIJ PAKIET
// ============================================================

void sendPacket(){
    if(!nrfOK)
        return;

    bool result = radio.write(&packet, sizeof(packet));

    if(result)
        Serial.println(F("[NRF] TX OK"));
    else
        Serial.println(F("[NRF] TX FAIL"));
}

// ============================================================
// SERIAL - POMIARY
// ============================================================

void printMeasurements(){
    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F(" AKTUALNE DANE"));
    Serial.println(F("================================"));

    if(packet.status & STATUS_AHT20){
        Serial.println(F("[AHT20]"));

        Serial.print(F("Temperatura: "));
        Serial.print(packet.ahtTemp / 100.0f);
        Serial.println(F(" C"));

        Serial.print(F("Wilgotnosc: "));
        Serial.print(packet.ahtHumidity / 100.0f);
        Serial.println(F(" %"));
    }

    if(packet.status & STATUS_BMP280){
        Serial.println(F("[BMP280]"));

        Serial.print(F("Temperatura: "));
        Serial.print(packet.bmpTemp / 100.0f);
        Serial.println(F(" C"));

        Serial.print(F("Cisnienie: "));
        Serial.print(packet.bmpPressure);
        Serial.println(F(" Pa"));
    }

    if(packet.status & STATUS_DS18B20){
        Serial.println(F("[DS18B20]"));

        Serial.print(F("Temperatura: "));
        Serial.print(packet.dsTemp / 100.0f);
        Serial.println(F(" C"));
    }

    if(packet.status & STATUS_TEMT6000){
        Serial.println(F("[TEMT6000]"));

        Serial.print(F("ADC: "));
        Serial.println(packet.light);
    }

    if(packet.status & STATUS_INA3221){
        Serial.println(F("[INA3221]"));

        Serial.print(F("CH1: "));
        Serial.print(packet.ina1Voltage / 1000.0f);
        Serial.print(F(" V / "));
        Serial.print(packet.ina1Current / 1000.0f);
        Serial.println(F(" A"));

        Serial.print(F("CH2: "));
        Serial.print(packet.ina2Voltage / 1000.0f);
        Serial.print(F(" V / "));
        Serial.print(packet.ina2Current / 1000.0f);
        Serial.println(F(" A"));

        Serial.print(F("CH3: "));
        Serial.print(packet.ina3Voltage / 1000.0f);
        Serial.print(F(" V / "));
        Serial.print(packet.ina3Current / 1000.0f);
        Serial.println(F(" A"));
    }

    Serial.println();
}

// ============================================================
// SERIAL KONFIGURATOR
// ============================================================

void processSerial(){
    static char buffer[120];
    static uint8_t index = 0;

    while(Serial.available()){
        char c = Serial.read();

        if(c == '\n' || c == '\r'){
            if(index == 0)
                continue;

            buffer[index] = 0;

            configProcessCommand(buffer);

            index = 0;
        }else{
            if(index < sizeof(buffer) - 1){
                buffer[index++] = c;
            }else{
                index = 0;
            }
        }
    }
}

// ============================================================
// SETUP
// ============================================================

void setup(){
    Serial.begin(115200);

    delay(500);

    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F(" WEATHERLINK STATION"));
    Serial.println(F("================================"));

    Wire.begin();

    // --------------------------------------------------------
    // EEPROM
    // --------------------------------------------------------

    Serial.println();
    Serial.println(F("[CONFIG] Odczyt konfiguracji EEPROM..."));

    if(!configLoad(config)){
        Serial.println(F("[CONFIG] Brak poprawnej konfiguracji"));
        Serial.println(F("[CONFIG] Tworzenie konfiguracji domyslnej"));

        configSetDefaults(config);

        if(configSave(config))
            Serial.println(F("[CONFIG] Domyslna zapisana"));
        else
            Serial.println(F("[CONFIG] BLAD zapisu EEPROM"));
    }else{
        Serial.println(F("[CONFIG] Konfiguracja OK"));
    }

    configPrint(config);

    // --------------------------------------------------------
    // CZUJNIKI
    // --------------------------------------------------------

    initSensors();

    // --------------------------------------------------------
    // NRF
    // --------------------------------------------------------

    configureRadio();

    // --------------------------------------------------------
    // GOTOWE
    // --------------------------------------------------------

    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F(" NADAJNIK GOTOWY"));
    Serial.println(F("================================"));
    Serial.println(F("USB: wpisz CONFIG aby wejsc"));
    Serial.println();
}

// ============================================================
// LOOP
// ============================================================

void loop(){
    processSerial();

    unsigned long now = millis();

    if(now - lastSend >= config.sendIntervalMs){
        lastSend = now;

        buildPacket();

        sendPacket();
    }

    if(now - lastPrint >= 10000UL){
        lastPrint = now;

        printMeasurements();
    }
}