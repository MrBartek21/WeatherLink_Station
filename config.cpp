#include "config.h"

WeatherConfig config;

#include <avr/wdt.h>

// ============================================================
// CRC16 CCITT
// ============================================================

uint16_t configCRC(const uint8_t *data, uint16_t length){
    uint16_t crc = 0xFFFF;

    while(length--){
        crc ^= (uint16_t)*data++ << 8;

        for(uint8_t i = 0; i < 8; i++){
            if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }

    return crc;
}

// ============================================================
// EEPROM WRITE
//
// 24C02 ma strony 8 bajtów.
// Nie przekraczamy granicy strony.
// ============================================================

bool eepromWrite(uint8_t address, const uint8_t *data, uint8_t length){
    while(length > 0){
        uint8_t pageOffset = address & 0x07;
        uint8_t space = 8 - pageOffset;

        uint8_t chunk = length;

        if(chunk > space) chunk = space;

        Wire.beginTransmission(CONFIG_EEPROM_ADDRESS);
        Wire.write(address);

        for(uint8_t i = 0; i < chunk; i++)
            Wire.write(data[i]);

        if(Wire.endTransmission() != 0)
            return false;

        delay(6);

        address += chunk;
        data += chunk;
        length -= chunk;
    }

    return true;
}

// ============================================================
// EEPROM READ
// ============================================================

bool eepromRead(uint8_t address, uint8_t *data, uint8_t length){
    uint8_t received = 0;

    while(length > 0){
        uint8_t chunk = length;

        if(chunk > 16)
            chunk = 16;

        Wire.beginTransmission(CONFIG_EEPROM_ADDRESS);
        Wire.write(address);

        if(Wire.endTransmission(false) != 0)
            return false;

        uint8_t count = Wire.requestFrom((uint8_t)CONFIG_EEPROM_ADDRESS, chunk);

        if(count != chunk)
            return false;

        for(uint8_t i = 0; i < chunk; i++){
            if(!Wire.available())
                return false;

            data[received++] = Wire.read();
        }

        address += chunk;
        length -= chunk;
    }

    return true;
}

// ============================================================
// DEFAULT
// ============================================================

void configSetDefaults(WeatherConfig &cfg){
    memset(&cfg, 0, sizeof(cfg));

    //strcpy(cfg.nodeId, "WEATHER01");
    cfg.nodeId = 1;

    cfg.nrfChannel = 76;

    // 0 MIN
    // 1 LOW
    // 2 HIGH
    // 3 MAX
    cfg.nrfPower = 2;

    // 0 = 250 kbps
    // 1 = 1 Mbps
    // 2 = 2 Mbps
    cfg.nrfDataRate = 0;

    cfg.sendIntervalMs = 2000;

    cfg.sensorFlags =
        SENSOR_AHT20 |
        SENSOR_BMP280 |
        SENSOR_DS18B20 |
        SENSOR_TEMT6000 |
        SENSOR_INA3221;

    cfg.tempOffsetC100 = 0;

    cfg.humidityOffsetC100 = 0;

    cfg.pressureOffsetPa = 0;

    cfg.seaLevelPressurePa = 101325;
}

// ============================================================
// LOAD
// ============================================================

bool configLoad(WeatherConfig &cfg){
    ConfigRecord record;

    if(!eepromRead(CONFIG_EEPROM_START, (uint8_t *)&record, sizeof(record)))
        return false;
    

    if(record.magic != CONFIG_MAGIC)
        return false;

    if(record.version != CONFIG_VERSION)
        return false;

    if(record.length != sizeof(WeatherConfig))
        return false;

    uint16_t crc = configCRC(((uint8_t *)&record) + 2, sizeof(record) - 4);

    if(crc != record.crc)
        return false;

    memcpy(&cfg, &record.config, sizeof(WeatherConfig));

    //cfg.nodeId[12] = 0;

    return true;
}

// ============================================================
// SAVE
// ============================================================

bool configSave(const WeatherConfig &cfg){
    ConfigRecord record;

    memset(&record, 0, sizeof(record));

    record.magic = CONFIG_MAGIC;
    record.version = CONFIG_VERSION;
    record.length = sizeof(WeatherConfig);

    memcpy(&record.config, &cfg, sizeof(WeatherConfig));

    record.crc = configCRC(((uint8_t *)&record) + 2, sizeof(record) - 4);

    return eepromWrite(CONFIG_EEPROM_START, (const uint8_t *)&record, sizeof(record));
}

// ============================================================
// PRINT
// ============================================================

void configPrint(const WeatherConfig &cfg){
    Serial.println();
    Serial.println(F("----------- CONFIG -----------"));

    Serial.print(F("Node ID: "));
    Serial.println(cfg.nodeId);

    Serial.print(F("NRF channel: "));
    Serial.println(cfg.nrfChannel);

    Serial.print(F("NRF power: "));
    Serial.println(cfg.nrfPower);

    Serial.print(F("NRF data rate: "));
    Serial.println(cfg.nrfDataRate);

    Serial.print(F("Send interval: "));
    Serial.print(cfg.sendIntervalMs);
    Serial.println(F(" ms"));

    Serial.print(F("Sensors flags: 0x"));
    Serial.println(cfg.sensorFlags, HEX);

    Serial.print(F("Temp offset: "));
    Serial.print(cfg.tempOffsetC100 / 100.0f);
    Serial.println(F(" C"));

    Serial.print(F("Humidity offset: "));
    Serial.print(cfg.humidityOffsetC100 / 100.0f);
    Serial.println(F(" %"));

    Serial.print(F("Pressure offset: "));
    Serial.print(cfg.pressureOffsetPa);
    Serial.println(F(" Pa"));

    Serial.print(F("Sea level pressure: "));
    Serial.print(cfg.seaLevelPressurePa);
    Serial.println(F(" Pa"));

    Serial.println(F("------------------------------"));
}

// ============================================================
// WALIDACJA
// ============================================================

bool configValidate(WeatherConfig &cfg){
    //cfg.nodeId[12] = 0;

    if(strlen(cfg.nodeId) > 255)
        return false;

    if(cfg.nrfChannel > 125)
        return false;

    if(cfg.nrfPower > 3)
        return false;

    if(cfg.nrfDataRate > 2)
        return false;

    if(cfg.sendIntervalMs < 100)
        return false;

    if(cfg.sendIntervalMs > 60000)
        return false;

    if(cfg.sensorFlags == 0)
        return false;

    if(cfg.seaLevelPressurePa < 80000UL)
        return false;

    if(cfg.seaLevelPressurePa > 120000UL)
        return false;

    return true;
}

// ============================================================
// POMOCNICZE PARSOWANIE
// ============================================================

char *nextField(char *&ptr){
    if(ptr == NULL)
        return NULL;

    char *start = ptr;

    char *separator = strchr(ptr, '|');

    if(separator){
        *separator = 0;
        ptr = separator + 1;
    }else{
        ptr = NULL;
    }

    return start;
}

// ============================================================
// SERIAL CONFIG
//
// GET
//
// SET|NODE|CHANNEL|POWER|RATE|INTERVAL|SENSORS|SEA|TOFF|HOFF|POFF
//
// RESTART
// ============================================================

void configProcessCommand(char *cmd){
    // usuń CR/LF
    char *p = strchr(cmd, '\r');
    if(p) *p = '\0';

    p = strchr(cmd, '\n');
    if(p) *p = '\0';

    // usuń spacje z początku
    while(*cmd == ' ' || *cmd == '\t')
        cmd++;

    // GET
    if(strcmp(cmd, "GET") == 0){
        configSend();
        return;
    }

    // PING
    /*if(strcmp(cmd, "PING") == 0){
        Serial.println(F("PONG"));
        return;
    }*/

    // CONFIG
    /*if(strcmp(cmd, "CONFIG") == 0){
        Serial.println(F("CONFIG_OK"));
        return;
    }*/

    // RESTART
    if(strcmp(cmd, "RESTART") == 0){
        Serial.println(F("RESTARTING"));
        Serial.flush();

        delay(100);

        wdt_enable(WDTO_15MS);

        while(true){}

        return;
    }

    // SET
    if(strncmp(cmd, "SET|", 4) == 0){
        char *ptr = cmd + 4;

        char *fNode = nextField(ptr);
        char *fChannel = nextField(ptr);
        char *fPower = nextField(ptr);
        char *fRate = nextField(ptr);
        char *fInterval = nextField(ptr);
        char *fSensors = nextField(ptr);
        char *fSea = nextField(ptr);
        char *fTemp = nextField(ptr);
        char *fHum = nextField(ptr);
        char *fPressure = nextField(ptr);

        if(!fNode || !fChannel || !fPower || !fRate || !fInterval || !fSensors || !fSea || !fTemp || !fHum || !fPressure){
            Serial.println(F("SET_ERROR|FORMAT"));
            return;
        }

        WeatherConfig newConfig;

        memset(&newConfig, 0, sizeof(newConfig));
        //strncpy(newConfig.nodeId, fNode, sizeof(newConfig.nodeId) - 1);

        char *nodeEnd;
        long nodeValue = strtol(fNode, &nodeEnd, 10);

        if(*fNode == '\0' || *nodeEnd != '\0' || nodeValue < 0 || nodeValue > 255){
            Serial.println(F("SET_ERROR|NODE"));
            return;
        }

        newConfig.nodeId = (uint8_t)nodeValue;

        newConfig.nrfChannel = atoi(fChannel);
        newConfig.nrfPower = atoi(fPower);
        newConfig.nrfDataRate = atoi(fRate);
        newConfig.sendIntervalMs = atol(fInterval);
        newConfig.sensorFlags = atoi(fSensors);
        newConfig.seaLevelPressurePa = atol(fSea);
        newConfig.tempOffsetC100 = atoi(fTemp);
        newConfig.humidityOffsetC100 = atoi(fHum);
        newConfig.pressureOffsetPa = atoi(fPressure);

        if(!configValidate(newConfig)){
            Serial.println(F("SET_ERROR|VALUE"));
            return;
        }

        if(!configSave(newConfig)){
            Serial.println(F("SET_ERROR|EEPROM"));
            return;
        }

        config = newConfig;

        Serial.println(F("SAVE_OK"));

        return;
    }


    Serial.print(F("ERR|UNKNOWN_COMMAND|"));
    Serial.println(cmd);
}

void configSend(){
    Serial.print(F("CFG|"));

    Serial.print(config.nodeId);
    Serial.print('|');

    Serial.print(config.nrfChannel);
    Serial.print('|');

    Serial.print(config.nrfPower);
    Serial.print('|');

    Serial.print(config.nrfDataRate);
    Serial.print('|');

    Serial.print(config.sendIntervalMs);
    Serial.print('|');

    Serial.print(config.sensorFlags);
    Serial.print('|');

    Serial.print(config.seaLevelPressurePa);
    Serial.print('|');

    Serial.print(config.tempOffsetC100);
    Serial.print('|');

    Serial.print(config.humidityOffsetC100);
    Serial.print('|');

    Serial.println(config.pressureOffsetPa);
}