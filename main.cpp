#include "mbed.h"

/**
 * A simple library for SHT21 humidity and temperature sensor.
 * Based on the Arduino library.
 */

class SHT21 {
public:
    SHT21(PinName sda, PinName scl, char addr = (0x40 << 1));
    SHT21(I2C &i2c,  char addr = (0x40 << 1));
    
    float temperature();
    float humidity();
    
    void reset();
    void serialNumber(uint8_t *serialNumber);
    
    ~SHT21();
private:
    enum commands {
        TRIGGER_T_MEASUREMENT_HM    = 0xE3,
        TRIGGER_RH_MEASUREMENT_HM   = 0xE5,
        TRIGGER_T_MEASUREMENT_NHM   = 0xF3,
        TRIGGER_RH_MEASUREMENT_NHM  = 0xF5,
        USER_REGISTER_W             = 0xE6,
        USER_REGISTER_R             = 0xE7,
        SOFT_RESET                  = 0xFE
    };

    static const uint16_t POLYNOMIAL;

    char addr_;
    I2C *pi2c_;
    I2C &i2c_;
    
    uint16_t readRaw(uint8_t cmd);
    static bool crcChecksum(const uint8_t *data, uint8_t sz, uint8_t checksum);
};

const uint16_t SHT21::POLYNOMIAL = 0x131;

SHT21::SHT21(PinName sda, PinName scl, char addr) :
    addr_(addr),
    pi2c_(new I2C(sda, scl)),
    i2c_(*pi2c_) {
}

SHT21::SHT21(I2C &i2c, char addr) :
    addr_(addr),
    pi2c_(NULL),
    i2c_(i2c) {
}

float SHT21::temperature() {
    uint16_t r = readRaw(TRIGGER_T_MEASUREMENT_HM);
    r &= ~0x0003;
    return (-46.85 + 175.72/65536 * (float) r);
}

float SHT21::humidity() {
    uint16_t r = readRaw(TRIGGER_RH_MEASUREMENT_HM);
    r &= ~0x0003;
    return (-6.0 + 125.0/65536 * (float) r);
}

void SHT21::reset() {
    uint8_t pcmd [] = { SOFT_RESET };
    i2c_.write(addr_, (const char *) pcmd, 1);
    wait_ms(15);
}

void SHT21::serialNumber(uint8_t *serialNumber) {
    uint8_t data[8];
    uint8_t pcmd0[2] = { 0xFA, 0x0F };
    uint8_t pcmd1[2] = { 0xFC, 0xC9 };
    uint8_t i = 0;
    i2c_.write(addr_, (const char *) pcmd0, 2);
    i2c_.read(addr_, (char *) data, 8);
    
    serialNumber[5] = data[i++];
    i++;
    serialNumber[4] = data[i++];
    i++;
    serialNumber[3] = data[i++];
    i++;
    serialNumber[2] = data[i++];
    i = 0;
    
    i2c_.write(addr_, (const char *) pcmd1, 2);
    i2c_.read(addr_, (char *) data, 6);
    
    serialNumber[1] = data[i++];
    serialNumber[0] = data[i++];
    i++;
    serialNumber[7] = data[i++];
    serialNumber[8] = data[i++];
    i = 0;
}

SHT21::~SHT21() {
    if (pi2c_) delete pi2c_;
}

uint16_t SHT21::readRaw(uint8_t cmd) {
    uint8_t checksum;
    uint8_t data[2];
    uint16_t result;

    uint8_t pcmd [] = { cmd };

    i2c_.write(addr_, (const char *) pcmd, 1);
    i2c_.read(addr_, (char *) data, 2);
    i2c_.read(addr_, (char *) &checksum, 1);

    result =    data[0] << 8;
    result +=   data[1];

    if (crcChecksum(data, 2, checksum)) {
        reset();
        return 1;
    }
    return result;
}

bool SHT21::crcChecksum(const uint8_t *data, uint8_t sz, uint8_t checksum) {
    uint8_t crc = 0;
    
    for (uint8_t i = 0; i < sz; ++i) {
        crc ^= data[i];
        for (uint8_t j /* bit index */ = 8; j > 0; --j) {
            if (crc & 0x80)
                crc = (crc << 1) ^ POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    
    return crc == checksum;
}

SHT21 sht21(PB_9 /* I2C_SDA */, PB_8 /* I2C_SCL */);
Serial pc(USBTX, USBRX);

int main() {
    while(1) {
        pc.printf(
            "%2.2f degC, %2.2f %%\n",
            sht21.temperature(),
            sht21.humidity());
        wait(1);
    }
}
