#ifndef PCA9685_H
#define PCA9685_H

#include <cstdint>

class PCA9685
{
public:

    PCA9685(uint8_t address = 0x40);

    ~PCA9685();

    bool begin();

    void setPWMFreq(float freq);

    void setPWM(uint8_t channel,
                uint16_t on,
                uint16_t off);

private:

    int fd;

    uint8_t address;

    void write8(uint8_t reg,
                uint8_t data);

    uint8_t read8(uint8_t reg);
};

#endif