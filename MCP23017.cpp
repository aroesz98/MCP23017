/*
 * MCP23017
 * Author: Arkadiusz Szlanta
 * Date: 08 Dec 2024
 *
 * License:
 * This source code is provided for hobbyist and private use only.
 * Any commercial or industrial use, including distribution, reproduction, or
 * incorporation in commercial or industrial products or services is prohibited.
 * Use at your own risk. The author(s) hold no responsibility for any damages
 * or losses resulting from the use of this software.
 *
 */

#include "MCP23017.hpp"

MCP23017::MCP23017(uint8_t address, I2C_HandleTypeDef &bus)
{
    _deviceAddr = address << 1;
    _bus = &bus;
}

MCP23017::~MCP23017() {}

void MCP23017::init()
{
    writeRegister(MCP23017Register::IOCON, 0b00100000);

    writeRegister(MCP23017Register::GPPU_A, 0xFF);
    writeRegister(MCP23017Register::GPPU_B, 0xFF);

    portMode(MCP23017Port::PORT_A, 0);
    portMode(MCP23017Port::PORT_B, 0);

    writeRegister(MCP23017Register::GPIO_A, 0x00);
    writeRegister(MCP23017Register::GPIO_B, 0x00);
}

void MCP23017::portMode(MCP23017Port port, uint8_t directions, uint8_t pullups, uint8_t inverted)
{
    writeRegister(MCP23017Register::IODIR_A + port, directions);
    writeRegister(MCP23017Register::GPPU_A + port, pullups);
    writeRegister(MCP23017Register::IPOL_A + port, inverted);
}

void MCP23017::pinMode(uint8_t pin, uint8_t mode, bool inverted)
{
    MCP23017Register iodirreg = MCP23017Register::IODIR_A;
    MCP23017Register pullupreg = MCP23017Register::GPPU_A;
    MCP23017Register polreg = MCP23017Register::IPOL_A;
    uint8_t iodir, pol, pull;

    if (pin > 7)
    {
        iodirreg = MCP23017Register::IODIR_B;
        pullupreg = MCP23017Register::GPPU_B;
        polreg = MCP23017Register::IPOL_B;
        pin -= 8;
    }

    iodir = readRegister(iodirreg);
    if (mode == INPUT || mode == INPUT_PULLUP)
    {
        setBit(iodir, pin);
    }
    else
    {
        clearBit(iodir, pin);
    }

    pull = readRegister(pullupreg);
    if (mode == INPUT_PULLUP)
    {
        setBit(pull, pin);
    }
    else
    {
        clearBit(pull, pin);
    }

    pol = readRegister(polreg);
    if (inverted)
    {
        setBit(pol, pin);
    }
    else
    {
        clearBit(pol, pin);
    }

    writeRegister(iodirreg, iodir);
    writeRegister(pullupreg, pull);
    writeRegister(polreg, pol);
}

void MCP23017::writePin(uint8_t pin, uint8_t state)
{
    MCP23017Register gpioreg = MCP23017Register::GPIO_A;
    uint8_t gpio;

    if (pin > 7)
    {
        gpioreg = MCP23017Register::GPIO_B;
        pin -= 8;
    }

    gpio = readRegister(gpioreg);
    if (state == STATE_HIGH)
    {
        setBit(gpio, pin);
    }
    else
    {
        clearBit(gpio, pin);
    }

    writeRegister(gpioreg, gpio);
}

uint8_t MCP23017::readPin(uint8_t pin)
{
    MCP23017Register gpioreg = MCP23017Register::GPIO_A;
    uint8_t gpio;

    if (pin > 7)
    {
        gpioreg = MCP23017Register::GPIO_B;
        pin -= 8;
    }

    gpio = readRegister(gpioreg);
    if (readBit(gpio, pin))
    {
        return STATE_HIGH;
    }

    return STATE_LOW;
}

void MCP23017::writePort(MCP23017Port port, uint8_t value)
{
    writeRegister(MCP23017Register::GPIO_A + port, value);
}

void MCP23017::writeWholeIO(uint16_t value)
{
    writeRegister(MCP23017Register::GPIO_A, (value & 0xFF));
    writeRegister(MCP23017Register::GPIO_B, (value >> 8));
}

uint8_t MCP23017::readPort(MCP23017Port port)
{
    return readRegister(MCP23017Register::GPIO_A + port);
}

uint16_t MCP23017::readWholeIO()
{
    uint8_t a = readPort(MCP23017Port::PORT_A);
    uint8_t b = readPort(MCP23017Port::PORT_B);

    return a | b << 8;
}

void MCP23017::writeRegister(MCP23017Register reg, uint8_t value)
{
    HAL_I2C_Mem_Write(_bus, _deviceAddr, (uint16_t)reg, 1, &value, 1, 100);
}

uint8_t MCP23017::readRegister(MCP23017Register reg)
{
    uint8_t data = 0;
    HAL_I2C_Mem_Read(_bus, _deviceAddr, (uint16_t)reg, 1, &data, 1, 100);
    return data;
}

void MCP23017::readRegister(MCP23017Register reg, uint8_t &portA, uint8_t &portB)
{
    uint8_t data[2];
    HAL_I2C_Mem_Read(_bus, _deviceAddr, (uint16_t)reg, 1, (uint8_t *)data, 2, 100);
    portA = data[0];
    portB = data[1];
}

void MCP23017::interruptMode(MCP23017InterruptMode intMode)
{
    uint8_t iocon = readRegister(MCP23017Register::IOCON);
    if (intMode == MCP23017InterruptMode::Or)
    {
        iocon |= static_cast<uint8_t>(MCP23017InterruptMode::Or);
    }
    else
    {
        iocon &= ~(static_cast<uint8_t>(MCP23017InterruptMode::Or));
    }

    writeRegister(MCP23017Register::IOCON, iocon);
}

void MCP23017::interrupt(MCP23017Port port, uint8_t mode)
{
    MCP23017Register defvalreg = MCP23017Register::DEFVAL_A + port;
    MCP23017Register intconreg = MCP23017Register::INTCON_A + port;

    writeRegister(MCP23017Register::GPINTEN_A + port, 0xFF);

    switch (mode)
    {
        case INT_ON_CHANGE:
            writeRegister(intconreg, 0);
            break;

        case INT_FALLING_EDGE:
            writeRegister(intconreg, 0xFF);
            writeRegister(defvalreg, 0xFF);
            break;

        case INT_RISING_EDGE:
            writeRegister(intconreg, 0xFF);
            writeRegister(defvalreg, 0x00);
            break;
    }
}

void MCP23017::interruptedBy(uint8_t &portA, uint8_t &portB)
{
    readRegister(MCP23017Register::INTF_A, portA, portB);
}

void MCP23017::disableInterrupt(MCP23017Port port)
{
    writeRegister(MCP23017Register::GPINTEN_A + port, 0x00);
}

void MCP23017::clearInterrupts()
{
    uint8_t a, b;
    clearInterrupts(a, b);
}

void MCP23017::clearInterrupts(uint8_t &portA, uint8_t &portB)
{
    readRegister(MCP23017Register::INTCAP_A, portA, portB);
}

