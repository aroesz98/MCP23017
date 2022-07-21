#pragma once

#include <i2c.h>

#define INT_RISING_EDGE		2
#define INT_FALLING_EDGE	1
#define INT_ON_CHANGE		0

#define STATE_LOW		0
#define STATE_HIGH		1

#define INPUT			0x0
#define OUTPUT			0x1
#define INPUT_PULLUP	0x2

#define readBit(value, bit) (((value) >> (bit)) & 0x01)
#define setBit(value, bit) ((value) |= (1UL << (bit)))
#define clearBit(value, bit) ((value) &= ~(1UL << (bit)))
#define writeBit(value, bit, bitvalue) ((bitvalue) ? setBit((value), (bit)) : clearBit((value), (bit) ))

/***************************************************************************************
** Class name:           	MCP23017Port
** Description:             GPIO Port defines
***************************************************************************************/
enum class MCP23017Port : uint8_t
{
	PORT_A = 0,
	PORT_B = 1
};

/***************************************************************************************
** Structure name:			MCP23017Pin
** Description:				GPIO Pin defines
***************************************************************************************/
enum MCP23017Pin {
	PA0 = 0,
	PA1 = 1,
	PA2 = 2,
	PA3 = 3,
	PA4 = 4,
	PA5 = 5,
	PA6 = 6,
	PA7 = 7,
	PB0 = 8,
	PB1 = 9,
	PB2 = 10,
	PB3 = 11,
	PB4 = 12,
	PB5 = 13,
	PB6 = 14,
	PB7 = 15
};

/***************************************************************************************
** Class name:           	MCP23017InterruptMode
** Description:             Controls if the two interrupt pins mirror each other
***************************************************************************************/
enum class MCP23017InterruptMode : uint8_t
{
	Separated = 0,											//Interrupt pins kept independent
	Or = 0b01000000											//Interrupt pins mirrored
};

/***************************************************************************************
** Class name:           	_MCP23017_regs
** Description:             List of all usable registers
***************************************************************************************/
enum class MCP23017Register : uint8_t
{
	IODIR_A		= 0x00, 									//Controls the direction of the data for port A.
	IODIR_B		= 0x01,										//Controls the direction of the data for port B.
	IPOL_A		= 0x02,										//Configures the polarity for port A.
	IPOL_B		= 0x03,										//Configures the polarity for port B.
	GPINTEN_A	= 0x04,										//Controls the interrupt-on-change for each pin of port A.
	GPINTEN_B	= 0x05,										//Controls the interrupt-on-change for each pin of port B.
	DEFVAL_A	= 0x06,										//Controls the default comparaison value for interrupt-on-change for port A.
	DEFVAL_B	= 0x07,										//Controls the default comparaison value for interrupt-on-change for port B.
	INTCON_A	= 0x08,										//Controls how the associated pin value is compared for the interrupt-on-change for port A.
	INTCON_B	= 0x09,										//Controls how the associated pin value is compared for the interrupt-on-change for port B.
	IOCON		= 0x0A,										//Controls the device.
	GPPU_A		= 0x0C,										//Controls the pull-up resistors for the port A pins.
	GPPU_B		= 0x0D,										//Controls the pull-up resistors for the port B pins.
	INTF_A		= 0x0E,										//Reflects the interrupt condition on the port A pins.
	INTF_B		= 0x0F,										//Reflects the interrupt condition on the port B pins.
	INTCAP_A	= 0x10,										//Captures the port A value at the time the interrupt occured.
	INTCAP_B	= 0x11,										//Captures the port B value at the time the interrupt occured.
	GPIO_A		= 0x12,										//Reflects the value on the port A.
	GPIO_B		= 0x13,										//Reflects the value on the port B.
	OLAT_A		= 0x14,										//Provides access to the port A output latches.
	OLAT_B		= 0x15,										//Provides access to the port B output latches.
};

inline MCP23017Register operator+(MCP23017Register a, MCP23017Port b) {
	return static_cast<MCP23017Register>(static_cast<uint8_t>(a) + static_cast<uint8_t>(b));
};

class MCP23017 {
	private:
		I2C_HandleTypeDef *_bus;
		uint8_t _deviceAddr;

	public:
		MCP23017(uint8_t address, I2C_HandleTypeDef &Bus);
		MCP23017(I2C_HandleTypeDef &bus);
		~MCP23017();

		void init();

		void portMode(MCP23017Port port, uint8_t directions, uint8_t pullups = 0xFF, uint8_t inverted = 0x00);
		void pinMode(uint8_t pin, uint8_t mode, bool inverted = false);

		void writePin(uint8_t pin, uint8_t state);
		uint8_t readPin(uint8_t pin);

		void writePort(MCP23017Port port, uint8_t value);
		void writeWholeIO(uint16_t value);

		uint8_t readPort(MCP23017Port port);
		uint16_t readWholeIO();

		void writeRegister(MCP23017Register reg, uint8_t value);
		uint8_t readRegister(MCP23017Register reg);
		void readRegister(MCP23017Register reg, uint8_t& portA, uint8_t& portB);

		void interruptMode(MCP23017InterruptMode intMode);
		void interrupt(MCP23017Port port, uint8_t mode);
		void disableInterrupt(MCP23017Port port);
		void interruptedBy(uint8_t& portA, uint8_t& portB);
		void clearInterrupts();
		void clearInterrupts(uint8_t& portA, uint8_t& portB);
};
