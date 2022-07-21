#pragma once

#include <i2c.h>

#define MCP23017_I2C_ADDRESS 0x20    //The default I2C address of MCP23017.
#define _MCP23017_INTERRUPT_SUPPORT_ //Enables support for MCP23017 interrupts.

#define RISING	2
#define FALLING	1
#define CHANGE	0

#define LOW		0
#define HIGH	1

#define INPUT			0x0
#define OUTPUT			0x1
#define INPUT_PULLUP	0x2

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet((value), (bit)) : bitClear((value), (bit) ))

enum class MCP23017Port : uint8_t
{
	A = 0,
	B = 1
};

struct MCP23017Pin
{
	enum Names {
		GPA0 = 0,
		GPA1,
		GPA2,
		GPA3,
		GPA4,
		GPA5,
		GPA6,
		GPA7,
		GPB0 = 8,
		GPB1,
		GPB2,
		GPB3,
		GPB4,
		GPB5,
		GPB6,
		GPB7
	};
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
enum class _MCP23017_regs : uint8_t
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

inline _MCP23017_regs operator+(_MCP23017_regs a, MCP23017Port b) {
	return static_cast<_MCP23017_regs>(static_cast<uint8_t>(a) + static_cast<uint8_t>(b));
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
		void writeABPort(uint16_t value);

		uint8_t readWholePort(MCP23017Port port);
		uint16_t readWholeIO();

		void writeRegister(_MCP23017_regs reg, uint8_t value);
		void writeRegister(_MCP23017_regs reg, uint8_t portA, uint8_t portB);
		uint8_t readRegister(_MCP23017_regs reg);
		void readRegister(_MCP23017_regs reg, uint8_t& portA, uint8_t& portB);

	#ifdef _MCP23017_INTERRUPT_SUPPORT_

		void interruptMode(MCP23017InterruptMode intMode);
		void interrupt(MCP23017Port port, uint8_t mode);
		void disableInterrupt(MCP23017Port port);
		void interruptedBy(uint8_t& portA, uint8_t& portB);
		void clearInterrupts();
		void clearInterrupts(uint8_t& portA, uint8_t& portB);

	#endif
};
