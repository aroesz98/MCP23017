# MCP23017 Library for STM32 MCU's
This is a library for the MCP23017 I2C Port Expanders.
To use this library simply create MCP23017 object by calling:
    MCP23017 object_name(address, Bus)
    
Where address is between 0x20-0x27 and bus is your actually used i2c bus like hi2c1. After creating object call init() function and enjoy.
List of all usable functions:

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
