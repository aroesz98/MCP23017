#include "MCP23017.h"

/***************************************************************************************
** Function name:		MCP23017
** Function name:		MCP23017 Class Constructor
***************************************************************************************/
MCP23017::MCP23017(uint8_t address, I2C_HandleTypeDef &bus) {
	_deviceAddr = address;
	_bus = &bus;
}

/***************************************************************************************
** Function name:		MCP23017
** Function name:		MCP23017 Class Destructor
***************************************************************************************/
MCP23017::~MCP23017() {}

/***************************************************************************************
** Function name:		init
** Function name:		Initialize MCP23017 with pull-up on GPIO's.
***************************************************************************************/
void MCP23017::init()
{
	writeRegister(_MCP23017_regs::IOCON, 0b00100000);
	writeRegister(_MCP23017_regs::GPPU_A, 0xFF, 0xFF);
}

/***************************************************************************************
** Function name:		portMode
** Description:			Set mode of selected gpio port (Input / Output), Pull-Up's, pin inversion
***************************************************************************************/
void MCP23017::portMode(MCP23017Port port, uint8_t directions, uint8_t pullups, uint8_t inverted)
{
	writeRegister(_MCP23017_regs::IODIR_A + port, directions);
	writeRegister(_MCP23017_regs::GPPU_A + port, pullups);
	writeRegister(_MCP23017_regs::IPOL_A + port, inverted);
}

/***************************************************************************************
** Function name:		pinMode
** Description:			Set mode of selected gpio pin (Input / Output), pin inversion
***************************************************************************************/
void MCP23017::pinMode(uint8_t pin, uint8_t mode, bool inverted)
{
	_MCP23017_regs iodirreg = _MCP23017_regs::IODIR_A;
	_MCP23017_regs pullupreg = _MCP23017_regs::GPPU_A;
	_MCP23017_regs polreg = _MCP23017_regs::IPOL_A;
	uint8_t iodir, pol, pull;

	if(pin > 7)
	{
		iodirreg = _MCP23017_regs::IODIR_B;
		pullupreg = _MCP23017_regs::GPPU_B;
		polreg = _MCP23017_regs::IPOL_B;
		pin -= 8;
	}

	iodir = readRegister(iodirreg);
	if(mode == INPUT || mode == INPUT_PULLUP) bitSet(iodir, pin);
	else bitClear(iodir, pin);

	pull = readRegister(pullupreg);
	if(mode == INPUT_PULLUP) bitSet(pull, pin);
	else bitClear(pull, pin);

	pol = readRegister(polreg);
	if(inverted) bitSet(pol, pin);
	else bitClear(pol, pin);

	writeRegister(iodirreg, iodir);
	writeRegister(pullupreg, pull);
	writeRegister(polreg, pol);
}

/***************************************************************************************
** Function name:		writePin
** Description:			Write state for selected GPIO Pin.
***************************************************************************************/
void MCP23017::writePin(uint8_t pin, uint8_t state)
{
	_MCP23017_regs gpioreg = _MCP23017_regs::GPIO_A;
	uint8_t gpio;
	if(pin > 7)
	{
		gpioreg = _MCP23017_regs::GPIO_B;
		pin -= 8;
	}

	gpio = readRegister(gpioreg);
	if(state == HIGH) bitSet(gpio, pin);
	else bitClear(gpio, pin);

	writeRegister(gpioreg, gpio);
}

/***************************************************************************************
** Function name:		writePin
** Description:			Read state for selected GPIO Pin.
***************************************************************************************/
uint8_t MCP23017::readPin(uint8_t pin)
{
	_MCP23017_regs gpioreg = _MCP23017_regs::GPIO_A;
	uint8_t gpio;
	if(pin > 7)
	{
		gpioreg = _MCP23017_regs::GPIO_B;
		pin -=8;
	}

	gpio = readRegister(gpioreg);
	if(bitRead(gpio, pin)) return HIGH;
	return LOW;
}

/***************************************************************************************
** Function name:		writePort
** Description:			Write value for selected GPIO Port.
***************************************************************************************/
void MCP23017::writePort(MCP23017Port port, uint8_t value)
{
	writeRegister(_MCP23017_regs::GPIO_A + port, value);
}


void MCP23017::writeABPort(uint16_t value)
{
	writeRegister(_MCP23017_regs::GPIO_A, lowByte(value), highByte(value));
}

/***************************************************************************************
** Function name:		readPort
** Description:			Read value for selected GPIO Port.
***************************************************************************************/
uint8_t MCP23017::readWholePort(MCP23017Port port)
{
	return readRegister(_MCP23017_regs::GPIO_A + port);
}

uint16_t MCP23017::readWholeIO()
{
	uint8_t a = readWholePort(MCP23017Port::A);
	uint8_t b = readWholePort(MCP23017Port::B);

	return a | b << 8;
}

/***************************************************************************************
** Function name:		writeRegister
** Description:			Write value for selected register.
***************************************************************************************/
void MCP23017::writeRegister(_MCP23017_regs reg, uint8_t value)
{
	HAL_I2C_Master_Transmit(_bus, _deviceAddr << 1, reinterpret_cast<uint8_t*>(&reg), 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(_bus, _deviceAddr << 1, reinterpret_cast<uint8_t*>(value), 1, HAL_MAX_DELAY);
}

void MCP23017::writeRegister(_MCP23017_regs reg, uint8_t portA, uint8_t portB)
{
	HAL_I2C_Master_Transmit(_bus, _deviceAddr << 1, reinterpret_cast<uint8_t*>(&reg), 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(_bus, _deviceAddr << 1, reinterpret_cast<uint8_t*>(portA), 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(_bus, _deviceAddr << 1, reinterpret_cast<uint8_t*>(portB), 1, HAL_MAX_DELAY);
}

/***************************************************************************************
** Function name:		readRegister
** Description:			Read value for selected register.
***************************************************************************************/
uint8_t MCP23017::readRegister(_MCP23017_regs reg)
{
	uint8_t data = 0;
	HAL_I2C_Master_Transmit(_bus, _deviceAddr << 1, reinterpret_cast<uint8_t*>(&reg), 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(_bus, _deviceAddr << 1, &data, 1, HAL_MAX_DELAY);
	return data;
}

void MCP23017::readRegister(_MCP23017_regs reg, uint8_t& portA, uint8_t& portB)
{
	uint8_t data[2];
	HAL_I2C_Master_Transmit(_bus, _deviceAddr << 1, reinterpret_cast<uint8_t*>(&reg), 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(_bus, _deviceAddr << 1, (uint8_t*)data, 2, HAL_MAX_DELAY);
	portA = data[0];
	portB = data[1];
}

#ifdef _MCP23017_INTERRUPT_SUPPORT_

/***************************************************************************************
** Function name:		interruptMode
** Description:			Set interrupt mode (Independent / Mirrored).
***************************************************************************************/
void MCP23017::interruptMode(MCP23017InterruptMode intMode)
{
	uint8_t iocon = readRegister(_MCP23017_regs::IOCON);
	if(intMode == MCP23017InterruptMode::Or) iocon |= static_cast<uint8_t>(MCP23017InterruptMode::Or);
	else iocon &= ~(static_cast<uint8_t>(MCP23017InterruptMode::Or));

	writeRegister(_MCP23017_regs::IOCON, iocon);
}

/***************************************************************************************
** Function name:		interruptMode
** Description:			Set interrupt mode for selected port (Independent / Mirrored).
***************************************************************************************/
void MCP23017::interrupt(MCP23017Port port, uint8_t mode)
{
	_MCP23017_regs defvalreg = _MCP23017_regs::DEFVAL_A + port;
	_MCP23017_regs intconreg = _MCP23017_regs::INTCON_A + port;

	//enable interrupt for port
	writeRegister(_MCP23017_regs::GPINTEN_A + port, 0xFF);
	switch(mode)
	{
	case CHANGE:
		//interrupt on change
		writeRegister(intconreg, 0);
		break;
	case FALLING:
		//interrupt falling : compared against defval, 0xff
		writeRegister(intconreg, 0xFF);
		writeRegister(defvalreg, 0xFF);
		break;
	case RISING:
		//interrupt rising : compared against defval, 0x00
		writeRegister(intconreg, 0xFF);
		writeRegister(defvalreg, 0x00);
		break;
	}
}

/***************************************************************************************
** Function name:		interruptedBy
** Description:			Check interrupts what came from.
***************************************************************************************/
void MCP23017::interruptedBy(uint8_t& portA, uint8_t& portB)
{
	readRegister(_MCP23017_regs::INTF_A, portA, portB);
}

/***************************************************************************************
** Function name:		disableInterrupt
** Description:			Disable external interrupts.
***************************************************************************************/
void MCP23017::disableInterrupt(MCP23017Port port)
{
	writeRegister(_MCP23017_regs::GPINTEN_A + port, 0x00);
}

/***************************************************************************************
** Function name:		clearInterrupts
** Description:			Clear interrupt flags.
***************************************************************************************/
void MCP23017::clearInterrupts()
{
	uint8_t a, b;
	clearInterrupts(a, b);
}

void MCP23017::clearInterrupts(uint8_t& portA, uint8_t& portB)
{
	readRegister(_MCP23017_regs::INTCAP_A, portA, portB);
}

#endif
