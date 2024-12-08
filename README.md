# MCP23017 Library for STM32 MCU's

## Implementation Description:
The following implementation contains the `MCP23017` class, which is used to operate the MCP23017 I/O expander via the I2C bus. The class allows configuration, control, and reading of the states of the pins and ports of the expander.

## Functionality Description:

1. **Constructor and Destructor:**
   - The constructor initializes the device address and the handle to the I2C bus.
   - The default destructor does not perform any operations.

2. **Initialization:**
   - The `init` method configures the IOCON, GPPU_A, and GPPU_B registers, sets the port mode to output, and resets the states of the GPIO_A and GPIO_B ports. This prepares the expander for operation by setting the appropriate initial parameters.

3. **Port Configuration:**
   - The `portMode` method sets the mode of operation for the ports (direction, pull-up, inversion).

4. **Pin Configuration:**
   - The `pinMode` method configures the mode of operation for a specific pin (input, output, pull-up, inversion).

5. **Pin Operations:**
   - The `writePin` method sets the state of a specific pin to HIGH or LOW.
   - The `readPin` method reads the state of a specific pin (HIGH or LOW).

6. **Port Operations:**
   - The `writePort` method sets the value of an entire port (8 pins).
   - The `writeWholeIO` method sets the values of all pins (16 pins).
   - The `readPort` method reads the value of an entire port.
   - The `readWholeIO` method reads the values of all pins.

7. **Register Operations:**
   - The `writeRegister` and `readRegister` methods write to and read the values of the expander's registers via the I2C bus.

8. **Interrupt Configuration:**
   - The `interruptMode` method sets the interrupt mode (AND/OR).
   - The `interrupt` method configures the interrupt triggering method (state change, falling edge, rising edge).
   - The `interruptedBy`, `disableInterrupt`, and `clearInterrupts` methods handle checking, disabling, and clearing interrupts.

## Function Descriptions:

1. **Constructor:**
   - **Purpose:** Initializes the MCP23017 object with the device address and I2C bus handle.
   - **Parameters:**
     - `uint8_t address`: The I2C address of the MCP23017 device.
     - `I2C_HandleTypeDef &bus`: The I2C bus handle for communication.

2. **Destructor:**
   - **Purpose:** Cleans up the MCP23017 object. No specific operations are performed.

3. **init():**
   - **Purpose:** Initializes the MCP23017 device by configuring control registers and setting port modes and states.
   - **Actions:**
     - Configures IOCON register.
     - Enables pull-up resistors on ports A and B.
     - Sets ports A and B to output mode.
     - Resets the states of ports A and B.

4. **portMode(MCP23017Port port, uint8_t directions, uint8_t pullups, uint8_t inverted):**
   - **Purpose:** Sets the mode of operation for a specified port.
   - **Parameters:**
     - `MCP23017Port port`: The port to configure (PORT_A or PORT_B).
     - `uint8_t directions`: The direction of the pins (input or output).
     - `uint8_t pullups`: The pull-up resistor configuration.
     - `uint8_t inverted`: The polarity inversion configuration.

5. **pinMode(uint8_t pin, uint8_t mode, bool inverted):**
   - **Purpose:** Configures the mode of operation for a specific pin.
   - **Parameters:**
     - `uint8_t pin`: The pin number to configure.
     - `uint8_t mode`: The mode of the pin (input, output, input pull-up).
     - `bool inverted`: Indicates whether the pin state should be inverted.

6. **writePin(uint8_t pin, uint8_t state):**
   - **Purpose:** Sets the state of a specific pin to HIGH or LOW.
   - **Parameters:**
     - `uint8_t pin`: The pin number to write to.
     - `uint8_t state`: The state to set (HIGH or LOW).

7. **readPin(uint8_t pin):**
   - **Purpose:** Reads the state of a specific pin (HIGH or LOW).
   - **Parameters:**
     - `uint8_t pin`: The pin number to read from.
   - **Returns:** The state of the pin (HIGH or LOW).

8. **writePort(MCP23017Port port, uint8_t value):**
   - **Purpose:** Sets the value of an entire port.
   - **Parameters:**
     - `MCP23017Port port`: The port to write to (PORT_A or PORT_B).
     - `uint8_t value`: The value to set for the port.

9. **writeWholeIO(uint16_t value):**
   - **Purpose:** Sets the values of all pins (16 pins).
   - **Parameters:**
     - `uint16_t value`: The value to set for all pins.

10. **readPort(MCP23017Port port):**
    - **Purpose:** Reads the value of an entire port.
    - **Parameters:**
      - `MCP23017Port port`: The port to read from (PORT_A or PORT_B).
    - **Returns:** The value of the port.

11. **readWholeIO():**
    - **Purpose:** Reads the values of all pins.
    - **Returns:** The values of all pins as a 16-bit integer.

12. **writeRegister(MCP23017Register reg, uint8_t value):**
    - **Purpose:** Writes a value to a specified register.
    - **Parameters:**
      - `MCP23017Register reg`: The register to write to.
      - `uint8_t value`: The value to write to the register.

13. **readRegister(MCP23017Register reg):**
    - **Purpose:** Reads the value of a specified register.
    - **Parameters:**
      - `MCP23017Register reg`: The register to read from.
    - **Returns:** The value of the register.

14. **interruptMode(MCP23017InterruptMode intMode):**
    - **Purpose:** Sets the interrupt mode (AND/OR).
    - **Parameters:**
      - `MCP23017InterruptMode intMode`: The interrupt mode to set.

15. **interrupt(MCP23017Port port, uint8_t mode):**
    - **Purpose:** Configures the interrupt triggering method.
    - **Parameters:**
      - `MCP23017Port port`: The port to configure (PORT_A or PORT_B).
      - `uint8_t mode`: The interrupt triggering method (state change, falling edge, rising edge).

16. **interruptedBy(uint8_t &portA, uint8_t &portB):**
    - **Purpose:** Checks which pins triggered an interrupt.
    - **Parameters:**
      - `uint8_t &portA`: Reference to store the interrupt status of port A.
      - `uint8_t &portB`: Reference to store the interrupt status of port B.

17. **disableInterrupt(MCP23017Port port):**
    - **Purpose:** Disables interrupts on a specified port.
    - **Parameters:**
      - `MCP23017Port port`: The port to disable interrupts on (PORT_A or PORT_B).

18. **clearInterrupts():**
    - **Purpose:** Clears all interrupts.

19. **clearInterrupts(uint8_t &portA, uint8_t &portB):**
    - **Purpose:** Clears interrupts and reads the captured state of the pins.
    - **Parameters:**
      - `uint8_t &portA`: Reference to store the captured state of port A.
      - `uint8_t &portB`: Reference to store the captured state of port B.

## Flow diagram:
![codetoflow (7)](https://github.com/user-attachments/assets/72af5c17-35df-413e-9bcb-5f4146ca8fab)
## Class diagram:
![codetoflow (9)](https://github.com/user-attachments/assets/bfb758d8-a09b-4a44-9d27-d9f96581a8cb)
