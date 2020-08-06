# Data Networking

Project completed as part of the Data Networking module I completed during my 3rd Year at the University of Manchester.

## Project Details 

### Setup 

EEPROM board connected to Nucleo-mbed shield-UM1601 assembly, which is mounted on an STM microcontroller 

### Description 

The program reads the value of temperature from the shield's sensor, and stores it in the Payload field of a packet of an experimental protocol. The packet is then stored in the EEPROM at an address given.

When the program starts, the MAC destination should be displayed. Thereafter, the program's behaviour is controlled by presses of the joystick.
- Joystick 'centre' button press: read the temperature value from the temperature sensor over I2C and display it on the LCD screen. Store the value in the Payload field of the packet in memory. Further 'centre' button presses result in the temperature value being read again and stored it in the packet, overwriting the previous value. Display a simple message to indicate the operation has completed.
- Joystick 'right' button press: write the entire 60-byte packet to the external EEPROM at I2C device address 0xA0, and starting at internal address 0x0000. Display a simple message to indicate the write operation has completed. Further 'right' button presses result in a repeat of the operation.
- Joystick 'left' button press: read the stored packet from the EEPROM (use 0xA1 when reading) and display it on the LCD. Display a simple message to indicate the read operation has completed. Further 'left' button presses result in a repeat of the operation.
- Joystick 'down' button press: display the next field of the packet. For example, if the MAC dest is currently displayed, then a single 'down' button press will result in the MAC src field being displayed instead. A further 'down' press results in the Length field being displayed, and so on. Once the last field of the packet is displayed, further 'down' presses have no effect, and the last field is continuously displayed.
- Joystick up button press: display the next field of the packet. For example, if the Payload is currently displayed, then a single 'up' button press will result in the Length field being displayed instead. A further 'up' press results in the MAC src field being displayed, etc. Once the first field of the packet is displayed, further 'up' presses have no effect, and the first field is continuously displayed.

In addition to the above aknowledge polling and CRC and CRC check were implemented.
