# seraphim-escape-arduino-joat

Designed to have one codebase for the arduinos, supports multiple inpust and outputs

At the moment it requires the Nrf24l01+ radio using an SPI interface into the arduino diagrams are a TODO

Also, because this requres a big serial buffer, you must edit your arduino.h header with the following at the top:

`#define SERIAL_TX_BUFFER_SIZE 160`

`#define SERIAL_RX_BUFFER_SIZE 160`

### Use Case:
To setup one open the arduino 

.ino project

select the JOAT.h header

device can either be master or slave (uncomment required one)

Give the device an id

Uncomment the device type, leave the rest commented out
