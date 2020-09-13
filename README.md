# STM32F103C8_USB_HID_PIROMETR
This project describes a USB pyrometric temperature sensor on STM32F103 and MLX90614.
## Main features:
1. Bare metal programming style for USB and I2C. No bulky libraries. There's only the necessary functions, correctly developed according to USB 2.0 and HID 1.11 protocols.
This is very well suited for developers who want to learn deeply into the algorithm of the USB and I2C  peripherals about STM32. You can see low level very simple unlike HAL library. USB contained only in 2 files: USB_packet.c, and USB_transaction.c. USB_packet.c described OSI packet level and lower. USB_transaction.c decribed transport OSI level and higer.

2. All work is built in interrupts. The main loop used only for Log output througt the SWO interface.  It will be completely free when logging is disabled.

3. The  I2C state machine is developed with errors control on the bus. It's does not glutch with any collisions: breaks or short circuits to zero or to each other.

4. USB works according to the HID protocol. The HID descriptor describes the standard class Sensor Thermometer. You don't need drivers for it. Unlike Custom Hid, you will see it  exactly as a Thermometer in the device manager, and you will be able to work from a high-level application  as a sensor, and not as an unknown Custom HID device. In the standard descriptor from "hutrr39b_0.pdf " added the ability to change a polling interval for reports.

5. There is great logging. Every interrupt, every request for a descriptor, every data transfer is written to the Log buffer. The transfer from the log buffer to the development environment occurs via the SWO interface at a speed of about 2 Mbps.


6. The main MAIN cycle uses only data sending via the SWO interface. When you finished device, you do not need debugging. So a disabling it frees up the main loop for your tasks. You do not need UART, etc. for this

7. The project is provided with extensive comments. When I declare constants or defines,  I give a link on a document ( page, table) where it is specified. It helps a lot with an abundance of new information regarding USB
***
In Device manager it look like as a standard Sensor
![how it look like in windows Device manager](https://github.com/mcSimoffa/STM32F103C8_USB_HID_PIROMETR/blob/master/photos/Temperature%20Sensor.jpg)
***
In Windows Control panel\Devive it look like as a how I called it "Pirometer"
![how it look like in windows Devices](https://github.com/mcSimoffa/STM32F103C8_USB_HID_PIROMETR/blob/master/photos/Device.jpg)
*I develop the principial circuit by DipTrace soft. 
https://github.com/mcSimoffa/STM32F103C8_USB_HID_PIROMETR/blob/master/Circuit%20Diagramm/DipTrace%20Schematic%20-%20USB_prometr.pdf

You must remove the built-in 1.5k resistor from the board and add a controlled pull-up circuit. This is necessary to simplify debugging, so as not to juggle the USB plug.
You can disconnect this circuit and restore the resistor in the finished device.
![how prototype look like](https://github.com/mcSimoffa/STM32F103C8_USB_HID_PIROMETR/blob/master/photos/IMG_3338.JPG)
You can download this utilite together Windows Development Kits. 
![the diagnostic utility sees my device like this](https://github.com/mcSimoffa/STM32F103C8_USB_HID_PIROMETR/blob/master/photos/SensorToolDiagnostic.jpg)
***
Log file look like this. You can  enadled or disabled SWO_USB_LOG and SWO_SMBUS_LOG separatly and undepended
![the Log information look like this](https://github.com/mcSimoffa/STM32F103C8_USB_HID_PIROMETR/blob/master/photos/log.jpg)
***
Development Enviroment  IAR Embedded workbench for ARM 8.50.6
## Thanks 
https://github.com/katyo/hid_def 
for the idea
