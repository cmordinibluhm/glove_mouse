Arduino and Teensyduino sketches for the Glove Mouse armband and base station.

**armband.ino** is for upload to the Arduino on the armband. It reads the sensor data and sends packets to the transmitting transceiver.  

**base_station.ino** is for upload to the Teensy++2.0 board on the base station. It receives packets from the receiving transceiver and performs the corresponding mouse actions on the PC.  

*base_station.ino requires the Teensyduino add-on for the Arduino IDE.*
