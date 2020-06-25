/* base_station.ino is for upload to the Teensy++2.0 MCU on the base station of the Glove Mouse.  
   
   It waits for a sync byte and an address byte and then reads the following 4 bytes into cursor
   movement, click, and scroll variables. These are then used to execute the mouse actions through 
   the Keyboard+Mouse mode of the Teensy IDE.
*/

//----------------------------------------------------------------------------------------------------//
//                                       DEBUGGING OPTIONS                                            //
//----------------------------------------------------------------------------------------------------//

bool serial_msgs = true; // True to print mouse actions to the serial monitor
bool print_packet = true; // True to print packet to the serial monitor

//----------------------------------------------------------------------------------------------------//
//                                  SERIAL COMMUNICATION VARIABLES                                    //
//----------------------------------------------------------------------------------------------------//

#define BAUDRATE 19200  // Baud rate for the radio transmitter

#define SYNC 0XAA // SYNC byte for packet
#define ADDR 0xFF // Address byte for packet

#define pixstep 5 // Pixel increment to move the cursor

volatile unsigned char sync;  // SYNC byte
volatile unsigned char addr;  // ADDRRESS byte
volatile unsigned char xAxis; // X Axis of the mouse
volatile unsigned char yAxis; // Y Axis of the mouse
volatile unsigned char wheel; // Scroll position
volatile unsigned char _click; // Information for mouse button clicks

volatile unsigned char rapid_fire;  // Flag for rapid fire clicks
volatile unsigned char firetime;  // Timeout counter for rapid fire
volatile unsigned char step_x;    // Step for x axis movement
volatile unsigned char step_y;    // Step for y axis movement

//----------------------------------------------------------------------------------------------------//
//                                           SET UP LOOP                                              //
//----------------------------------------------------------------------------------------------------//

void setup() {  
  //Mouse.screenSize(1440,900,true); // only available with Teensy 3.x +
  
  // set the teensy GPIO pin connected to the CMD humpro line to LOW to accept commands
  pinMode(PIN_D6, OUTPUT);
  digitalWrite(PIN_D6, LOW);
  
  Serial.begin(9600); // HumPRO transceivers start at 9600, so this is required during setup
  Serial1.begin(9600); // 58820 equivalent to Arduino 57600. Or set arduino to 57601 (this works better).

  // set the addressing mode to 0x07 (Extended User Addressing Mode)
  // ADDMODE register
  // 0xFF     0x02    0x04              0x07
  // header   size    non-vol address   value
  // 0x4F volatile addr, 0x04 non-volatile address
  // Default addresses are 0xFF for both receiver and transmit
  Serial1.write(0xFF);
  Serial1.write(0x02);
  Serial1.write(0x4F);
  Serial1.write(0x07);

/*
  // check that the ADDMODE has been set (desired output: 64F7)
  Serial1.write(0xFF);
  Serial1.write(0x02);
  Serial1.write(0xFE);
  Serial1.write(0x4F);
*/

  // change the data rate with the UARTBAUD register (default is 9600)
  // 0xFF     0x02    0x03                    V
  // header   size    non-volatile address    value
  // value is 0x04 for a baud rate of 57600 
  // power must be cycled for non-volatile changes
  Serial1.write(0xFF); // header
  Serial1.write(0x02); // size
  Serial1.write(0x4E); // volatile register address (UARTBAUD)
  Serial1.write(0x02); // set baud rate to 57600 (0x04), or 9600 (0x01), 19200(0x02), 38400(0x03)
  
  // reconfigure the Teensy baud rates to match the new HumPRO baud rate
  Serial.flush();
  Serial.begin(BAUDRATE);
  Serial1.flush();
  Serial1.begin(BAUDRATE);
  while(Serial.available() > 0) Serial.read(); // clear corrupted bytes
  while(Serial1.available() > 0) Serial1.read();

/*
  // check the new baud rate on HumPRO 
  Serial1.write(0xFF);
  Serial1.write(0x02);
  Serial1.write(0xFE);
  Serial1.write(0x4E);
*/
  // Pull HumPRO CMD pin HIGH to enter transmit mode
  digitalWrite(PIN_D6, HIGH);
  
  // Initialize flag for rapid fire mode
  rapid_fire = 0;

}

//----------------------------------------------------------------------------------------------------//
//                                              MAIN LOOP                                             //
//----------------------------------------------------------------------------------------------------//

/* Endless while loop to wait for data from transceiver.
*/
void loop() {
  
  // Receive messages in order if sync byte matches
  if (Serial1.available() > 0) {
    sync = Serial1.read();
  }
  
  if (sync == SYNC) { 
    
    // Receive further messages if address matches
    while (!Serial1.available()) {
    }
    addr = Serial1.read();

    if (addr == ADDR) {

      // Get the other data
       while (!Serial1.available()) {
    }
      xAxis = Serial1.read(); // Cursor movement on x axis
       while (!Serial1.available()) {
    }
      yAxis = Serial1.read(); // Cursor movement on y axis
       while (!Serial1.available()) {
    }
      wheel = Serial1.read(); // Scroll movement  
       while (!Serial1.available()) {
    }
      _click = Serial1.read(); // Click Status

      if (print_packet) {
        Serial.print(sync);
        Serial.print(addr);
        Serial.print(xAxis);
        Serial.print(yAxis);
        Serial.print(wheel);
        Serial.print(_click);
        Serial.println();
      }
        // If Port C.7 button is pressed    (Left Click)
      if ((_click & 0x80) == 0x80) { 
        Mouse.set_buttons(1, 0, 0);
        if(serial_msgs) {Serial.println("Left click");}
      }
      // If Port C.6 button is pressed  (Middle Click)  
      else if ((_click & 0x40) == 0x40) { 
        Mouse.set_buttons(0, 1, 0);
        if(serial_msgs) {Serial.println("Middle click"); }
      }
      // If Port C.5 button is pressed  (Right Click)
      else if ((_click & 0x20) == 0x20) {
        Mouse.set_buttons(0, 0, 1);
        if(serial_msgs) {Serial.println("Right click"); }
      }
      // If Port C.7 button is released   (Left Click release)
      else if ((_click & 0x01) == 0x01) { 
        Mouse.set_buttons(0, 0, 0);
        if(serial_msgs) {Serial.println("Left click release"); }

      }
      // If Port C.6 button is released   (Middle Click release)
      else if ((_click & 0x02) == 0x02) { 
        Mouse.set_buttons(0, 0, 0);
        if(serial_msgs) {Serial.println("Middle click release"); }

      }
      // If Port C.5 button is released   (Right Click release)
      else if ((_click & 0x04) == 0x04) { 
        Mouse.set_buttons(0, 0, 0);
        if(serial_msgs) {Serial.println("Right click release"); }

      }
      // If Port C.7 button is pressed in rapid fire mode (Repeated Left Clicks)
      else if ((_click & 0x08) == 0x08) { 
        if (rapid_fire == 1) rapid_fire = 0;
        else rapid_fire = 1;
        Mouse.set_buttons(rapid_fire, 0, 0);
        if(serial_msgs) {Serial.println("Rapid firing"); }
      }

      Serial.println(xAxis);
      
      // Move the mouse cursor
      if (xAxis < 128 && xAxis > 10) {
        Mouse.move(pixstep,0);
      } else if (xAxis > 128 && xAxis < 246) {
        Mouse.move(-pixstep,0);
      }
     if (yAxis < 128 && yAxis > 10) {
        Mouse.move(0,pixstep);
      } else if (yAxis > 128 && yAxis < 246) {
        Mouse.move(0,-pixstep);
      }
      
      //Mouse.move(xAxis,yAxis);
      Mouse.scroll(wheel); // vertical scroll
      /*
      if (wheel != 0) {
        if(serial_msgs) {Serial.println("Scrolling"); }

        Mouse.scroll(yAxis,0); // doesn't work cause no xAxis and yAxis values are transmitted while scrolling
        Mouse.scroll(0,xAxis);
      }
      */    
    }
  }
}
