/* armband.ino is for upload to the Arduino on the armband of the Glove Mouse. 

   It reads sensor data from the glove and sends packets with cursor and click information to the 
   transmitting transceiver.
   
   Currently it uses the SoftwareSerial library for serial communication with the transceiver, but this
   will be changed to a hardware serial when Arduino Mega is used instead of Arduino Uno. 
*/

// Debugging parameters for serial communication
bool tx_enable = true; // true to send packets to transceiver 
bool print_packet = false; // true to print packets to serial monitor for debugging
bool print_mouse_actions = false; // true to print click messages to serial monitor

//----------------------------------------------------------------------------------------------------//
//                                         PIN ASSIGNMENTS                                            //
//----------------------------------------------------------------------------------------------------//

/* Accelerometer analog pin assignments. 
   The accelerometer outputs three analog voltages proportional to the tilt on three axes.
   On the Arduino Uno, pins A0-A5 are analog input.
 */
#define x_accel_pin A0
#define y_accel_pin A1
#define z_accel_pin A2

/* Flex sensor analog pin assignments. 
   Flex sensors are variable resistors where the resistance is proportional to the amount of bend.
   There are three remaingin analog pins available after the accelerometer. 
   The flex sensors are each hooked up as R1 in a voltage divider.
 */
#define pointer_flex_pin A4               // left click         (0x01 from flexRead())
#define middle_flex_pin A3                // right click        (0x02 from flexRead())
 
/* Contact pad digital pin assignments.
   For Arduino Uno, pins 2 - 13 are digital I/O pins. Pins 8-13 are in PINB register.
   The contact pads function like buttons. 
   In the setup() function, the internal pullup resistors are enabled, so that the contact pads are 
   set to LOW when contact is not made and HIGH when they are in contact. 
 */
#define inter_finger_pad_pin 13           // middle click       (0x20 on PINB)
#define pointer_pad_pin 12                // move enable        (0x10 on PINB)
#define middle_pad_pin 11                 // scroll enable      (0x08 on PINB)
#define thumb_pad_pin 10                  // rapid fire enable  (0x04 on PINB)

// Define some pin number macros to make modifying later code easier
#define PIN8 0x01       // Pins for the PINB register. Each pin is a bit in a binary byte (8 bits).
#define PIN9 0x02       // For example, 00000010 is 0x02 in hexadecimal and 2 in decimal.
#define PIN10 0x04      // 00000100
#define PIN11 0x08      // 00001000
#define PIN12 0x10      // 00010000
#define PIN13 0x20      // 00100000 The last two bits in this register are for something else. 

//----------------------------------------------------------------------------------------------------//
//                                     SERIAL COMMUNICATION SETUP                                     //
//----------------------------------------------------------------------------------------------------//

#include <SoftwareSerial.h> // Library that turns 2 GPIO ports into RX/TX ports. Low baud capabilities.

// Transmission pin alias
#define TX_PIN 4
#define RX_PIN 3

SoftwareSerial HumPROSerial(RX_PIN,TX_PIN); // Create a Software Serial port on GPIO pins

#define BAUDRATE 19200  // Choose the baud rate. SoftwareSerial works for only 9600 and 19200 currently. 
                        // Supposedly it should go higher, but I can't get anything above 19200.

// used by the base station code to determine the start of a packet
#define SYNC 0xAA

// HumPRO transceivers have unique 4-byte DSN but, in Extended user addressing mode, HumPRO 
// transceivers default to 0xFF address
#define ADDR 0xFF

//----------------------------------------------------------------------------------------------------//
//                                         DEBOUNCING DEFINITIONS                                     //
//----------------------------------------------------------------------------------------------------//

#define timeout 30      // Timeout value for the state machine
  
#define NoPush      1
#define MaybePush   2
#define Pushed      3   
#define MaybeNoPush 4   // ie MaybeRelease

//----------------------------------------------------------------------------------------------------//
//                                         DEBOUNCING PARAMS                                          //
//----------------------------------------------------------------------------------------------------//

volatile unsigned char tasktime;  // Timeout counter

unsigned char PushState;          // State machine for contact pads
unsigned char PushState_flex;     // State machine for flex sensors

unsigned char butnumPressed;      // Stores which contact pad was last pressed 
unsigned char flexnumPressed;     // Stores which flex sensor was last flexed 

unsigned char currentButton;      // Stores which contact pad is currently pressed 
unsigned char currentFlex;        // Stores whcih flex sensor is currently flexed

// Debouncing subroutine called in main loop
void task1(void);

//----------------------------------------------------------------------------------------------------//
//                                         SENSOR MESSAGES                                            //
//----------------------------------------------------------------------------------------------------//
 
#define left_click            0x80 
#define left_click_release    0x01

#define middle_click          0x40
#define middle_click_release  0x02

#define right_click           0x20
#define right_click_release   0x04

#define rapid_fire            0x08

unsigned char msg_inter_finger_pad;        // Middle click
unsigned char msg_inter_finger_pad_release;

unsigned char msg_pointer_flex;           // Left click
unsigned char msg_pointer_flex_release;

unsigned char msg_middle_flex;            // Right click
unsigned char msg_middle_flex_release;

//----------------------------------------------------------------------------------------------------//
//                                         ACCELEROMETER PARAMS                                       //
//----------------------------------------------------------------------------------------------------//
  
#define accel_timeout 2                   // for state machine

char move_en;                             // Flag for movement enable

volatile unsigned char aX;                // Samples analog pins
volatile unsigned char aY;
volatile unsigned char aZ;

// Calculated in the calibration function each time move is enabled
volatile signed long aX_ref;              // Offset calculated from position on move enable
volatile signed long aY_ref;
volatile signed long aZ_ref;

volatile signed int accel_x;              // Calibrated values for axis tilt in XY plane
volatile signed int accel_y;

volatile signed int accel_x_prev;         // Stores previous axis tilt
volatile signed int accel_y_prev;

volatile signed int scale_x;              // Scale for cursor movement
volatile signed int scale_y;

volatile signed int move_x;               // Final XY movement to transmit to computer
volatile signed int move_y;

#define thresh 30                    /*  Threshold below which accelerometer changes are disregarded 
                                        so as to remove noise from hand shaking. Could be cool to 
                                        calibrate this for each new user upon start up.             */

volatile unsigned char acceltime;   /*  Used in ISR to alternate between calculating and sending
                                        data. timeout is accel_timeout = 2 from above.              */
                                        
// Enable scrolling. Takes precedence over cursor movement. 
char scroll_en;

// Time count for scrolling
// Goes from 0 to 90. Multiply by the 8ms increment of the ISR, and you get 
// a scroll message sent every 720 ms
volatile unsigned char scrolltime; 

//----------------------------------------------------------------------------------------------------//
//                                             MODE FLAGS                                             //
//----------------------------------------------------------------------------------------------------//

char rapid_fire_en;   // Flag for rapid fire mode enable. Toggled by contact pad

char send_rapid_fire; // Flag for transmitting rapid fire

signed int invert_x;  // Flag for invert_x enable
signed int invert_y;  // Flag for invert_y enable

char accel_mode;      // handshake (1) vs flat (0) orientation

// Cursor sensitivity flag
unsigned char sensitivity;

//----------------------------------------------------------------------------------------------------//
//                                         UART COMMUNICATION                                         //
//----------------------------------------------------------------------------------------------------//

/* tx_packet function packs a 6 byte packet, slaps a stamp on it and sends it to the serial port
   
   @param x the x-position of the mouse
   @param y the y-position of the mouse
   @param _scroll the scroll position of the mouse
   @param button_press the contact pad/ flex sensor readings
*/
void tx_packet(uint8_t x, uint8_t y, uint8_t _scroll, uint8_t button_press) {

  if (tx_enable) { // send to serial port
    HumPROSerial.write(SYNC);         // check the time (SYNC)
    HumPROSerial.write(ADDR);         // slap a stamp on it (receiver address)
    HumPROSerial.write(x);            // pack mouse x-position
    HumPROSerial.write(y);            // pack mouse y-position 
    HumPROSerial.write(_scroll);       // pack scroll position
    HumPROSerial.write(button_press); // pack press/flex info    
    
  } else if (print_packet) { // print to the serial monitor for debugging
    Serial.print(SYNC);         // check the time (SYNC)
    Serial.print(ADDR);         // slap a stamp on it (receiver address)
    Serial.print(x);            // pack mouse x-position
    Serial.print(y);            // pack mouse y-position 
    Serial.print(_scroll);       // pack scroll position
    Serial.print(button_press); // pack press/flex info 
    Serial.println();
  }
}

//----------------------------------------------------------------------------------------------------//
//                                        SENSOR READING FUNCTIONS                                    //
//----------------------------------------------------------------------------------------------------//

/* readAccel function reads the inputs from the accelerometer, storing the appropriate values based  
   on the value of accelmode (0 for flat hand mode, 1 for handshake mode).
*/ 
void readAccel() {
  if (accel_mode == 1 ) { // if in handshake mode, use Z and Y components
    aX = analogRead(z_accel_pin); // use z-axis accelerometer output as x-axis for mouse
    aY = analogRead(y_accel_pin); // use y-axis accelerometer output as y-axis for mouse

  } else { // else use flat mode, use Y and X components
    aX = analogRead(x_accel_pin); // use x for x
    aY = analogRead(y_accel_pin); // use y for y
  }
}

/* readFlex function works like PINx register for analog inputs
     
   @return FLEX the unsigned char corresponding to the bent flex sensor.  
*/
unsigned char readFlex() {
  
  unsigned char FLEXread = 0x00;
  
  uint16_t pointerflex = analogRead(pointer_flex_pin);
  uint16_t middleflex = analogRead(middle_flex_pin);

  /*
  Serial.print("pointerflex value: ");
  Serial.print(pointerflex);
  Serial.println();
  
  Serial.print("middleflex value: ");
  Serial.print(middleflex);
  Serial.println();  
  */
  
  // flex sensors are R1 in voltage divider with 10k resistors. Values range from ~400 down
  if (middleflex < 280 && middleflex > 150) { // middle finger sensor over thumb sensor
    FLEXread = 0x02;
  }
  if (pointerflex < 280 && pointerflex > 150) { // left_click takes priority
    FLEXread = 0x01;
 
  }

  return FLEXread;
}

/* calibrate function tares the orientation of the accelerometer (modified from glove.c calibrate 
   function). As in the code from Hyodong, samples 1024 values of the acceleration and averages.
   Current issue: not sure about values from analogRead()
   Hyodong only took the high byte, so not sure if that makes a difference. 
 */
void calibrate(void) {

  // Take 1024 samples of X,Y,Z tilt and sum the values for each axis
  for (int i = 0; i++; i < 1024) {
    aX = analogRead(x_accel_pin);
    aY = analogRead(y_accel_pin);
    aZ = analogRead(z_accel_pin);

    // sum
    aX_ref = aX_ref + aX;
    aY_ref = aY_ref + aY;
    aZ_ref = aZ_ref + aZ;
  }
  
  // take the average by dividing by the number of samples
  aX_ref = aX_ref/1024;
  aY_ref = aY_ref/1024;
  aZ_ref = aZ_ref/1024;   

          // or do what Hyodong did bitwise, which I do not understand

  /* aX_ref = aX_ref >> 10;      // Is this averaging the sums from the while loop?
  aY_ref = aY_ref >> 10;
  aZ_ref = aZ_ref >> 10; */

  /* Optionally 2 different modes for hand orientation (acc z vs y or acc y vs x)
   */
  // Determine hand position (handshake orientation vs palm down orientation)
  // Will need to adjust this experimentally for particular accelerometer
  if ((aZ_ref > 60) && (aZ_ref < 100)) { //hand is in handshake orientation
    accel_mode = 1; // put it in handshake mode  
    aX_ref = aZ_ref; // switch X and Z reference values
    
  } else {            // else flat hand mode
    accel_mode  = 0;  // put it in flat hand mode
  }

  // use reference orientation to set initial cursor acceleration parameters
  accel_x_prev = aX_ref; // (which could be the value from aZ_ref if in handshake mode)
  accel_y_prev = aY_ref;
}

//----------------------------------------------------------------------------------------------------//
//                                       TIMER INTERRUPT DEFINITION                                   //
//----------------------------------------------------------------------------------------------------//

/*  Timer 0 compare ISR.
    Interrupt occurs every 4 ms. 
    The ADC sample is read and the tilt is computed as cursor movement or scroll, then transmitted 
    every 8 ms.
    If in rapid fire mode, transmit a rapid fire packet every 30 ms. 
 */
ISR (TIMER0_COMPA_vect) {
  
  // Decrement the task time counts if they are not already zero
  if (tasktime > 0) --tasktime;
  if (acceltime > 0) --acceltime;

  // Every 8 ms, read ADC value from accelerometer and compute the change of the cursor position
  if (acceltime == 0) {
    acceltime = accel_timeout;    // Reset timeout counter to 2
    readAccel();                  // Read ADC values from accelerometer

    // Eliminate the offset by subtracting the zero reference
    if (accel_mode == 1) { // Handshake mode
      accel_x = aX_ref - aX;
      
    } else { // Flat mode
      accel_x = aX - aX_ref; 
    }
    accel_y = aY - aY_ref;

    // Set up threshold to prevent drift
    /* (Use thresh to discard small movements) */
    if ((-thresh <= accel_x) && (accel_x <= thresh)) {
      accel_x = 0;
    }
    if ((-thresh <= accel_y) && (accel_y <= thresh)) {
      accel_y = 0;
    }

    // Compute scale parameters based on the cursor acceleration and position inversion
    if (accel_x > accel_x_prev) {
      scale_x = invert_x * (accel_x - accel_x_prev + 8) >> 3;
    } else {
      scale_x = invert_x * (accel_x_prev - accel_x + 8) >> 3;
    }
    if (accel_y > accel_y_prev) {
      scale_y = invert_y * (accel_y - accel_y_prev + 8) >> 3;
    }
    else {
      scale_y = invert_y * (accel_y_prev - accel_y + 8) >> 3;
    }

    // Store the tilt value for computing cursor acceleration
    accel_x_prev = accel_x;
    accel_y_prev = accel_y;

    // Compute change of cursor position to be sent to base station, based on scale and sensitivity
    move_x = (accel_x >> sensitivity) * scale_x; // sensitivity == 3 or 4 in OG. For me 0 works, 3 or 4 breaks mouse
    move_y = (accel_y >> (sensitivity + scroll_en)) * scale_y;
  } 
  // Every other 8 ms, transmit the cursor position or scroll message
  else if (acceltime == 1) {
    
    // If scroll is enabled, transmit the scroll message every 720 ms
    if (scroll_en == 1) {
      scrolltime++; // Increase the time count for scroll message transmission
      //if (move_y > (35 >> (sensitivity + 1)) * scale_y) tx_packet(0x00, 0x00, -0x01, 0x00);
      //else if (move_y < - ((35 >> (sensitivity + 1)) * scale_y)) tx_packet(0x00, 0x00, 0x01, 0x00);
    
      if (scrolltime == 90) {
        scrolltime = 0;
        if (move_y > 128) {
          tx_packet(xAxis, yAxis, 0x01, 0x00);
          Serial.println(move_y);
        } else if (move_y < 128) {
          Serial.println(move_y);
          tx_packet(xAxis, yAxis, 0xFE, 0x00);
        }
      }
    }
    // If move is enabled and scroll is disabled, transmit the cursor position message    
    else if (move_en == 1) {
      tx_packet(move_x, move_y, 0x00, 0x00);
    }
  }
  
  // If in rapid fire mode
  if (send_rapid_fire == 1) {
    // Transmit left button click message every 30 ms
    if (tasktime % 15 == 2)  {
      tx_packet(0x00, 0x00, 0x00, rapid_fire);
    }
  }
}

//----------------------------------------------------------------------------------------------------//
//                                         ARDUINO SETUP LOOP                                         //
//----------------------------------------------------------------------------------------------------//

/* Executed once when code is run.
   Sets the pins for the sensors
   Initializes various flags/modes
   Sets up ISR 
   Starts UART serial communication
*/
void setup() {

  // set HumPROSerial pins
  pinMode(3, INPUT);
  pinMode(4, OUTPUT);

  // pull the CMD line low to place HumPRO in command mode (pin 2 on arduino)
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  // initialize the serial communications (computer-USB, Arduino-HumPRO)
  Serial.begin(9600);
  HumPROSerial.begin(9600);
  
  // set the addressing mode to 0x07 (Extended User Addressing Mode)
  // ADDMODE register
  // 0xFF     0x02    0x04              0x07
  // header   size    non-vol address   value
  // 0x4F volatile addr
  HumPROSerial.write(0xFF);
  HumPROSerial.write(0x02);
  HumPROSerial.write(0x4F);
  HumPROSerial.write(0x07);
/*
  // check that the ADDMODE has been set (desired output: 64F7)
  HumPROSerial.write(0xFF);
  HumPROSerial.write(0x02);
  HumPROSerial.write(0xFE);
  HumPROSerial.write(0x4F);
*/  
  // both the source and dest addresses are set to 0xFF by default 
  // Default settings work for demo purposes but should be changed later

  // change the data rate with the UARTBAUD register (initially is 9600)
  // 0xFF   0x02  0x03                  0x04
  // header size  non-volatile address  value
  // value is 0x04 for a baud rate of 57600 
  // volatile address is 0x4E
  // power must be cycled for non-volatile changes
  HumPROSerial.write(0xFF); // header
  HumPROSerial.write(0x02); // size
  HumPROSerial.write(0x4E); // register address (UARTBAUD)
  HumPROSerial.write(0x02); // set baud rate to 57600

  // reconfigure the Arduino baud rates to match HumPRO internal baud rate
  Serial.flush();
  Serial.begin(BAUDRATE);
  HumPROSerial.flush();
  HumPROSerial.begin(BAUDRATE);
  while(Serial.available() > 0) Serial.read(); // clear bytes corrupted during the change
  while(HumPROSerial.available() > 0) HumPROSerial.read();

/*
  // check the new baud rate on HumPRO 
  HumPROSerial.write(0xFF);
  HumPROSerial.write(0x02);
  HumPROSerial.write(0xFE);
  HumPROSerial.write(0x4E);
  */

  digitalWrite(2, HIGH);   // pull CMD line to HIGH to enter transmit mode

  // Pin modes
  analogReference(EXTERNAL);

  // Accelerometer pinModes set to INPUT
  pinMode(x_accel_pin, INPUT);
  pinMode(y_accel_pin, INPUT);
  pinMode(z_accel_pin, INPUT);
  
  // Flex sensor pinModes set to INPUT
  pinMode(pointer_flex_pin, INPUT);
  pinMode(middle_flex_pin, INPUT);
  //pinMode(thumb_flex_pin, INPUT);
  
  // Contact pad pinModes set to INPUT with internal pullup resistors
  pinMode(pointer_pad_pin, INPUT_PULLUP);
  pinMode(middle_pad_pin, INPUT_PULLUP);
  pinMode(inter_finger_pad_pin, INPUT_PULLUP);
  pinMode(thumb_pad_pin, INPUT_PULLUP);

  //og code set up button related parameters
  currentButton = 0x00;
  butnumPressed = 0x00;
  currentFlex = 0x00;
  flexnumPressed = 0x00;
  PushState = NoPush;
  PushState_flex = NoPush;
  tasktime = timeout;                       // Counter for debouncing subroutine task1(). timeout = 30

  //og code set message chars
  msg_pointer_flex = left_click;                    // 0x80
  msg_middle_flex = right_click;                    // 0x20
  //msg_thumb_flex = ;                              // Currently no flex sensor on thumb

  msg_pointer_flex_release = left_click_release;    // 0x01
  msg_middle_flex_release = right_click_release;    // 0x04
  //msg_thumb_flex_release = ;                      // Currently, no flex sensor on thumb

  //more char messages for contact pads
  msg_inter_finger_pad = middle_click;                  // 0x04
  msg_inter_finger_pad_release = middle_click_release;  // 0x02

  scrolltime = 0;

  TIMSK0 = (1 << OCIE0A);               // turn on the timer0 compare match ISR
  OCR0A = 249;                          // set the compare register. 
  TCCR0B = TCCR0B & 0b11111000 | 0b100; // set prescalar to 256. ISR will execute every 4ms.
  TCNT0 = 0; 
  TCCR0A = 0;                           // turn on clear-on_match

  // Initialize acceleration related parameters
  acceltime = 0;
  aX = 0;
  aY = 0;
  aZ = 0;
  accel_x = 0;
  accel_y = 0;
  scale_x = 0;
  scale_y = 0;
  sensitivity = 0;
  Serial.print("sensitivity is: ");
  Serial.print(sensitivity);
  Serial.println();
  
  // Initialize mode flags and parameters
  move_en = 1;
  scroll_en = 0;
  rapid_fire_en = 0;
  send_rapid_fire = 0;
  invert_x = 1;
  invert_y = 1;
  accel_mode = 0; // (0 = flat hand mode, 1 = handshake mode)

  sei();   //Enable interrupt
}

//----------------------------------------------------------------------------------------------------//
//                                          ARDUINO MAIN LOOP                                         //
//----------------------------------------------------------------------------------------------------//

/*  Arduino loop() function calls the debouncing subroutine task1() repeatedly  
*/
void loop() {

  // Call the debouncing subroutine
  if (tasktime == 0) {
    task1();
  }
}

//----------------------------------------------------------------------------------------------------//
//                                          DEBOUNCING SUBROUTINE                                     //
//----------------------------------------------------------------------------------------------------//

 /* State machine for contact pad presses and flex sensors bending
    Serves to prevent button bounce
  */
void task1(void) {

  tasktime = timeout;             // Reset the task timer (timeout = 30)
  currentFlex = readFlex();       // Read currently flexed sensors
  currentButton = ~(PINA-61);     // Read currently pressed buttons (negated bc buttons are pulled high
                                  // by internal pullup resistors)
  /*                                
  Serial.print("The currentButton value is: ");
  Serial.print(currentButton);   
  Serial.println();
  */
  /*
  Serial.print("The currentFlex value is: ");
  Serial.print(currentFlex);   
  Serial.println();
  */
                   
// State machine for contact pads --------------------------------------------------------------------//
  switch(PushState) {
    
    case NoPush:
      // If any of the buttons on PINB are on, user might be pressing it
      // In og code inputs are pulled high, mine are pulled low
      if (currentButton != 0x00) {
        PushState = MaybePush;
        butnumPressed = currentButton;
      }
      break;
      
    case MaybePush: 
      if (butnumPressed == currentButton) { // If same button still pressed, it must be a valid press

        PushState = Pushed;

        if (currentButton == PIN13) { // else if (INTER FINGER PAD) _____ middle click

          cli();
          tx_packet(0x00,0x00,0x00, msg_inter_finger_pad);
          sei();
        }
        else if (currentButton == PIN10) { // If pin 11 pressed (POINTER PAD) __________ move_en
          move_en ^= 1; // Toggle move_en with bitwise XOR assignment (inverter)
          if (move_en == 1) {
            cli();
            calibrate();
            sei();
            Serial.println("Movement enabled");
          } else {
            Serial.println("Movement disabled");
          }
        } else if (currentButton == PIN12) { // else if (MIDDLE PAD) ____ scroll_en
          scroll_en = 1;
          if (scroll_en == 1) {
            cli();
            calibrate();
            sei();
          }
          Serial.println("Scroll on");
     
        } else if (currentButton == PIN11) { // else if (THUMB PAD) _____ rapid_fire_en
          rapid_fire_en ^= 1;
          if (rapid_fire_en == 1) {
            Serial.println("Rapid fire enabled");
          } else {
            Serial.println("Rapid fire disabled");
          }
        }
      } else { // button is not still pressed, false alarm
        PushState == NoPush;
      }
      break; 
      
    case Pushed:
      // Button aleady down, if release is detected
      if (butnumPressed != currentButton) {
        PushState = MaybeNoPush;
      }
      break;

    case MaybeNoPush:
      // If the button is not pressed, maybe the user did it
      if (butnumPressed == currentButton) {
        PushState == Pushed;
        
      } else { // else it is a valid release
        PushState = NoPush;

        // If pin 10 released (INTER FINGER PAD RELEASE) ____ middle click release
        if (butnumPressed == PIN13) {
          cli();
          tx_packet(0x00,0x00,0x00, msg_inter_finger_pad_release); //middle_click_release
          sei();
        }
        // If pin 11 released (POINTER PAD RELEASE) ____ move_en
        if (butnumPressed == PIN10) {
          // Currently move_en is toggled with a press rather than held down
        }
        // If pin 12 released (MIDDLE PAD RELEASE) ____ scroll_en
        else if (butnumPressed == PIN12) {
          scroll_en = 0;
          Serial.println("Scroll off");
        }
        // If pin 13 released (THUMB PAD RELEASE)
        else if (butnumPressed == PIN11) {
          // currently rapid_fire_en is toggled with press rather than held down
        } 
         butnumPressed = 0x00; // reset the button press 
      }
      break;
  }

 // State machine for flex sensors -------------------------------------------------------------------//
  switch(PushState_flex) {
    
    case NoPush:
      // If readFlex() function returns anything besides 0x00, might be a valid flex
      if (currentFlex != 0x00) {
        PushState_flex = MaybePush;
        flexnumPressed = currentFlex; // store which sensor was flexed
      }
      break;
      
    case MaybePush: 
      // If same flex sensor is still bent, it must be a valid flex
      if (flexnumPressed == currentFlex) {
        PushState_flex = Pushed; 
        
        // If pointer flex sensor is bent (POINTER FLEX)
        if (currentFlex == 0x01) {
          
          cli(); // clear interrupts

          if (rapid_fire_en == 1) {
            send_rapid_fire = 1; // Toggle send_rapid_fire, causing ISR to transmit rapid fire clicks
          } else {
            tx_packet(0x00,0x00,0x00, msg_pointer_flex); // Send left click message
            Serial.print("Pointer flex: ");
            Serial.print(currentFlex);
          }
          sei(); // resume interrupt
          
        // else if middle flex sensor is bent (MIDDLE FLEX)
        } else if (currentFlex == 0x02) {
          cli();
          tx_packet(0x00,0x00,0x00, msg_middle_flex); // Send right click message
          Serial.print("Middle flex: ");
          Serial.print(currentFlex);
          sei();
          
        }
      } else { // button is not still pressed, false alarm
        PushState_flex == NoPush;
      }
      break;
 
    case Pushed:
      // Button aleady down, if release is detected
      if (flexnumPressed != currentFlex) {
        PushState_flex = MaybeNoPush;
        currentFlex = currentFlex;
      }
      break;

    case MaybeNoPush:
      //If the button is actually still pressed, it was a false alarm
      if (flexnumPressed == currentFlex) { // if the last push is the same as the current
        PushState_flex == Pushed; // keep it pushed
        
      } else { // else it is a valid release
        PushState_flex = NoPush;

        // If pin A3 released (POINTER FLEX RELEASE)
        if (flexnumPressed == 0x01) {
          cli();
          tx_packet(0x00,0x00,0x00, msg_pointer_flex_release); //left_click_release
          sei();
          send_rapid_fire = 0;  
        }
        // If pin A4 released (MIDDLE FLEX RELEASE)
        else if (flexnumPressed == 0x02) {
          cli();
          tx_packet(0x00,0x00,0x00, msg_middle_flex_release); // right_click_release
          sei(); 
        }
        // If pin A5 released (THUMB FLEX RELEASE)
        else if (flexnumPressed == 0x04) {
          // Currently, no flex sensor on thumb
        }
         butnumPressed = 0x00; // reset the button press 
      }
      break;
  }       
}
