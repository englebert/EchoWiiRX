/***
A basic receiver for the nRF24L01 module to receive 7 channels send a ppm sum
with all of them on digital pin D2.
 ***/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

////////////////////// PPM CONFIGURATION//////////////////////////
#define PPM_MAXCHANNEL 8       // Set the number of channels
#define PPM_PIN 2               // Set the PPM Signal Output pin on arduino
#define PPM_FRAME_LENGTH 27000  // Set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PULSE_LENGTH 400    // Set the PPM pulse length
#define SERIAL_PORT_SPEED 57600
//////////////////////////////////////////////////////////////////

int ppm[PPM_MAXCHANNEL];

#define DISPLAY_LED 3

/*
 * For backlight management
 */
#define BACKLIGHT_LED 4
uint16_t backlight_counter = 0;
uint8_t backlight_enable = 1;
uint8_t backlight_pattern = 0B01000010;
uint8_t backlight_pattern_pos = 0;

#define RIGHTWING_LED 5
uint16_t rightwing_light_counter = 0;
uint8_t rightwing_light_enable = 1;
uint8_t rightwing_light_pattern = 0B000101101;
uint8_t rightwing_light_pattern_pos = 0;

#define LEFTWING_LED 6
uint16_t leftwing_light_counter = 0;
uint8_t leftwing_light_enable = 1;
uint8_t leftwing_light_pattern = 0B000101101;
uint8_t leftwing_light_pattern_pos = 0;

#define BACKLIGHT_ENABLE 1
#define BACKLIGHT_DISABLE 0

/*
 * For command bits
 */
uint8_t command_received;
uint8_t command_value;
#define CMD_BACKLIGHT 0x01

const uint64_t pipeIn =  0xE8E8F0F0E1LL;  // Must be same as the transmission
RF24 radio(9, 10);                        // Starting up the module on D9, D10

// The sizeof this struct should not exceed 32 bytes
struct MyData {
    uint8_t ch1;
    uint8_t ch2;
    uint8_t ch3;
    uint8_t ch4;
    uint8_t ch5;
    uint8_t ch6;
    uint8_t ch7;
};

MyData data;

void resetData() {
    // 'safe' values to use when no radio input is detected
    data.ch1 = 0;
    data.ch2 = 0;
    data.ch3 = 0;
    data.ch4 = 0;
    data.ch5 = 0;
    data.ch6 = 0;
    data.ch7 = 0;
    
    setPPMValuesFromData();
}

uint16_t aux2, aux3, aux4;

void setPPMValuesFromData() {
    /*
     * The values for:
     * - throttle
     * - yaw
     * - pitch
     * - roll
     */
    ppm[0] = map(data.ch1, 0, 255, 1000, 2000);
    ppm[1] = map(data.ch2, 0, 255, 1000, 2000);
    ppm[2] = map(data.ch3, 0, 255, 1000, 2000);
    ppm[3] = map(data.ch4, 0, 255, 1000, 2000);

    /*
     * For Aux1, can directly tapped to ch5 since is analog signal
     */
    ppm[4] = map(data.ch5, 0, 255, 1000, 2000);

    /*
     * TODO: For Aux2, Aux3, Aux4 is in bits format. Below two lines need to redo.
     */
    /*
    RX Format:
    +--------+------+-------+----------+-----+------+-------------------------------------------------------+-----------------+
    |        |      |       |          |     |      |                           CH6                         |                 |
    |Channels| CH1  |  CH2  |    CH3   | CH4 |  CH5 +------+------+------+------+------+------+------+------+       CH7       |
    |        |      |       |          |     |      | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |                 |
    +--------+------+-------+----------+-----+------+------+------+------+------+------+------+------+------+-----------------+
    | Values | Roll | Pitch | Throttle | Yaw | Aux1 | Aux2 | Aux3 | Aux4 |          Command Bits            |  Command Values |
    +--------+------+-------+----------+-----+------+------+------+------+------+------+------+------+------+-----------------+

    Command Bits:
    0x01: Turns On/Off BackLight Blink. If CH7 value = 1 then ON, value = 0 = OFF
    */
    // Extracting data from data.ch6 for Aux2, Aux3 and Aux4
    if(((data.ch6 & B10000000) >> 7) == 1) {
        aux2 = 2000;
    } else {
        aux2 = 1000;
    }
    
    if(((data.ch6 & B01000000) >> 6) == 1) {
        aux3 = 2000;
    } else {
        aux3 = 1000;
    }

    if(((data.ch6 & B00100000) >> 5) == 1) {
        aux4 = 2000;
    } else {
        aux4 = 1000;
    }
    ppm[5] = aux2;
    ppm[6] = aux3;
    ppm[7] = aux4;

    // Handling command bits
    command_received = data.ch6 & 0B00011111;       // Removed the Bit7, Bit6 and Bit5
    command_value = data.ch7;

    // Checking command types
    if(command_received == CMD_BACKLIGHT) {
        // If enable set the value else set disable
        if(command_value == BACKLIGHT_ENABLE)
            backlight_enable = 1;
        if(command_value == BACKLIGHT_DISABLE) {
            // Disable LED
            PORTD = PORTD & ~B00010000;     // Turns OFF D4
            PORTD = PORTD & ~B00100000;     // Turns OFF D5
            PORTD = PORTD & ~B01000000;     // Turns OFF D6
            backlight_enable = 0;
        } 
    } 
}

/**************************************************/

void setupPPM() {
    pinMode(PPM_PIN, OUTPUT);
    digitalWrite(PPM_PIN, 0);       //set the PPM signal pin to the default state (off)

    cli();
    TCCR1A = 0;                     // set entire TCCR1 register to 0
    TCCR1B = 0;

    OCR1A = 100;                    // compare match register (not very important, sets the timeout for the first interrupt)
    TCCR1B |= (1 << WGM12);         // turn on CTC mode
    TCCR1B |= (1 << CS11);          // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A);        // enable timer compare interrupt
    sei();
}

void setup() { 
    // Setting Up LED Port
    pinMode(DISPLAY_LED, OUTPUT);       // D3
    pinMode(BACKLIGHT_LED, OUTPUT);     // D4
    pinMode(RIGHTWING_LED, OUTPUT);     // D5
    pinMode(LEFTWING_LED, OUTPUT);      // D6

    // Debug
    Serial.begin(9600);
    
    resetData();
    setupPPM();
    
    // Set up radio module
    radio.begin();

    radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
    radio.setAutoAck(false);

    // Added for extending the range - Englebert
    radio.setPALevel(RF24_PA_MAX);

    // Above most Wifi Channels
    // radio.setChannel(111); 
    radio.setChannel(1);

    // 8 bits CRC
    // radio.setCRCLength( RF24_CRC_8 ) ; 

    // Disable dynamic payloads 
    // radio.write_register(DYNPD,0); 

    // increase the delay between retries & # of retries 
    // radio.setRetries(15,15);
    // End of modification

    // Debugging Start
    Serial.print("PA Level:");
    Serial.println(radio.getPALevel());
    Serial.print("Data Rate:");
    Serial.println(radio.getDataRate());
    // Debugging End

    radio.openReadingPipe(1,pipeIn);
    radio.startListening();
}

/**************************************************/

unsigned long lastRecvTime = 0;

void recvData() {  
    while ( radio.available() ) {        
        radio.read(&data, sizeof(MyData));
        lastRecvTime = millis();
    }
}

/**************************************************/
int ledState = LOW;

void loop() {
    recvData();
  
    unsigned long now = millis();

    // signal lost?
    if(now - lastRecvTime > 1000) resetData();
    
    setPPMValuesFromData();
  
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
        ledState = HIGH;
        PORTD = PORTD | B00001000;      // turn pin 3 on. Could also use: digitalWrite(PPM_PIN,1)
    } else {
        ledState = LOW;
        PORTD = PORTD & ~B00001000;     // turn pin 2 off. Could also use: digitalWrite(PPM_PIN,0)
    }
}

/**************************************************/

// #error Delete this line befor you cahnge the value (clockMultiplier) below
#define clockMultiplier 2 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino

ISR(TIMER1_COMPA_vect){
    static boolean state = true;

    TCNT1 = 0;

    backlight_counter++;
    rightwing_light_counter++;
    leftwing_light_counter++;

    // Flip it
    // TODO: HERE to enable or disable
    if(backlight_enable == 1) {
        if(backlight_counter > 50) {
            backlight_counter = 0;      // Reset
    
            // Based on pattern to switch on / off the lights
            if(((backlight_pattern >> backlight_pattern_pos) & 0x01) == 1) {
                PORTD = PORTD |  B00010000;     // Turns ON D4
            } else {
                PORTD = PORTD & ~B00010000;     // Turns OFF D4
            }
            backlight_pattern_pos++;
    
            if(backlight_pattern_pos > 7) backlight_pattern_pos = 0;
        }

        // For left and right wing will be using this section for the moment. Later will shift to detecting the 
        // signal and display the pattern accordingly.
        if(rightwing_light_counter > 50) {
            rightwing_light_counter = 0;      // Reset
    
            if(((rightwing_light_pattern >> rightwing_light_pattern_pos) & 0x01) == 1) {
                PORTD = PORTD |  B00100000;     // Turns ON D5
            } else {
                PORTD = PORTD & ~B00100000;     // Turns OFF D5
            }
            rightwing_light_pattern_pos++;
   
            if(rightwing_light_pattern_pos > 7) rightwing_light_pattern_pos = 0;
        }
        if(leftwing_light_counter > 50) {
            leftwing_light_counter = 0;      // Reset
    
            if(((leftwing_light_pattern >> leftwing_light_pattern_pos) & 0x01) == 1) {
                PORTD = PORTD |  B01000000;     // Turns ON D6
            } else {
                PORTD = PORTD & ~B01000000;     // Turns OFF D6
            }
            leftwing_light_pattern_pos++;
   
            if(leftwing_light_pattern_pos > 7) leftwing_light_pattern_pos = 0;
        }
    
    }

    if(state) {
        //end pulse
        PORTD = PORTD & ~B00000100; // turn pin 2 off. Could also use: digitalWrite(PPM_PIN,0)
        OCR1A = PPM_PULSE_LENGTH * clockMultiplier;
        state = false;
    } else {
        //start pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        PORTD = PORTD | B00000100; // turn pin 2 on. Could also use: digitalWrite(PPM_PIN,1)
        state = true;

        if(cur_chan_numb >= PPM_MAXCHANNEL) {
            cur_chan_numb = 0;
            calc_rest += PPM_PULSE_LENGTH;
            OCR1A = (PPM_FRAME_LENGTH - calc_rest) * clockMultiplier;
            calc_rest = 0;
        } else {
            OCR1A = (ppm[cur_chan_numb] - PPM_PULSE_LENGTH) * clockMultiplier;
            calc_rest += ppm[cur_chan_numb];
            cur_chan_numb++;
        }     
    }
}
