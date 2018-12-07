/***
A basic receiver for the nRF24L01 module to receive 7 channels send a ppm sum
with all of them on digital pin D2.
 ***/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

////////////////////// PPM CONFIGURATION//////////////////////////
#define PPM_MAXCHANNEL 8        // Set the number of channels
#define PPM_PIN 2               // Set the PPM Signal Output pin on arduino
#define PPM_FRAME_LENGTH 27000  // Set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PULSE_LENGTH 400    // Set the PPM pulse length
#define SERIAL_PORT_SPEED 57600
//////////////////////////////////////////////////////////////////

int ppm[PPM_MAXCHANNEL];
#define DISPLAY_LED 3

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
    uint8_t ch8;
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
    data.ch8 = 0;
    /*
    data.throttle = 0;
    data.yaw = 127;
    data.pitch = 127;
    data.roll = 127;
    data.AUX1 = 0;
    data.AUX2= 0;
    data.AUX3 = 0;
    */
    
    setPPMValuesFromData();
}

void setPPMValuesFromData() {
    /*
    ppm[0] = map(data.throttle, 0, 255, 1000, 2000);
    ppm[1] = map(data.yaw,      0, 255, 1000, 2000);
    ppm[2] = map(data.pitch,    0, 255, 1000, 2000);
    ppm[3] = map(data.roll,     0, 255, 1000, 2000);
    ppm[4] = map(data.AUX1,     0, 1, 1000, 2000);
    ppm[5] = map(data.AUX2,     0, 1, 1000, 2000);
    ppm[6] = map(data.AUX3,     0, 1, 1000, 2000);  
    
    ppm[0] = data.ch1;
    ppm[1] = data.ch2;
    ppm[2] = data.ch3;
    ppm[3] = data.ch4;
    ppm[4] = data.ch5;
    ppm[5] = data.ch6;
    ppm[6] = data.ch7;
    */
    ppm[0] = map(data.ch1, 0, 255, 1000, 2000);
    ppm[1] = map(data.ch2, 0, 255, 1000, 2000);
    ppm[2] = map(data.ch3, 0, 255, 1000, 2000);
    ppm[3] = map(data.ch4, 0, 255, 1000, 2000);
    ppm[4] = map(data.ch5, 0, 255, 1000, 2000);
    ppm[5] = map(data.ch6, 0, 255, 1000, 2000);
    ppm[6] = map(data.ch7, 0, 255, 1000, 2000);
    ppm[7] = map(data.ch8, 0, 255, 1000, 2000);
    /*
    ppm[0] = data.ch1;
    ppm[1] = data.ch2;
    ppm[2] = data.ch3;
    ppm[3] = data.ch4;
    ppm[4] = data.ch5;
    ppm[5] = data.ch6;
    ppm[6] = data.ch7;
    ppm[7] = data.ch8;
      */
 
//  Serial.print("1:" + String(data.ch1) + " ");
//  Serial.print("2:" + String(data.ch2) + " ");
//  Serial.print("3:" + String(data.ch3) + " ");
//  Serial.print("4:" + String(data.ch4) + " ");
//  Serial.print("5:" + String(data.ch5) + " ");
//  Serial.print("6:" + String(data.ch6) + " ");
//  Serial.print("7:" + String(data.ch7) + " ");
//  Serial.print("8:" + String(data.ch8) + " ");
//  Serial.print("1:" + String(ppm[0]) + " ");
//  Serial.print("2:" + String(ppm[1]) + " ");
//  Serial.print("3:" + String(ppm[2]) + " ");
//  Serial.print("4:" + String(ppm[3]) + " ");
//
//  Serial.println();
}

/**************************************************/

void setupPPM() {
    pinMode(PPM_PIN, OUTPUT);
    digitalWrite(PPM_PIN, 0);  //set the PPM signal pin to the default state (off)

    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;

    OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();
}

void setup() { 
    // Setting Up LED Port
    pinMode(DISPLAY_LED, OUTPUT);
    
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
    // Serial.print("PA Level:");
    // Serial.println(radio.getPALevel());
    // Serial.print("Data Rate:");
    // Serial.println(radio.getDataRate());
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
    if ( now - lastRecvTime > 1000 ) {
        // signal lost?
        resetData();
    }
    
    setPPMValuesFromData();
  
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
        ledState = HIGH;
        PORTD = PORTD | B00001000; // turn pin 3 on. Could also use: digitalWrite(PPM_PIN,1)
    } else {
        ledState = LOW;
        PORTD = PORTD & ~B00001000; // turn pin 2 off. Could also use: digitalWrite(PPM_PIN,0)
    }
  
    // set the LED with the ledState of the variable:
    // TODO: Use another way to speed this up
    // digitalWrite(DISPLAY_LED, ledState);
}

/**************************************************/

// #error Delete this line befor you cahnge the value (clockMultiplier) below
#define clockMultiplier 2 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino

ISR(TIMER1_COMPA_vect){
    static boolean state = true;

    TCNT1 = 0;

    if ( state ) {
        //end pulse
        PORTD = PORTD & ~B00000100; // turn pin 2 off. Could also use: digitalWrite(PPM_PIN,0)
        OCR1A = PPM_PULSE_LENGTH * clockMultiplier;
        state = false;
    }
    else {
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
