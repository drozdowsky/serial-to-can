#include <Serial_CAN_Module.h>
#include <SoftwareSerial.h>
#include <TimerOne.h>

#define CAN_TX  2           // tx of serial can module connect to (D2)
#define CAN_RX  3           // rx of serial can module connect to (D3)

#define RECEIVE_CAN_ID 0x7DF  // address on which CAN message come from (requests)
#define RESPOND_CAN_ID 0x7E8  // address on which CAN messages ECU should respond to (response)

#define NOTHING_RECEIVED        0
#define R_MESSAGE               1
#define N_MESSAGE               2

#define SERIAL_UPDATE_RATE 10   // Serial read refresh from Speeduino
#define CAN_PACKET_SIZE    123  // Number of data that comes from 'n' Speeduino Command

// #define DEBUG 1

static uint32_t oldtime=millis();   // for the timeout
uint8_t SpeedyResponse[CAN_PACKET_SIZE]; //The data buffer for the serial3 data. This is longer than needed, just in case
bool doRequest; // when true, it's ok to request more data from speeduino serial
uint8_t SerialState;

Serial_CAN CAN1;

void setup(){
    Serial.begin(115200); // speeduino baudrate

    doRequest = false;
    CAN1.begin(CAN_TX, CAN_RX, 9600);

    // Start with sensible values for some of these variables.
    SerialState = NOTHING_RECEIVED;

    // setup timer
    Timer1.initialize(10000);
    Timer1.attachInterrupt(requestData);

    doRequest = true; // all set. Start requesting data from speeduino
}

void requestData() {
    if (doRequest){
        Serial.write("n"); // Send n to request real time data
        doRequest = false;
    }
}

void handleCanMessage() {
    unsigned long id = 0;
    unsigned char buf[8];
    while (CAN1.recv(&id, buf)) {
        if (id == RECEIVE_CAN_ID && buf[1] == 0x01) {
            sendCanMessage(buf[2]);
        }
    }
}

void sendCanMessage(unsigned char requestedPIDlow){
    unsigned char buf[8];
    uint16_t obdcalcA, obdcalcB, obdcalcC, obdcalcD;
    uint32_t obdcalcE32, obdcalcF32;
    uint16_t obdcalcG16, obdcalcH16;

    switch (requestedPIDlow)
    {
        case 0:       //PID-0x00 PIDs supported 01-20  
            buf[0] =  0x06;    // sending 6 bytes
            buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x00;    // PID code
            buf[3] =  B00001000;   //1-8
            buf[4] =  B01111110;   //9-16
            buf[5] =  B10100000;   //17-24
            buf[6] =  B00010001;   //17-32
            buf[7] =  B00000000;   
            break;

        case 5:      //PID-0x05 Engine coolant temperature , range is -40 to 215 deg C , formula == A-40
            buf[0] =  0x03;                 // sending 3 bytes
            buf[1] =  0x41;                 // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x05;                 // pid code
            buf[3] =  SpeedyResponse[7];    // A
            buf[4] =  0x00;                 //the data value B which is 0 as unused
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 10:        // PID-0x0A , Fuel Pressure (Gauge) , range is 0 to 765 kPa , formula == A / 3)
            buf[0] =  0x03;    // sending 3 byte
            buf[1] =  0x41;    // 
            buf[2] =  0x0A;    // pid code
            buf[3] =  SpeedyResponse[103];
            buf[4] =  0x00;
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 11:        // PID-0x0B , MAP , range is 0 to 255 kPa , Formula == A
            buf[0] =  0x03;    // sending 3 byte
            buf[1] =  0x41;    // 
            buf[2] =  0x0B;    // pid code
            buf[3] =  SpeedyResponse[4];
            buf[4] =  0x00;
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 12:        // PID-0x0C , RPM  , range is 0 to 16383.75 rpm , Formula == 256A+B / 4
            buf[0] = 0x04;                        // sending 4 byte
            buf[1] = 0x41;                        // 
            buf[2] = 0x0C;                        // pid code
            buf[3] = SpeedyResponse[15];
            buf[4] = SpeedyResponse[14];
            buf[5] = 0x00; 
            buf[6] = 0x00; 
            buf[7] = 0x00;
            break;

        case 13:        //PID-0x0D , Vehicle speed , range is 0 to 255 km/h , formula == A 
            buf[0] =  0x03;                       // sending 3 bytes
            buf[1] =  0x41;                       // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x0D;                       // pid code
            buf[3] =  SpeedyResponse[100];
            buf[4] =  0x00;
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 14:      //PID-0x0E , Ignition Timing advance, range is -64 to 63.5 BTDC , formula == A/2 - 64 
            buf[0] =  0x03;                     // sending 3 bytes
            buf[1] =  0x41;                     // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x0E;                     // pid code
            buf[3] =  SpeedyResponse[23];
            buf[4] =  0x00;
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 15:      //PID-0x0F , Inlet air temperature , range is -40 to 215 deg C, formula == A-40 
            buf[0] =  0x03;                                                         // sending 3 bytes
            buf[1] =  0x41;                                                         // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x0F;                                                         // pid code
            buf[3] =  SpeedyResponse[6];                                            // A
            buf[4] =  0x00;                                                         // B
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 17:  // PID-0x11 , TPS percentage, range is 0 to 100 percent, formula == 100/256 A 
            buf[0] =  0x03;                    // sending 3 bytes
            buf[1] =  0x41;                    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x11;                    // pid code
            buf[3] =  SpeedyResponse[24];      // A
            buf[4] =  0x00;                    // B
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 19:      //PID-0x13 , oxygen sensors present, A0-A3 == bank1 , A4-A7 == bank2 , 
            uint16_t O2present;
            O2present = B00000011 ;       //realtimebufferA[24];         TEST VALUE !!!!!
            buf[0] =  0x03;           // sending 3 bytes
            buf[1] =  0x41;           // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x13;           // pid code
            buf[3] =  O2present ;     // A
            buf[4] =  0x00;           // B
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 28:      // PID-0x1C obd standard
            uint16_t obdstandard;
            obdstandard = 7;              // This is OBD2 / EOBD
            buf[0] =  0x03;           // sending 3 bytes
            buf[1] =  0x41;           // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x1C;           // pid code
            buf[3] =  obdstandard;    // A
            buf[4] =  0x00;           // B
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 32:      // PID-0x20 PIDs supported [21-40]
            buf[0] =  0x06;          // sending 6 bytes
            buf[1] =  0x41;          // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x20;          // pid code
            buf[3] =  B00011000;     //33-40
            buf[4] =  B00000000;     //41-48
            buf[5] =  B00100000;     //49-56
            buf[6] =  B00000001;     //57-64
            buf[7] = 0x00;
            break;

        case 36:      // PID-0x24 O2 sensor, AB: fuel/air equivalence ratio, CD: voltage ,  Formula == (2/65536)(256A +B) , 8/65536(256C+D) , Range is 0 to <2 and 0 to >8V 
            //uint16_t O2_1e ;
            //int16_t O2_1v ; 
            obdcalcH16 = 147/10 ;            // configPage2.stoich(is *10 so 14.7 is 147)
            obdcalcE32 = SpeedyResponse[10]/10;            // afr(is *10 so 25.5 is 255) , needs a 32bit else will overflow
            obdcalcF32 = (obdcalcE32<<8) / obdcalcH16;      //this is same as (obdcalcE32/256) / obdcalcH16 . this calculates the ratio      
            obdcalcG16 = (obdcalcF32 *32768)>>8;          
            obdcalcA = highByte(obdcalcG16);
            obdcalcB = lowByte(obdcalcG16);       

            buf[0] =  0x06;    // sending 6 bytes
            buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x24;    // pid code
            buf[3] =  obdcalcA;   // A
            buf[4] =  obdcalcB;   // B
            buf[5] =  obdcalcA;   // C
            buf[6] =  obdcalcB;   // D
            buf[7] =  0x00;
            break;

        case 37:      //O2 sensor2, AB fuel/air equivalence ratio, CD voltage ,  2/65536(256A +B) ,8/65536(256C+D) , range is 0 to <2 and 0 to >8V
            //uint16_t O2_2e ;
            //int16_t O2_2V ; 
            obdcalcH16 = 147/10 ;            // configPage2.stoich(is *10 so 14.7 is 147)
            obdcalcE32 = SpeedyResponse[39]/10;            // afr(is *10 so 25.5 is 255) , needs a 32bit else will overflow
            obdcalcF32 = (obdcalcE32<<8) / obdcalcH16;      //this is same as (obdcalcE32/256) / obdcalcH16 . this calculates the ratio      
            obdcalcG16 = (obdcalcF32 *32768)>>8;          
            obdcalcA = highByte(obdcalcG16);
            obdcalcB = lowByte(obdcalcG16);       

            buf[0] =  0x06;    // sending 6 bytes
            buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x25;    // pid code
            buf[3] =  obdcalcA;   // A
            buf[4] =  obdcalcB;   // B
            buf[5] =  obdcalcA;   // C
            buf[6] =  obdcalcB;   // D 
            buf[7] =  0x00;
            break;

        case 51:      //PID-0x33 Absolute Barometric pressure , range is 0 to 255 kPa , formula == A
            buf[0] =  0x03;                  // sending 3 bytes
            buf[1] =  0x41;                  // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x33;                  // pid code
            buf[3] =  SpeedyResponse[40];    // A
            buf[4] =  0x00;                  // B which is 0 as unused
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 64:      // PIDs supported [41-60]  
            buf[0] =  0x06;    // sending 4 bytes
            buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x40;    // pid code
            buf[3] =  B01000100;    // 65-72dec
            buf[4] =  B00000000;    // 73-80
            buf[5] =  B01000000;   //  81-88
            buf[6] =  B00010000;   //  89-96
            buf[7] =  0x00;
            break;

        case 66:      //control module voltage, 256A+B / 1000 , range is 0 to 65.535v
            uint16_t temp_ecuBatt;
            temp_ecuBatt = SpeedyResponse[9];
            obdcalcA = temp_ecuBatt*100;               // should be *1000 but ecuBatt is already *10
            buf[0] =  0x04;                       // sending 4 bytes
            buf[1] =  0x41;                       // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x42;                       // pid code
            buf[3] =  highByte(obdcalcA) ;        // A
            buf[4] =  lowByte(obdcalcA) ;         // B
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 70:        //PID-0x46 Ambient Air Temperature , range is -40 to 215 deg C , formula == A-40
            uint16_t temp_ambientair;
            temp_ambientair = 11;              // TEST VALUE !!!!!!!!!!
            obdcalcA = temp_ambientair + 40 ;    // maybe later will be (byte)(currentStatus.AAT + CALIBRATION_TEMPERATURE_OFFSET)
            buf[0] =  0x03;             // sending 3 byte
            buf[1] =  0x41;             // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x46;             // pid code
            buf[3] =  obdcalcA;         // A 
            buf[4] =  0x00;
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 82:        //PID-0x52 Ethanol fuel % , range is 0 to 100% , formula == (100/255)A
            buf[0] =  0x03;                       // sending 3 byte
            buf[1] =  0x41;                       // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc. 
            buf[2] =  0x52;                       // pid code
            buf[3] =  SpeedyResponse[34];
            buf[4] =  0x00;
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 92:        //PID-0x5C Engine oil temperature , range is -40 to 210 deg C , formula == A-40
            uint16_t temp_engineoiltemp;
            temp_engineoiltemp = 40;              // TEST VALUE !!!!!!!!!! 
            obdcalcA = temp_engineoiltemp+40 ;    // maybe later will be (byte)(currentStatus.EOT + CALIBRATION_TEMPERATURE_OFFSET)
            buf[0] =  0x03;                // sending 3 byte
            buf[1] =  0x41;                // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc. 
            buf[2] =  0x5C;                // pid code
            buf[3] =  obdcalcA ;           // A
            buf[4] =  0x00;
            buf[5] =  0x00; 
            buf[6] =  0x00; 
            buf[7] =  0x00;
            break;

        case 96:       //PIDs supported [61-80]  
            buf[0] =  0x06;    // sending 4 bytes
            buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0x60;    // pid code
            buf[3] =  0x00;    // B0000 0000
            buf[4] =  0x00;    // B0000 0000
            buf[5] =  0x00;    // B0000 0000
            buf[6] =  0x00;    // B0000 0000
            buf[7] =  0x00;
            break;

        case 208:      // PID-0xD0 Pulsewidth, ms multiplied by 10 (uS).
            buf[0] =  0x04;    // sending 4 bytes
            buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0xD0;    // pid code
            buf[3] =  SpeedyResponse[21];
            buf[4] =  SpeedyResponse[20];
            buf[5] =  0x00;    // B0000 0000
            buf[6] =  0x00;    // B0000 0000
            buf[7] =  0x00;
            break;

        case 209:      // PID-0xD1 VE
            buf[0] =  0x03;    // sending 4 bytes
            buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0xD1;    // pid code
            buf[3] =  SpeedyResponse[18];
            buf[4] =  0x00;
            buf[5] =  0x00;
            buf[6] =  0x00;
            buf[7] =  0x00;
            break;

        case 210:      // PID-0xD2 O2 sensor, AB fuel/air 2/65536(256A +B) range is 0 to <2.
            obdcalcH16 = 147/10 ;            // configPage2.stoich(is *10 so 14.7 is 147)
            obdcalcE32 = SpeedyResponse[39]/10;            // afr(is *10 so 25.5 is 255) , needs a 32bit else will overflow
            obdcalcF32 = (obdcalcE32<<8) / obdcalcH16;      //this is same as (obdcalcE32/256) / obdcalcH16 . this calculates the ratio      
            obdcalcG16 = (obdcalcF32 *32768)>>8;          
            obdcalcA = highByte(obdcalcG16);
            obdcalcB = lowByte(obdcalcG16);       

            buf[0] =  0x04;    // sending 6 bytes
            buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            buf[2] =  0xD2;    // pid code
            buf[3] =  obdcalcA;   // A
            buf[4] =  obdcalcB;   // B
            buf[5] =  0x00;      // C
            buf[6] =  0x00;      // D 
            buf[7] =  0x00;
            break;

        default:
            // exit early.
            return;
    }

    CAN1.send(RESPOND_CAN_ID, 0, 0, 8, buf);
}

void handleN()
{
    Serial.read();  // cmd: 0x32, discard
    uint8_t packet_size = Serial.read();
    for (int i=0; i<packet_size; i++) {
        if (i < CAN_PACKET_SIZE) {
            SpeedyResponse[i] = Serial.read();
        }
        else {
            // discard
            Serial.read();
        }
    }

    #ifdef DEBUG
    Serial.print(packet_size);
    for (int i=0; i<packet_size; i++) {
      Serial.print(SpeedyResponse[i]);
    }
    #endif

    doRequest = true;               // restart data reading
    oldtime = millis();             // zero the timeout
    SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}

void handleR()
{
    // To do in the future.
    byte tmp0;
    byte tmp1;
    uint8_t canin_channel = Serial.read();
    tmp0 = Serial.read();  // read in lsb of source can address
    tmp1 = Serial.read();  // read in msb of source can address
    // CanAddress = tmp1<<8 | tmp0 ;
    Serial.write("G");                      // reply "G" cmd
    Serial.write(0);                        // send 0 to confirm cmd received but not valid
    Serial.write(canin_channel);            // destination channel
    for (int i=0; i<8; i++) {                // we need to still write some crap as an response, or real time data reading will slow down significantly
        Serial.write(0);
    }
    SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}

void readSerial()
{
    uint8_t currentCommand = Serial.read();
    switch (currentCommand)
    {
        case 'n':  // Speeduino sends data in n-message
            SerialState = N_MESSAGE;
            break;
        case 'R':  // Speeduino requests data in R-message
            SerialState = R_MESSAGE;
            break;
        default:
             #ifdef DEBUG
              Serial.write("Bad command1");
            #endif
            break;
    }
}

void loop() {
    // Speeduino get data Serial communication code - should be non-blocking
    switch(SerialState) {
        case NOTHING_RECEIVED:
            if (Serial.available() > 0) { readSerial(); }  // read bytes from serial3 to define what message speeduino is sending.
            break;
        case N_MESSAGE:
            if (Serial.available() >= CAN_PACKET_SIZE) { handleN(); }  // read and process the A-message from serial3, when it's fully received.
            break;
        case R_MESSAGE:
            if (Serial.available() >= 3) {  handleR(); }  // read and process the R-message from serial3, when it's fully received.
            break;
        default:
            #ifdef DEBUG
              Serial.write("Bad command2");
            #endif
            break;
    }
    if ( (millis()-oldtime) > 500) { // timeout
        oldtime = millis();
        doRequest = true;                // restart data reading
    }

    // Handle CAN communication.
    handleCanMessage();
}
