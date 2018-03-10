#include "mbed/mbed.h"
#include "SHA256.h"
#include "Timer.h"
#include "rtos.h"


//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
const int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

// Function prototypes
void mine();
void interrupt();
inline int8_t readRotorState();
int8_t motorHome();
void motorOut(int8_t driveState, float duty);
void setSpeed();
void setPWMperiod(float period);
void motorOff();
void commOutFn();
void putMessage(uint8_t code, uint32_t data);
// Define Threads

// Declare global variables
int8_t orState = 0;    //Rotot offset at motor state 0
int32_t largeStep = 0;   
int32_t angVel = 0;
int32_t setVel = 10;
Timer t2;
int32_t targetSpeed = 0;
float duty = 1;
bool debugFlag = true;
Thread commOutT;
typedef struct{
    uint8_t code;
    uint32_t data;
 } message_t ;
Mail<message_t,16> outMessages;

Serial pc(SERIAL_TX, SERIAL_RX);
  
//Main
int main() {
    
    //Thread 
    commOutT.start(commOutFn);

    // Set PWM freq
    //setPWMperiod(0.00001);
    motorOff();

    //Initialise the serial port
    
    //int8_t intState = 0;
    //int8_t intStateOld = 0;
    putMessage(1, 1);
    
    //Run the motor synchronisation, should be 1 for motor 13
    //orState = motorHome();
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    if(orState !=3 && debugFlag){
        motorOut((-orState+lead+6)%6, 0);
        wait(1);
        orState = motorHome();
    }
    if(orState !=3 && debugFlag){
        motorOff();
        pc.printf("Failure to set motor to state 3\n\r");
        return 1;
    }
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    
    I1.rise(&interrupt);
    I1.fall(&interrupt);
    I2.rise(&interrupt);
    I2.fall(&interrupt);
    I3.rise(&interrupt);
    I3.fall(&interrupt);
    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    interrupt();
    
    //miningThr.start(mine)
    // osThreadCreate(osThread(mine), NULL);
    Timer t1;
    targetSpeed = 1000/(6*setVel);
    t1.start();
    t2.start();
    interrupt();
    while (1) {
        setSpeed();
        // pc.printf("tick\n\r");
        if(t1.read() >= 5){
            angVel = largeStep/(6*t1);
            pc.printf("Speed: %d\n\r", angVel);
            largeStep = 0;
            t1.reset();
        }
    }
}
/*
void setPWMperiod(float period){
    L1L.period(period);
    L1H.period(period);
    L2L.period(period);
    L2H.period(period);
    L3L.period(period);
    L3H.period(period);
}*/

void interrupt(){
    largeStep++;
    
}

void setSpeed(){
    //if(t2.read_ms() >= targetSpeed){
        int8_t intState = readRotorState();
        motorOut((intState-orState+lead+6)%6, duty); //+6 to make sure the remainder is positive
        // t2.reset();
    //}
}
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0, 0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}

//Set a given drive state
void motorOut(int8_t driveState, float duty){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
}

void motorOff(){
     
    //Turn off first
    L1L = 0;
    L1H = 1;
    L2L = 0;
    L2H = 1;
    L3L = 0;
    L3H = 1;
}

void mine(){
    //Crypto
    SHA256 crypt;
    Serial pc2(SERIAL_TX, SERIAL_RX);
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];
    
    uint8_t hashCount = 0;
    Timer t;

    while (1) {
        t.start();
        *key = 0;
        crypt.computeHash(hash, sequence, 64);
        *nonce = *nonce + 1;
        //pc.printf("%d\n\r", nonce);
        if(hash[0] == 0 && hash[1] == 0){
            pc2.printf("Found\n\r");
            hashCount++;
        }
        //t.stop();
        if(t.read() > 1){
            t.reset();
            pc2.printf("%d\n\r", hashCount);
            hashCount = 0;
        }
        //t.start()
    }
}

void commOutFn(){
    while(1) {
osEvent newEvent = outMessages.get();
message_t *pMessage = (message_t*)newEvent.value.p;
pc.printf("Message %d with data 0x%016x\n",
pMessage->code,pMessage->data);
outMessages.free(pMessage);
 }
}

void putMessage(uint8_t code, uint32_t data){
    message_t *pMessage = outMessages.alloc();
    pMessage->code = code;
    pMessage->data = data;
    outMessages.put(pMessage);
 }