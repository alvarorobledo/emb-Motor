#include "mbed.h"
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
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

// Function prototypes
void mine();
void interrupt();
inline int8_t readRotorState();
int8_t motorHome();
void motorOut(int8_t driveState, uint32_t torque);
void setSpeed();
void setPWMperiod(float period);
void motorOff();
void commOutFn();
void putMessage(uint8_t code, uint32_t data);
void serialISR();
void commDecFn();
void decodeR();
void decodeV();
void decodeK();
void decodeT();
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
char inComm[30]; // Input command
int inCommPtr = 0; // Pointer to the first empty position of the buff array
//volatile uint64_t newKey = 0;
volatile uint64_t newKey = 0;
float PWMPeriod = 128;
Mutex newKey_mutex;
// Threads
Thread commOutT = Thread(osPriorityNormal);
Thread commDec = Thread(osPriorityNormal);// Command decode thread
Thread minning = Thread(osPriorityNormal, 1024);

typedef struct{
    uint8_t code;
    uint32_t data;
 } message_t ;
Mail<message_t,16> outMessages;
Queue<void, 8> inCharQ;

RawSerial pc(SERIAL_TX, SERIAL_RX);
  
//Main
int main() {
    
    //Thread 
    osThreadSetPriority(osThreadGetId(), osPriorityNormal);
    commOutT.start(commOutFn);
    commDec.start(commDecFn);
    //minning.start(mine);

    // Set PWM freq
    setPWMperiod(PWMPeriod);
    //motorOff();

    //Initialise the serial port
    
    //int8_t intState = 0;
    //int8_t intStateOld = 0;
    putMessage(1, 0x999);
    
    //Run the motor synchronisation, should be 1 for motor 13
    //orState = motorHome();
    orState = motorHome();
    //pc.printf("Rotor origin: %x\n\r",orState);
    if(orState !=3 && debugFlag){
        motorOut((-orState+lead+6)%6, 0);
        orState = motorHome();
    }
    if(orState !=3 && debugFlag){
        motorOff();
        //pc.printf("Failure to set motor to state 3\n\r");
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
        wait_ms(500);
        
        // pc.printf("tick\n\r");
        if(t1.read() >= 5){
            angVel = largeStep/(6*t1);
            //pc.printf("Speed: %d\n\r", angVel);
            largeStep = 0;
            t1.reset();
        }
    }
}

void setPWMperiod(float period){
    L1L.period_us(period);
    L2L.period_us(period);
    L3L.period_us(period);
}

void interrupt(){
    largeStep++;
    setSpeed();
}

void setSpeed(){
    //if(t2.read_ms() >= targetSpeed){
        int8_t intState = readRotorState();
        motorOut((intState-orState+lead+6)%6, PWMPeriod*duty); //+6 to make sure the remainder is positive
        // t2.reset();
    //}
}
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0, PWMPeriod);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}

//Set a given drive state
void motorOut(int8_t driveState, uint32_t torque){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(torque);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(torque);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(torque);
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
    //rial pc2(SERIAL_TX, SERIAL_RX);
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
        //putMessage(90, 1);
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();
        crypt.computeHash(hash, sequence, 64);

        *nonce = *nonce + 1;
        //pc.printf("%d\n\r", nonce);
        if(hash[0] == 0 && hash[1] == 0){
            putMessage(90, *key);
            putMessage(91, *nonce);
            hashCount++;
        }
        //t.stop();
        // if(t.read() > 1){
        //     t.reset();
        //     pc2.printf("%d\n\r", hashCount);
        //     hashCount = 0;
        // }
        //t.start()
    }
}

void commOutFn(){
    while(1) {
osEvent newEvent = outMessages.get();
message_t *pMessage = (message_t*)newEvent.value.p;
pc.printf("Message %d with data 0x%016x\r\n",
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

 void serialISR(){
     uint8_t newChar = pc.getc();
     inCharQ.put((void*)newChar);
 }

 void commDecFn(){
     pc.attach(&serialISR);
     inCommPtr = 0;
     while(1){
         osEvent newEvent = inCharQ.get();
         uint8_t newChar = (uint8_t) newEvent.value.p;
        if(newChar != (char)'\r'){
            inComm[inCommPtr++] = newChar;
        }else{
            inComm[inCommPtr++] = (char)'\0';
            inCommPtr = 0;
            // Decode command
            if(inComm[0] == (char)'R') decodeR();
            if(inComm[0] == (char)'V') decodeV();
            if(inComm[0] == (char)'K') decodeK();
            if(inComm[0] == (char)'T') decodeT();
        }
        if(inCommPtr == 30){
            inCommPtr = 0;
        }
        
     }

     
 }

 void decodeK(){
     newKey_mutex.lock();
     //sscanf(inComm, "K%llx", &newKey);
     uint8_t* tempPtr = (uint8_t*) &newKey;
     unsigned int tempInt1;
     unsigned int  tempInt2;
     for(int i = 0; i < 16; i = i + 2){
         
         sscanf(&inComm[16-i], "%1x", &tempInt1);
         sscanf(&inComm[15-i], "%1x", &tempInt2);
         *tempPtr = (uint8_t)(tempInt2 << 4 | tempInt1);
         tempPtr++;
     }
     newKey_mutex.unlock();
 }
 
void decodeR(){};
void decodeV(){};
void decodeT(){
    unsigned int tempInt1;
    sscanf(&inComm[3], "%2u", &tempInt1);
    duty = ((float) tempInt1)/100;
}