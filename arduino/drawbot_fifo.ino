//  drawbot_fifo.ino
//
//  Runs a stepper driver, by reading step data and time data over serial
//  This code is public domain.
//  by Scott Cooper, <scottslongemailaddress@gmail.com>
//
//  Inspiration from https://code.google.com/p/gocupi/
//  Much of FIFO buffer written by Nick Gammon
//
//                   ----____---- 
//            Reset | 1       28 | Analog 5
//         pin 0 RX | 2       27 | Analog 4
//         pin 1 TX | 3       26 | Analog 3
//            pin 2 | 4       25 | Analog 2
//        pin 3 PWM | 5       24 | Analog 1
//            pin 4 | 6       23 | Analog 0
//              +5V | 7       22 | Ground
//           Ground | 8       21 | NC
//          Crystal | 9       20 | +5V
//          Crystal | 10      19 | pin 13
//        pin 5 PWM | 11      18 | pin 12
//        pin 6 PWM | 12      17 | pin 11 PWM
//            pin 7 | 13      16 | pin 10 PWM
//            pin 8 | 14      15 | pin 9 PWM
//                   ------------

#include <Servo.h> 
#include <digitalWriteFast.h>

// A hacky way to delay for exactly 62.5 nano seconds on a 16MHz ATmega
#define NOP __asm__ __volatile__ ("nop\n\t")

#define ledPin             4
#define requestDataPin    13
#define servoPin           8
#define motorsEnablePin   15
#define motor1StepPin     17
#define motor1DirPin      16
#define motor2StepPin     10
#define motor2DirPin       9

// Globals
const unsigned int BUFFER_CAPACITY = 1536;
const unsigned int BUFFER_FULL = 1516;
char buffer[BUFFER_CAPACITY];    // a circular buffer step commands and time delays
unsigned int bufferStart = 0;    // where data is currently being read from
unsigned int bufferLength = 0;   // the number of items in the buffer
unsigned long curTime;           // current time in microseconds
unsigned long prevTime = 0;      // time of previous interation
unsigned int interval = 0;       // delay after a step command
Servo myservo;                   // create servo object to control a servo 
int servo_position = 0;
int servo_last_position = 1;


////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);

  pinMode(ledPin,          OUTPUT);
  pinMode(requestDataPin,  OUTPUT);
  pinMode(motorsEnablePin, OUTPUT);
  pinMode(motor1StepPin,   OUTPUT);
  pinMode(motor1DirPin,    OUTPUT);
  pinMode(motor2StepPin,   OUTPUT);
  pinMode(motor2DirPin,    OUTPUT);
  digitalWrite(motorsEnablePin, HIGH);

  myservo.attach(servoPin);
  requestMoreData();
}

////////////////////////////////////////////////////////////////////////////////
void loop() {
  curTime = micros();
  if ((curTime - prevTime) >= interval) {
    prevTime = curTime;
    updatePins();
    requestMoreData();
  } 
  
  if (Serial.available() > 1) {
    bufferPut(Serial.read());
    bufferPut(Serial.read());
    requestMoreData();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update stepper, LED, and servo pins
//
// Command structure of first byte:
//    bit 7:  Unused
//    bit 6:  LED
//    bit 5:  Servo up, if high
//    bit 4:  Relax motors, if high
//    bit 3:  Motor1 step, if high
//    bit 2:  Motor1 step direction
//    bit 1:  Motor2 step, if high
//    bit 0:  Motor2 step direction
// Second byte is always the time delay
void updatePins() {
  char cmdByte;
  char timeByte;
 
  if (bufferLength > 1) {
    cmdByte = bufferGet();
    timeByte = bufferGet();
    //Serial.println("cmd: "); Serial.print(cmdByte, BIN);
    //Serial.println("time:"); Serial.print(timeByte);

    digitalWriteFast(motorsEnablePin, bitRead(cmdByte, 4));
    digitalWriteFast(motor1DirPin, bitRead(cmdByte, 2));
    digitalWriteFast(motor2DirPin, bitRead(cmdByte, 0));
    digitalWriteFast(ledPin, bitRead(cmdByte, 6));
    NOP; NOP; NOP; NOP;  // 250ns setup time
      
    if(bitRead(cmdByte, 3)) { digitalWriteFast(motor1StepPin, HIGH); }
    if(bitRead(cmdByte, 1)) { digitalWriteFast(motor2StepPin, HIGH); }
    NOP; NOP; NOP; NOP; // 250ns
   
    servo_position = bitRead(cmdByte, 5);
    if(servo_position != servo_last_position) {
      digitalWriteFast(requestDataPin, LOW);
      
      for(int c=0; c<=20; c++) {
        if (servo_position) {
          myservo.write(80 + c);
        } else {
          myservo.write(100 - c);
        }
        delay(10);
      }  
      delay(200);
      servo_last_position = servo_position;
    }

    digitalWriteFast(motor1StepPin, LOW);
    digitalWriteFast(motor2StepPin, LOW);
    interval = timeByte * 75;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put a value onto the end of the buffer
void bufferPut(char value) {
  int writePosition = bufferStart + bufferLength;
  if (writePosition >= BUFFER_CAPACITY) {
    writePosition = writePosition - BUFFER_CAPACITY;
  }

  buffer[writePosition] = value;

  if (bufferLength == BUFFER_CAPACITY) { // full, overwrite existing data
    bufferStart++;
    if (bufferStart == BUFFER_CAPACITY) {
      bufferStart = 0;
    }
  } else {
    bufferLength++;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get a value from the front of the buffer
char bufferGet() {
  if(bufferLength == 0) {
    return 0;
  }

  char result = buffer[bufferStart];
  bufferStart++;
  if (bufferStart == BUFFER_CAPACITY) {
    bufferStart = 0;
  }
  bufferLength--;
  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Turn on the requestDataPin if we have plenty of space in the buffer buffer
void requestMoreData() {
  if (bufferLength < BUFFER_FULL){
    digitalWriteFast(requestDataPin, HIGH);
  } else {
    digitalWriteFast(requestDataPin, LOW);
  }
}

////////////////////////////////////////////////////////////////////////////////



