#include <Dynamixel2Arduino.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define DIR_PIN 2 //this is used to drive the interrupt. Should be 2, 3 or 7
#define BUTTON0 15 //SCK
#define BUTTON1 16
#define BUTTON2 4
#define BUTTON3 6
#define BUTTON4 8
Dynamixel2Arduino dxl(Serial1, DIR_PIN);

//// Half-duplex TTL signalling ////
void set_output_mode(){
  // enable the TX pin for use with Serial1
  sbi(UCSR1B, TXEN1); //attach TX pin
  cbi(UCSR1B, RXEN1); //detach RX pin
  cbi(UCSR1B, RXCIE1); //detach RX interrupt
}

void set_input_mode(){
  //disable TX with Serial1
  cbi(UCSR1B, TXEN1); //detach TX pin
  sbi(UCSR1B, RXEN1); //attach RX pin
  sbi(UCSR1B, RXCIE1); //attach RX interrupt
  pinMode(1, INPUT_PULLUP);
}

void dir_pin_changed(){
  //read the DIR pin and set the in/out mode for the TTL signal
  if(digitalRead(DIR_PIN)){
    set_output_mode();
  }else{
    set_input_mode();
  }
}
//// End of half-duplex routines ////

///--- Define the 5 positions here ---////
/// a value of 1024 is 90 degrees
/// Don't start at 0 because it shifts a bit into negative numbers and causes the next movement to go 270 degrees
/// so starting at 512 should be fine
const int POS0 = 512; //45
const int POS1 = 1024*1; //90
const int POS2 = 1024*2; //180
const int POS3 = 1024*3; //270
const int POS4 = 10; //360
const int MOTOR1 = 1;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A3, OUTPUT);
  Serial.begin(115200); 
  dxl.begin(57600); //by default the motors use 57600
  attachInterrupt(digitalPinToInterrupt(DIR_PIN), dir_pin_changed, CHANGE); //for the half duplex switching
  pinMode(BUTTON0, INPUT_PULLUP);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(BUTTON4, INPUT_PULLUP);
  //
  ////////// Set motor options (if needed) /////////////
  //
  // to make permanent changes to motor options (such as operation mode, or ID),
  // we need to turn the torque off
  // dxl.torqueOff(MOTOR1);
  // dxl.setOperatingMode(MOTOR1, OP_POSITION); //only needs to be set once per motor
  // dxl.torqueOn(MOTOR1);
}

unsigned long timeMotorLastMoved = 0;
bool motorEnabled = false;
byte led_counter = 0; //for the idle fade effect

void enableMotor(){
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, MOTOR1, 3); //set accel to 3
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, MOTOR1, 20); // set top speed to 20
    dxl.torqueOn(MOTOR1);
    timeMotorLastMoved = millis();
    motorEnabled = true;
}


void loop() {
  if(!digitalRead(BUTTON0)){
    // Serial.println("button0");
    enableMotor();
    dxl.setGoalPosition(MOTOR1, POS0);
  }
  if(!digitalRead(BUTTON1)){
    // Serial.println("button1");
    enableMotor();
    dxl.setGoalPosition(MOTOR1, POS1);
  }
  if(!digitalRead(BUTTON2)){
    // Serial.println("button2");
    enableMotor();
    dxl.setGoalPosition(MOTOR1, POS2);
  }
  if(!digitalRead(BUTTON3)){
    // Serial.println("button3");
    enableMotor();
    dxl.setGoalPosition(MOTOR1, POS3);
  }
  if(!digitalRead(BUTTON4)){
    // Serial.println("button4");
    enableMotor();
    dxl.setGoalPosition(MOTOR1, POS4);
  }
  // Serial.println(dxl.readControlTableItem(OPERATING_MODE, MOTOR1));
  // Serial.println(dxl.readControlTableItem(TORQUE_ENABLE, MOTOR1));
  Serial.println(dxl.getPresentPosition(MOTOR1));

  if(motorEnabled){
    if (dxl.getPresentVelocity(MOTOR1) != 0) {
      //motor is turning. Blink the LED and update the time record
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      //also turn pin A3 high
      digitalWrite(A3, HIGH);
      timeMotorLastMoved = millis();
    }else{
      //motor has stopped. 
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(A3, LOW);
      //How long since it stopped?
      if(millis() - timeMotorLastMoved > 1000){
        //it's been stopped for a while so disable the torque
        dxl.torqueOff(MOTOR1);
        motorEnabled = false;
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
  }else{
    //motor is idle. Fade the LED in and out
    int led_brightness = abs(128-led_counter);
    analogWrite(LED_BUILTIN, led_brightness);
    led_counter+=8;
  }
  delay(20);
}