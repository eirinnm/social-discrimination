#include <Dynamixel2Arduino.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define DIR_PIN 2 //this is used to drive the interrupt. Should be 2, 3 or 7
#define BUTTON1 8
#define BUTTON2 6
#define BUTTON3 4
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

///--- Define the 3 positions here ---////
/// a value of 1024 is 90 degrees
/// Don't start at 0 because it shifts a bit into negative numbers and causes the next movement to go 270 degrees
/// so starting at 512 should be fine
const int POS1 = 400;
const int POS2 = 1024+512;
const int POS3 = 2048+512;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200); 
  dxl.begin(57600); //by default the motors use 57600
  attachInterrupt(digitalPinToInterrupt(DIR_PIN), dir_pin_changed, CHANGE); //for the half duplex switching
  // dxl.torqueOff(1);
  // dxl.setOperatingMode(1, OP_POSITION); //only needs to be set once per motor
  // dxl.torqueOn(1);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
}

unsigned long timeMotorLastMoved = 0;
bool motorEnabled = false;
byte led_counter = 0; //for the idle fade effect

void enableMotor(){
  dxl.writeControlTableItem(PROFILE_ACCELERATION, 1, 3);
  dxl.writeControlTableItem(PROFILE_VELOCITY, 1, 20);
  dxl.torqueOn(1);
  timeMotorLastMoved = millis();
  motorEnabled = true;
}


void loop() {
  if(!digitalRead(BUTTON1)){
    enableMotor();
    dxl.setGoalPosition(1, POS1);
  }
  if(!digitalRead(BUTTON2)){
    enableMotor();
    dxl.setGoalPosition(1, POS2);
  }
  if(!digitalRead(BUTTON3)){
    enableMotor();
    dxl.setGoalPosition(1, POS3);
  }
  // Serial.println(dxl.readControlTableItem(OPERATING_MODE, 1));
  // Serial.println(dxl.readControlTableItem(TORQUE_ENABLE, 1));
  Serial.println(dxl.getPresentPosition(1));

  if(motorEnabled){
    if (dxl.getPresentVelocity(1) != 0) {
      //motor is turning. Blink the LED and update the time record
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      timeMotorLastMoved = millis();
    }else{
      //motor has stopped. 
      digitalWrite(LED_BUILTIN, HIGH);
      //How long since it stopped?
      if(millis() - timeMotorLastMoved > 1000){
        //it's been stopped for a while so disable the torque
        dxl.torqueOff(1);
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
  delay(50);
}