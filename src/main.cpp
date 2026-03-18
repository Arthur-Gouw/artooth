#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdio.h>

// ---------------- Motor pin definitions (ORIGINAL — DO NOT CHANGE) --------
#define DIRA1 3 
#define DIRA2 10  // A motor (left front)

#define DIRB1 11 
#define DIRB2 12  // B motor (right front)

#define DIRC1 13 
#define DIRC2 14  // C motor (left rear)

#define DIRD1 21  
#define DIRD2 47  // D motor (right rear)

#define RXD2 16  // GPIO16 as RX
#define TXD2 17  // GPIO17 as TX

// ---------------- IR Sensor pins (minilab 5) ----------------
#define IR_PIN_0  5   // bit0 = right sensor
#define IR_PIN_1  6   // bit1 = middle sensor
#define IR_PIN_2  7   // bit2 = left sensor
#define IR_PIN_3  15  // bit3 (unused for now)
#define IR_PIN_4  38  // bit4 (unused)
#define IR_PIN_5  39  // bit5 (unused)
#define IR_PIN_6  40  // bit6 (unused)

const int irPins[7] = {IR_PIN_0, IR_PIN_1, IR_PIN_2,
                       IR_PIN_3, IR_PIN_4, IR_PIN_5, IR_PIN_6};

// ---------------- Motor control macros (ORIGINAL — DO NOT CHANGE) ---------
// A = Left front
#define MOTORA_FORWARD(pwm)    do{analogWrite(DIRA1,0);   analogWrite(DIRA2,pwm);}while(0)
#define MOTORA_STOP(pwm)       do{analogWrite(DIRA1,0);   analogWrite(DIRA2,0); }while(0)
#define MOTORA_BACKOFF(pwm)    do{analogWrite(DIRA1,pwm); analogWrite(DIRA2,0); }while(0)

// B = Right front
#define MOTORB_FORWARD(pwm)    do{analogWrite(DIRB1,0);   analogWrite(DIRB2,pwm);}while(0)
#define MOTORB_STOP(pwm)       do{analogWrite(DIRB1,0);   analogWrite(DIRB2,0); }while(0)
#define MOTORB_BACKOFF(pwm)    do{analogWrite(DIRB1,pwm); analogWrite(DIRB2,0); }while(0)

// C = Left rear
#define MOTORC_FORWARD(pwm)    do{analogWrite(DIRC1,pwm); analogWrite(DIRC2,0);}while(0)
#define MOTORC_STOP(pwm)       do{analogWrite(DIRC1,0);   analogWrite(DIRC2,0); }while(0)
#define MOTORC_BACKOFF(pwm)    do{analogWrite(DIRC1,0);   analogWrite(DIRC2,pwm); }while(0)

// D = Right rear
#define MOTORD_FORWARD(pwm)    do{analogWrite(DIRD1,pwm); analogWrite(DIRD2,0);}while(0)
#define MOTORD_STOP(pwm)       do{analogWrite(DIRD1,0);   analogWrite(DIRD2,0); }while(0)
#define MOTORD_BACKOFF(pwm)    do{analogWrite(DIRD1,0);   analogWrite(DIRD2,pwm); }while(0)

#define SERIAL  Serial

// ---------------- PWM parameters (ORIGINAL) ----------------
#define MAX_PWM   200
#define MIN_PWM   50

int Motor_PWM = 160;

HardwareSerial uart2(2); // UART2 for Pi communication

// ---------------- IR broadcast interval ----------------
unsigned long lastIrSend = 0;
const unsigned long IR_SEND_INTERVAL = 50;  // ms (20 Hz)

// ============================================================
// Low-level motion functions
// ============================================================

//    ↑A-----B↑   
//     |  ↑  |
//    ↑C-----D↑
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM); MOTORB_FORWARD(Motor_PWM);    
  MOTORC_FORWARD(Motor_PWM); MOTORD_FORWARD(Motor_PWM);    
}

//    ↓A-----B↓ 
//     |  ↓  |
//    ↓C-----D↓
void BACK()
{
  MOTORA_BACKOFF(Motor_PWM); MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); MOTORD_BACKOFF(Motor_PWM);
}

// STRAFE left (mecanum lateral movement — A/D keys in manual mode)
//    ↓A-----B↑   
//     |  ←  |
//    ↑C-----D↓
void LEFT_2()
{
  MOTORA_BACKOFF(Motor_PWM); MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); MOTORD_BACKOFF(Motor_PWM);
}

// STRAFE right (mecanum lateral movement — A/D keys in manual mode)
//    ↑A-----B↓   
//     |  →  |
//    ↓C-----D↑
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM); MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); MOTORD_FORWARD(Motor_PWM);
}

// ============================================================
// TANK ROTATION — Q/E keys and used by line-following FSM
// Both sides work in OPPOSITE directions → spins from center
// ============================================================

// ROTATE LEFT (CCW):
// Left side (A,C) go FORWARD, Right side (B,D) go BACKWARD
//    ↑A-----B↓
//     | CCW  |
//    ↑C-----D↓
void TURNLEFT()
{
  MOTORA_FORWARD(Motor_PWM);   // A (left front)  = forward
  MOTORB_BACKOFF(Motor_PWM);   // B (right front)  = backward
  MOTORC_FORWARD(Motor_PWM);   // C (left rear)   = forward
  MOTORD_BACKOFF(Motor_PWM);   // D (right rear)   = backward
}

// ROTATE RIGHT (CW):
// Left side (A,C) go BACKWARD, Right side (B,D) go FORWARD
//    ↓A-----B↑
//     |  CW  |
//    ↓C-----D↑
void TURNRIGHT()
{
  MOTORA_BACKOFF(Motor_PWM);   // A (left front)  = backward
  MOTORB_FORWARD(Motor_PWM);   // B (right front)  = forward
  MOTORC_BACKOFF(Motor_PWM);   // C (left rear)   = backward
  MOTORD_FORWARD(Motor_PWM);   // D (right rear)   = forward
}

//    =A-----B=  
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM); MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); MOTORD_STOP(Motor_PWM);
}

// ---------------- Optional: other moves kept from original ----------------
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM);    MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); MOTORD_STOP(Motor_PWM);
}

void LEFT_3()
{
  MOTORA_BACKOFF(Motor_PWM); MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);    MOTORD_BACKOFF(Motor_PWM);
}

void RIGHT_1()
{
  MOTORA_FORWARD(Motor_PWM); MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);    MOTORD_FORWARD(Motor_PWM);
}

void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM);    MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); MOTORD_STOP(Motor_PWM);
}

// Old one-sided pivots kept for reference but NOT used
void TURNLEFT_1()
{
  MOTORA_STOP(Motor_PWM); MOTORB_FORWARD(Motor_PWM);
  MOTORC_STOP(Motor_PWM); MOTORD_FORWARD(Motor_PWM);
}

void TURNRIGHT_1()
{
  MOTORA_FORWARD(Motor_PWM); MOTORB_STOP(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); MOTORD_STOP(Motor_PWM);
}

// ---------------- Logging ----------------
#define LOG_DEBUG

#ifdef LOG_DEBUG
#define M_LOG SERIAL.print
#else
#define M_LOG 
#endif

// ---------------- IO init ----------------
void IO_init()
{
  pinMode(DIRA1, OUTPUT); pinMode(DIRA2, OUTPUT);
  pinMode(DIRB1, OUTPUT); pinMode(DIRB2, OUTPUT);
  pinMode(DIRC1, OUTPUT); pinMode(DIRC2, OUTPUT);
  pinMode(DIRD1, OUTPUT); pinMode(DIRD2, OUTPUT);
  STOP();
}

// ============================================================
// Speed mapping
// ============================================================

int speedToPwm(int speedPercent)
{
  if (speedPercent <= 0) return 0;
  if (speedPercent > 100) speedPercent = 100;
  int pwm = MIN_PWM + (MAX_PWM - MIN_PWM) * speedPercent / 100;
  return pwm;
}

void advance(int speed)
{
  Motor_PWM = speedToPwm(speed);
  if (Motor_PWM == 0) { STOP(); } else { ADVANCE(); }
}

void back(int speed)
{
  Motor_PWM = speedToPwm(speed);
  if (Motor_PWM == 0) { STOP(); } else { BACK(); }
}

// Strafe left/right — only used by A/D manual keys
void left2(int speed)
{
  Motor_PWM = speedToPwm(speed);
  if (Motor_PWM == 0) { STOP(); } else { LEFT_2(); }
}

void right2(int speed)
{
  Motor_PWM = speedToPwm(speed);
  if (Motor_PWM == 0) { STOP(); } else { RIGHT_2(); }
}

// Tank rotation — used by Q/E keys AND by line-following FSM
void turnleft(int speed)
{
  Motor_PWM = speedToPwm(speed);
  if (Motor_PWM == 0) { STOP(); } else { TURNLEFT(); }
}

void turnright(int speed)
{
  Motor_PWM = speedToPwm(speed);
  if (Motor_PWM == 0) { STOP(); } else { TURNRIGHT(); }
}

// ============================================================
// IR sensor reading
// ============================================================
uint8_t readIrStatus()
{
  uint8_t val = 0;
  for (int i = 0; i < 7; i++) {
    if (digitalRead(irPins[i])) {
      val |= (1 << i);
    }
  }
  return val;
}

// ============================================================
// UART command handling
// ============================================================

void handleCommand(const String& commandName, const String& paramsStr)
{
  int speed = 0;
  if (paramsStr.length() > 0) {
    speed = paramsStr.toInt();
  }

  SERIAL.print("Command: ");
  SERIAL.print(commandName);
  SERIAL.print("  params: ");
  SERIAL.println(paramsStr);

  if (commandName == "mv_fwd") {
    advance(speed);
  } else if (commandName == "mv_rev") {
    back(speed);
  } else if (commandName == "mv_left") {
    left2(speed);                    // STRAFE (A key) — mecanum lateral
  } else if (commandName == "mv_right") {
    right2(speed);                   // STRAFE (D key) — mecanum lateral
  } else if (commandName == "mv_turnleft") {
    turnleft(speed);                 // TANK ROTATE (Q key) — both sides opposite
  } else if (commandName == "mv_turnright") {
    turnright(speed);                // TANK ROTATE (E key) — both sides opposite
  } else if (commandName == "stop") {
    STOP();
  } else {
    SERIAL.println("Unknown command");
  }
}

// ============================================================
// Setup & loop
// ============================================================

void setup()
{
  SERIAL.begin(115200);
  uart2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  uart2.setTimeout(10);

  // IR sensor pins
  for (int i = 0; i < 7; i++) {
    pinMode(irPins[i], INPUT);
  }

  uart2.println("ESP32 Ready to communicate with Raspberry Pi");
  IO_init();
}

unsigned long lastPingTime = 0;
const unsigned long PING_INTERVAL_MS = 1500;

void loop()
{
  // --- Read UART commands from Raspberry Pi ---
  if (uart2.available()) {
    String input = uart2.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      SERIAL.print("UART RX: ");
      SERIAL.println(input);

      int separatorIndex = input.indexOf(':');
      if (separatorIndex != -1) {
        String commandName = input.substring(0, separatorIndex);
        String paramsStr   = input.substring(separatorIndex + 1);
        handleCommand(commandName, paramsStr);
      }
    }
  }

  // --- Broadcast IR status to Raspberry Pi at 20 Hz ---
  unsigned long now = millis();
  if (now - lastIrSend >= IR_SEND_INTERVAL) {
    uint8_t ir = readIrStatus();
    uart2.println("IR_STATUS:" + String(ir));
    lastIrSend = now;
  }

  // --- Periodic heartbeat ---
  if (now - lastPingTime >= PING_INTERVAL_MS) {
    uart2.println("Hello from ESP32");
    lastPingTime = now;
  }
}