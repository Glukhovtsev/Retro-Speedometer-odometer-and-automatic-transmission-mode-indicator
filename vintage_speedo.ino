#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <EEPROM.h>
#include <util/atomic.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

struct StateMap { uint8_t mask; char value; };

const StateMap stateTable[] = {    /// D1...D7
  {1<<1,'1'}, //D2
  {1<<2,'2'}, //D3
  {1<<3,'D'}, //D4
  {1<<4,'N'}, //D5
  {1<<5,'R'}, //D6
  {1<<6,'P'}, //D7
  {(1<<3)|(1<<0),'3'} //D4&D1
};

const uint8_t fullStep[4][4] = {
  {1,0,0,0},
  {0,1,0,0},
  {0,0,1,0},
  {0,0,0,1}
};

const uint16_t PULSES_PER_100M = 2170;
const uint8_t SENSOR_PIN = B0;


const uint8_t SPEED_STEP_KMH = 10;
const uint8_t SPEED_LUT_SIZE = 15;

const uint16_t speedAngleLUT[SPEED_LUT_SIZE] = {
     0,  125,  251,  432,  612,
   793,  973, 1151, 1331, 1512,
  1691, 1874, 2051, 2234, 2413
};

const float SPEED_ON_THRESHOLD  = 1.0;
const float SPEED_OFF_THRESHOLD = 0.5;

const int NEEDLE_SLOW_ZONE = 25;


const int NEEDLE_STOP_SOFTNESS = 5;

/* ================= EEPROM ================= */

const uint8_t RECORD_COUNT = 20;

struct Record {
  uint32_t distance_100m;
  uint8_t  seq;
  uint8_t  crc;
};

/* ================= CRC-8 ================= */

uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
  }
  return crc;
}

const uint8_t STEP_PINS[4] = {A0, A1, A2, A3};

const uint16_t STEPS_MAX          = 630;
const uint16_t STEPS_PER_REV_CALC = 720;

const uint16_t MOTOR_POWER_OFF_DELAY_MS = 800;
bool motorPowered = true;
unsigned long lastMotorMoveMs = 0;
volatile bool motorStepped = false;

#define DEG10_TO_STEPS(x) ((long)(x) * STEPS_PER_REV_CALC / 3600L)

void motorPowerOff() {
  motorPowered = false;
  for(uint8_t i=0;i<4;i++)
    digitalWrite(STEP_PINS[i], LOW);
}

int speedToStepsLUT(float speedKmh){
  if (speedKmh <= 0) return 0;

  float maxSpeed = (SPEED_LUT_SIZE - 1) * SPEED_STEP_KMH;
  if (speedKmh >= maxSpeed)
    return DEG10_TO_STEPS(speedAngleLUT[SPEED_LUT_SIZE - 1]);

  float idx = speedKmh / SPEED_STEP_KMH;
  int i = (int)idx;
  float frac = idx - i;

  long angle10 =
    speedAngleLUT[i] +
    (speedAngleLUT[i + 1] - speedAngleLUT[i]) * frac;

  return constrain(DEG10_TO_STEPS(angle10), 0, STEPS_MAX);
}

volatile uint8_t rawMask = 0;
volatile bool maskReady = false;
uint8_t appliedMask = 0;

char state = ' ';

volatile uint32_t pulseCounter = 0;
volatile uint32_t pulseCounterForSpeed = 0;

uint32_t distance_100m = 0;
uint8_t  currentRecordIndex = 0;
uint8_t  currentSeq = 0;

volatile uint32_t speedPulses = 0;
volatile bool speedReady = false;

float kmhPerPulse = 0;

bool needleActive = false;

volatile int currentStepPos = 0;
volatile int targetStepPos  = 0;
volatile int smoothTargetStepPos = 0;
uint8_t motorPhase = 0;

const uint8_t rateDiv = 2;
const int TARGET_DEADBAND_STEPS = 2; //// чувствительность стрелки (0 - макс чувствительность)

char decodeState(uint8_t mask){
  for(uint8_t i=0; i < sizeof(stateTable) / sizeof(stateTable[0]); i++)
    if(stateTable[i].mask==mask) return stateTable[i].value;
  return ' ';
}

ISR(PCINT0_vect) {
  static uint8_t last = 0;
  uint8_t now = PINB & (1<<PB0);
  if (now && !last) {
    pulseCounter++;
    pulseCounterForSpeed++;
  }
  last = now;

  if (pulseCounter >= PULSES_PER_100M) {
    pulseCounter = 0;
    if (distance_100m < 9999999) distance_100m++;
  }
}

ISR(TIMER1_COMPA_vect) {
  speedPulses = pulseCounterForSpeed;
  pulseCounterForSpeed = 0;
  speedReady = true;
}

ISR(PCINT2_vect) {
  rawMask = (PIND >> 1) & 0b01111111;
  maskReady = true;
}

ISR(TIMER2_COMPA_vect) {
  if (!motorPowered) return;
  if (currentStepPos == smoothTargetStepPos) return;

  static uint8_t div = 0;
  if (++div < rateDiv) return;
  div = 0;

  int dir = (smoothTargetStepPos > currentStepPos) ? 1 : -1;
  motorPhase = (motorPhase + dir + 4) % 4;

  for(uint8_t i=0;i<4;i++)
    digitalWrite(STEP_PINS[i], fullStep[motorPhase][i]);

  currentStepPos += dir;
  motorStepped = true;
}

void saveDistance() {
  currentSeq++;
  currentRecordIndex = (currentRecordIndex + 1) % RECORD_COUNT;

  Record rec;
  rec.distance_100m = distance_100m;
  rec.seq = currentSeq;
  rec.crc = crc8((uint8_t*)&rec, 5);

  EEPROM.put(currentRecordIndex * sizeof(Record), rec);
}

void loadDistance() {
  Record rec;
  bool found = false;
  uint8_t bestSeq = 0;

  for(uint8_t i=0;i<RECORD_COUNT;i++){
    EEPROM.get(i * sizeof(Record), rec);

    if (crc8((uint8_t*)&rec, 5) != rec.crc)
      continue;

    if (!found || (uint8_t)(rec.seq - bestSeq) < 128) {
      distance_100m = rec.distance_100m;
      bestSeq = rec.seq;
      currentSeq = rec.seq;
      currentRecordIndex = i;
      found = true;
    }
  }

  if (!found) {
    distance_100m = 0;
    currentSeq = 0;
    currentRecordIndex = 0;
    saveDistance();
  }
}

void setupSpeedTimer(float PeriodS) {
  noInterrupts();

  kmhPerPulse = 360.0 / (PULSES_PER_100M * PeriodS);

  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1B |= (1<<WGM12) | (1<<CS12) | (1<<CS10);

  OCR1A = (uint16_t)(F_CPU / 1024.0 * PeriodS);

  TCNT1 = 0;
  TIMSK1 |= (1<<OCIE1A);

  interrupts();
}

void setup(){

  pinMode(SENSOR_PIN, INPUT);

  loadDistance();

  PCICR |= (1<<PCIE0) | (1<<PCIE2);
  PCMSK0 |= (1<<PCINT0);
  PCMSK2 |= 0b11111110;

  appliedMask = (PIND >> 1) & 0b01111111;
  rawMask = appliedMask;
  state = decodeState(appliedMask);

  setupSpeedTimer(0.3);

  TCCR2A = (1<<WGM21);
  TCCR2B = (1<<CS22) | (1<<CS20);
  OCR2A  = 161;
  TIMSK2 |= (1<<OCIE2A);

  for(uint8_t i=0;i<4;i++){
    pinMode(STEP_PINS[i], OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
  }

  motorPowered = true;

  currentStepPos      = 0;
  targetStepPos       = 0;
  smoothTargetStepPos = 0;

  targetStepPos       = STEPS_MAX + 10;
  smoothTargetStepPos = STEPS_MAX + 10;

  while (true) {
    bool done;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      done = (currentStepPos == smoothTargetStepPos);
    }
    if (done) break;
  }

  delay(150);

  targetStepPos       = 0;
  smoothTargetStepPos = 0;

  while (true) {
    bool done;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      done = (currentStepPos == smoothTargetStepPos);
    }
    if (done) break;
  }

  delay(100);

  currentStepPos      = 0;
  targetStepPos       = 0;
  smoothTargetStepPos = 0;

  motorPowerOff();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
}

void loop(){

  static uint32_t lastSaved = 0;
  if(distance_100m != lastSaved){
    saveDistance();
    lastSaved = distance_100m;
  }

  if(maskReady){
    maskReady = false;
    if(rawMask != appliedMask){
      appliedMask = rawMask;
      state = decodeState(appliedMask);
    }
  }

  if(speedReady){
    speedReady = false;

    float speedKmhRaw = (speedPulses + 0.5) * kmhPerPulse;

    float dynCal = 1.02 + 0.08 * exp(-pow(speedKmhRaw / 24.0, 1.2));
    if (speedKmhRaw < 10) dynCal *= 1.1;
    if (speedKmhRaw > 15 && speedKmhRaw < 25) dynCal *= 0.992;

    float speedKmh = speedKmhRaw * dynCal;

    if (speedKmh > SPEED_ON_THRESHOLD)  needleActive = true;
    if (speedKmh < SPEED_OFF_THRESHOLD) needleActive = false;

    int newTarget = needleActive ? speedToStepsLUT(speedKmh) : 0;

    if (abs(newTarget - targetStepPos) > TARGET_DEADBAND_STEPS) {
      targetStepPos = newTarget;
      motorPowered = true;
    }
  }


  int diff  = targetStepPos - smoothTargetStepPos;
  int adiff = abs(diff);

  if (diff != 0) {
    if (adiff > NEEDLE_SLOW_ZONE) {
      smoothTargetStepPos = targetStepPos;
    } else {
      int stepLimit = map(adiff, 0, NEEDLE_SLOW_ZONE, 1, NEEDLE_STOP_SOFTNESS);
      smoothTargetStepPos += constrain(diff, -stepLimit, stepLimit);
    }
    motorPowered = true;
  }

  if (motorStepped) {
    lastMotorMoveMs = millis();
    motorStepped = false;
  }

  int pos, tgt;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = currentStepPos;
    tgt = smoothTargetStepPos;
  }

  if (motorPowered &&
      pos == tgt &&
      (millis() - lastMotorMoveMs) > MOTOR_POWER_OFF_DELAY_MS) {
    motorPowerOff();
  }

  char distanceStrFull[9];
  snprintf(distanceStrFull, sizeof(distanceStrFull),
           "%6lu.%1u",
           distance_100m / 10,
           (unsigned)(distance_100m % 10));

  display.clearDisplay();
  display.setTextColor(WHITE);

  if (state != ' ') {
    display.setTextSize(3);
    display.setCursor(3,5);
    display.print(state);
  }

  display.setTextSize(2);
  display.setCursor(32,10);
  display.print(distanceStrFull);

  display.display();
}
