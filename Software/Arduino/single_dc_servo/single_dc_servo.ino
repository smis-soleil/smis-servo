#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AS5600.h>
#include <pid-autotune.h>

size_t strlcpy(char *dst, const char *src, size_t siz);

#include <CommandParser.h>

#ifndef ARDUINO
#define ARDUINO 10607
#endif

size_t strlcpy(char *dst, const char *src, size_t siz) {
  size_t srcLen = strlen(src);

  if (siz > 0) {
    size_t copyLen = (srcLen >= siz) ? (siz - 1) : srcLen;
    memcpy(dst, src, copyLen);
    dst[copyLen] = '\0';
  }

  return srcLen;
}

#define PWM_ADDRESS 0x40
#define PWM_CHANNEL 0
#define PWM_MAX 2048
#define PWM_MIN 270
#define PWM_DELAY_MS 2000
#define POLL_DELAY_MS 10
#define SERVO_FREQ 1000
#define SEQUENCE_STEP_MS 1000

#define ENCODER_COUNTS_PER_REV 4096
#define ENCODER_WRAP_THRESHOLD (ENCODER_COUNTS_PER_REV / 2)

#define POSITION_DEADBAND 0.5f
#define INTEGRAL_LIMIT 1500.0f
#define MIN_DRIVE_PWM 100

#define forward_pin 0
#define backward_pin 1

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Magnetic encoder
AMS_5600 ams5600;

typedef CommandParser<> MyCommandParser;
MyCommandParser parser;

int status = 0; // 0 - stopped, 1 - forward, -1 - backward

int lastRawAngle = 0;
bool encoderInitialized = false;
long continuousCounts = 0;

float continuousDeg = 0.0f;
float targetContinuousDeg = 0.0f;
float motorCommandPwm = 0.0f;
float sequenceMotorPwm = (float)PWM_MAX;

float pidKp = 9.0f;
float pidKi = 0.0f;
float pidKd = 0.07f;

bool tuningActive = false;
bool pidComputeEnabled = false;

enum SequenceMode : uint8_t {
  seqNone = 0,
  seqStep = 1,
  seqRamp = 2
};

bool sequenceActive = false;
SequenceMode activeSequenceMode = seqNone;
uint8_t sequenceStep = 0;
unsigned long sequenceStepStartMs = 0;
unsigned long sequenceRampPhaseStartMs = 0;
uint8_t sequenceRampPhase = 0;

PID anglePid;
pid_tuner angleTuner(anglePid);

void runMotorPwmSequence();
void updateMotorPwmSequence();

void updateContinuousCount(int rawAngle) {
  if (!encoderInitialized) {
    lastRawAngle = rawAngle;
    encoderInitialized = true;
    return;
  }

  int delta = rawAngle - lastRawAngle;

  // Unwrap periodic encoder reading so 4095->0 is +1 and 0->4095 is -1.
  if (delta > ENCODER_WRAP_THRESHOLD) {
    delta -= ENCODER_COUNTS_PER_REV;
  } else if (delta < -ENCODER_WRAP_THRESHOLD) {
    delta += ENCODER_COUNTS_PER_REV;
  }

  continuousCounts += delta;
  lastRawAngle = rawAngle;
}

void cmd_set(CommandParser<>::Argument *args, char *response) {
  targetContinuousDeg = (float)args[0].asDouble;
  anglePid.setSetPoint(targetContinuousDeg);
  snprintf(response,
           CommandParser<>::MAX_RESPONSE_SIZE,
           "OK: TARGET_DEG=%.2f",
           targetContinuousDeg);
}

void cmd_get(CommandParser<>::Argument *args, char *response) {
  (void)args;
  int rawAngle = ams5600.getRawAngle();
  updateContinuousCount(rawAngle);
  continuousDeg = (continuousCounts * 360.0f) / ENCODER_COUNTS_PER_REV;
  snprintf(response,
           CommandParser<>::MAX_RESPONSE_SIZE,
           "RAW=%d CONT_COUNTS=%ld CONT_DEG=%.2f TARGET_DEG=%.2f OUT=%.1f KP=%.3f KI=%.3f KD=%.3f TUNING=%d",
           rawAngle,
           continuousCounts,
           continuousDeg,
           targetContinuousDeg,
           motorCommandPwm,
           pidKp,
           pidKi,
           pidKd,
           tuningActive ? 1 : 0);
}

void cmd_tune(CommandParser<>::Argument *args, char *response) {
  (void)args;

  if (tuningActive) {
    snprintf(response,
             CommandParser<>::MAX_RESPONSE_SIZE,
             "ERR: TUNING ALREADY RUNNING");
    return;
  }

  angleTuner.setTargetValue(targetContinuousDeg);
  angleTuner.setHysteresis(1.0);
  angleTuner.setOutputRange(0, PWM_MAX);
  angleTuner.setCycles(8);
  angleTuner.setTimeout(3000);
  angleTuner.setTuningMode(pid_tuner::NO_OVERSHOOT);
  angleTuner.start();

  tuningActive = true;

  snprintf(response,
           CommandParser<>::MAX_RESPONSE_SIZE,
           "OK: TUNING STARTED");
}

void cmd_seq(CommandParser<>::Argument *args, char *response) {
  (void)args;

  if (tuningActive) {
    snprintf(response,
             CommandParser<>::MAX_RESPONSE_SIZE,
             "ERR: STOP TUNE FIRST");
    return;
  }

  runMotorPwmSequence();

  snprintf(response,
           CommandParser<>::MAX_RESPONSE_SIZE,
           "OK: SEQUENCE STARTED");
}

void cmd_seq_ramp(CommandParser<>::Argument *args, char *response) {
  (void)args;

  if (tuningActive) {
    snprintf(response,
             CommandParser<>::MAX_RESPONSE_SIZE,
             "ERR: STOP TUNE FIRST");
    return;
  }

  runMotorPwmRampSequence();

  snprintf(response,
           CommandParser<>::MAX_RESPONSE_SIZE,
           "OK: RAMP SEQUENCE STARTED");
}

void cmd_seq_pwm(CommandParser<>::Argument *args, char *response) {
  float pwmValue = (float)args[0].asDouble;
  if (pwmValue < 0.0f) {
    pwmValue = -pwmValue;
  }

  if (pwmValue > (float)PWM_MAX) {
    pwmValue = (float)PWM_MAX;
  }

  sequenceMotorPwm = pwmValue;

  snprintf(response,
           CommandParser<>::MAX_RESPONSE_SIZE,
           "OK: SEQ_PWM=%.1f",
           sequenceMotorPwm);
}

void cmd_pid(CommandParser<>::Argument *args, char *response) {
  int enable = (int)args[0].asDouble;

  if (enable != 0 && enable != 1) {
    snprintf(response,
             CommandParser<>::MAX_RESPONSE_SIZE,
             "ERR: PID expects 0 or 1");
    return;
  }

  pidComputeEnabled = (enable == 1);
  if (!pidComputeEnabled) {
    motorCommandPwm = 0.0f;
    set_motor(0);
  }

  snprintf(response,
           CommandParser<>::MAX_RESPONSE_SIZE,
           "OK: PID=%d",
           pidComputeEnabled ? 1 : 0);
}

void cmd_pid_set(CommandParser<>::Argument *args, char *response) {
  pidKp = (float)args[0].asDouble;
  pidKi = (float)args[1].asDouble;
  pidKd = (float)args[2].asDouble;

  anglePid.setConstants(pidKp, pidKi, pidKd);

  snprintf(response,
           CommandParser<>::MAX_RESPONSE_SIZE,
           "OK: KP=%.3f KI=%.3f KD=%.3f",
           pidKp,
           pidKi,
           pidKd);
}

void cmd_zero(CommandParser<>::Argument *args, char *response) {
  (void)args;

  int rawAngle = ams5600.getRawAngle();
  continuousCounts = 0;
  lastRawAngle = rawAngle;
  encoderInitialized = true;
  continuousDeg = 0.0f;

  snprintf(response,
           CommandParser<>::MAX_RESPONSE_SIZE,
           "OK: CONT_DEG=0.00");
}

void cmd_help(CommandParser<>::Argument *args, char *response) {
  (void)args;
  snprintf(response, CommandParser<>::MAX_RESPONSE_SIZE, "SET <target_deg>, GET, PID <0|1>, PIDSET <kp> <ki> <kd>, ZERO, TUNE, SEQ, SEQRAMP, SEQPWM <pwm>, HELP");
}

void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  char line[64];
  size_t lineLength = Serial.readBytesUntil('\n', line, sizeof(line) - 1);
  line[lineLength] = '\0';
  if ((lineLength > 0) && (line[lineLength - 1] == '\r')) {
    line[lineLength - 1] = '\0';
  }

  if (line[0] == '\0') {
    return;
  }

  char response[CommandParser<>::MAX_RESPONSE_SIZE];
  parser.processCommand(line, response);
  Serial.println(response);
}

void setup() {
  Wire.setClock(400000);

  Serial.begin(115200);
  Serial.println("16 channel PWM test!");

  parser.registerCommand("SET", "d", &cmd_set);
  parser.registerCommand("GET", "", &cmd_get);
  parser.registerCommand("PID", "d", &cmd_pid);
  parser.registerCommand("PIDSET", "ddd", &cmd_pid_set);
  parser.registerCommand("ZERO", "", &cmd_zero);
  parser.registerCommand("TUNE", "", &cmd_tune);
  parser.registerCommand("SEQ", "", &cmd_seq);
  parser.registerCommand("SEQRAMP", "", &cmd_seq_ramp);
  parser.registerCommand("SEQPWM", "d", &cmd_seq_pwm);
  parser.registerCommand("HELP", "", &cmd_help);
  Serial.println("Commands: SET <target_deg>, GET, PID <0|1>, PIDSET <kp> <ki> <kd>, ZERO, TUNE, SEQ, SEQRAMP, SEQPWM <pwm>, HELP");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pwm.setPin(forward_pin, 0);
  pwm.setPin(backward_pin, 0); // Set channel 1 to full on (4095)

  if(ams5600.detectMagnet() == 0 ){
    while(1){
        if(ams5600.detectMagnet() == 1 ){
            Serial.print("Current Magnitude: ");
            Serial.println(ams5600.getMagnitude());
            break;
        }
        else{
            Serial.println("Can not detect magnet");
        }
        delay(1000);
    }
  }

  int rawAngle = ams5600.getRawAngle();
  updateContinuousCount(rawAngle);
  continuousDeg = (continuousCounts * 360.0f) / ENCODER_COUNTS_PER_REV;
  targetContinuousDeg = continuousDeg;

  anglePid.setConstants(pidKp, pidKi, pidKd);
  anglePid.setConstrains(-PWM_MAX, PWM_MAX);
  anglePid.setSetPoint(targetContinuousDeg);
  anglePid.enable();

}

void set_motor(int commandPwm) {
  int speed = abs(commandPwm);
  if (speed > PWM_MAX) {
    speed = PWM_MAX;
  }

  if (commandPwm > PWM_MIN) {
    pwm.setPin(forward_pin, speed, true);
    pwm.setPin(backward_pin, 0, true); // Set channel 1 to full on (4095)
    status = 1;
  } else if (commandPwm < -PWM_MIN) {
    pwm.setPin(forward_pin, 0, true); // Set channel 0 to full on (4095)
    pwm.setPin(backward_pin, speed, true); // Set channel 1 to half on (512)
    status = -1;
  } else {
    pwm.setPin(forward_pin, 0); // Set channel 0 to off (0)
    pwm.setPin(backward_pin, 0); // Set channel 1 to off (0)
    status = 0;
  }
}

void runMotorPwmSequence() {
  sequenceActive = true;
  activeSequenceMode = seqStep;
  sequenceStep = 0;
  sequenceStepStartMs = millis();
  motorCommandPwm = 0.0f;
}

void runMotorPwmRampSequence() {
  sequenceActive = true;
  activeSequenceMode = seqRamp;
  sequenceRampPhase = 0;
  sequenceRampPhaseStartMs = millis();
  motorCommandPwm = 0.0f;
}

void updateMotorPwmRampSequence() {
  unsigned long nowMs = millis();
  const unsigned long rampDurationMs = 2 * SEQUENCE_STEP_MS;

  switch (sequenceRampPhase) {
    case 0: { // ramp up to positive sequenceMotorPwm
      float progress = (float)(nowMs - sequenceRampPhaseStartMs) / (float)rampDurationMs;
      if (progress >= 1.0f) {
        sequenceRampPhase = 1;
        sequenceRampPhaseStartMs = nowMs;
        motorCommandPwm = 0.0f;
      } else {
        motorCommandPwm = progress * sequenceMotorPwm;
      }
      break;
    }

    case 1: { // ramp back down to 0
      float progress = (float)(nowMs - sequenceRampPhaseStartMs) / (float)rampDurationMs;
      if (progress > 1.0f) {
        sequenceRampPhase = 2;
        sequenceRampPhaseStartMs = nowMs;
        motorCommandPwm = 0.0f;
      } else {
        motorCommandPwm = sequenceMotorPwm * (1.0f - progress);
      }
      break;
    }

    case 2: { // hold at 0 for SEQUENCE_STEP_MS
      if (nowMs - sequenceRampPhaseStartMs >= SEQUENCE_STEP_MS) {
        sequenceRampPhase = 3;
        sequenceRampPhaseStartMs = nowMs;
        motorCommandPwm = 0.0f;
      } else {
        motorCommandPwm = 0.0f;
      }
      break;
    }

    case 3: { // ramp down to negative sequenceMotorPwm
      float progress = (float)(nowMs - sequenceRampPhaseStartMs) / (float)rampDurationMs;
      if (progress >= 1.0f) {
        sequenceRampPhase = 4;
        sequenceRampPhaseStartMs = nowMs;
        motorCommandPwm = 0.0f;
      } else {
        motorCommandPwm = -sequenceMotorPwm * progress;
      }
      break;
    }

    case 4: { // ramp back up to 0
      float progress = (float)(nowMs - sequenceRampPhaseStartMs) / (float)rampDurationMs;
      if (progress > 1.0f) {
        sequenceActive = false;
        activeSequenceMode = seqNone;
        motorCommandPwm = 0.0f;
      } else {
        motorCommandPwm = -sequenceMotorPwm * (1.0f - progress);
      }
      break;
    }

    default:
      sequenceActive = false;
      activeSequenceMode = seqNone;
      motorCommandPwm = 0.0f;
      break;
  }
}

void updateMotorPwmSequence() {
  if (!sequenceActive) {
    return;
  }

  if (activeSequenceMode == seqRamp) {
    updateMotorPwmRampSequence();
    return;
  }

  unsigned long nowMs = millis();

  switch (sequenceStep) {
    case 0: // Wait 1 second before forward
      motorCommandPwm = 0.0f;
      if (nowMs - sequenceStepStartMs >= SEQUENCE_STEP_MS) {
        sequenceStep = 1;
        sequenceStepStartMs = nowMs;
      }
      break;

    case 1: // Forward sequence PWM for 1 second
      motorCommandPwm = sequenceMotorPwm;
      if (nowMs - sequenceStepStartMs >= SEQUENCE_STEP_MS) {
        sequenceStep = 2;
        sequenceStepStartMs = nowMs;
      }
      break;

    case 2: // Wait 1 second before reverse
      motorCommandPwm = 0.0f;
      if (nowMs - sequenceStepStartMs >= SEQUENCE_STEP_MS) {
        sequenceStep = 3;
        sequenceStepStartMs = nowMs;
      }
      break;

    case 3: // Reverse sequence PWM for 1 second
      motorCommandPwm = -sequenceMotorPwm;
      if (nowMs - sequenceStepStartMs >= SEQUENCE_STEP_MS) {
        sequenceActive = false;
        motorCommandPwm = 0.0f;
      }
      break;
  }
}

void loop() {

  int rawAngle = ams5600.getRawAngle();

  updateContinuousCount(rawAngle);
  continuousDeg = (continuousCounts * 360.0f) / ENCODER_COUNTS_PER_REV;

  if (sequenceActive) {
    updateMotorPwmSequence();
  }
  else if (tuningActive) {
    motorCommandPwm = (float)angleTuner.update(continuousDeg);

    if (angleTuner.isDone()) {
      double *constants = angleTuner.getConstants();
      pidKp = (float)constants[0];
      pidKi = (float)constants[1];
      pidKd = (float)constants[2];

      anglePid.setConstants(pidKp, pidKi, pidKd);
      anglePid.setSetPoint(targetContinuousDeg);
      anglePid.enable();
      tuningActive = false;

      Serial.print("TUNE DONE KP=");
      Serial.print(pidKp, 4);
      Serial.print(" KI=");
      Serial.print(pidKi, 4);
      Serial.print(" KD=");
      Serial.println(pidKd, 4);
    } else if (angleTuner.hasFailed()) {
      tuningActive = false;
      Serial.println("TUNE FAILED: TIMEOUT");
    }
  } 
  else if (pidComputeEnabled && abs(continuousDeg - targetContinuousDeg) > POSITION_DEADBAND) {
    anglePid.setSetPoint(targetContinuousDeg);
    float rawCommandPwm = (float)anglePid.compute(continuousDeg);
    int s = (rawCommandPwm > 0) - (rawCommandPwm < 0);
    motorCommandPwm = s * map(abs(rawCommandPwm), 0, PWM_MAX, PWM_MIN, PWM_MAX);
  }
  else {
    motorCommandPwm = 0.0f;
    set_motor(0);
  }

  set_motor((int)motorCommandPwm);
  handleSerialCommands();

  Serial.print(continuousDeg, 2);
  Serial.print(",");
  Serial.print(targetContinuousDeg, 2);
  Serial.print(",");
  Serial.println(motorCommandPwm, 1);

  delay(POLL_DELAY_MS);
}