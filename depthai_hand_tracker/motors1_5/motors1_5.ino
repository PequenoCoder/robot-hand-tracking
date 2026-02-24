#include <SimpleFOC.h>

// ===== Motor 1 =====
MagneticSensorPWM sensor1 = MagneticSensorPWM(15, 3, 941);
void doPWM1() { sensor1.handlePWM(); }
BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(1, 2, 3, 4); // (A,B,C,EN)

// ===== Motor 2 =====
MagneticSensorPWM sensor2 = MagneticSensorPWM(14, 3, 941);
void doPWM2() { sensor2.handlePWM(); }
BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(6, 7, 8, 9); // (A,B,C,EN)

// ===== Motor 3 =====
MagneticSensorPWM sensor3 = MagneticSensorPWM(16, 3, 941);
void doPWM3() { sensor3.handlePWM(); }
BLDCMotor motor3 = BLDCMotor(11);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(24, 25, 28, 10); // (A,B,C,EN)

// ===== Motor 4 =====
MagneticSensorPWM sensor4 = MagneticSensorPWM(17, 3, 941);
void doPWM4() { sensor4.handlePWM(); }
BLDCMotor motor4 = BLDCMotor(11);
BLDCDriver3PWM driver4 = BLDCDriver3PWM(24, 25, 28, 29); // (A,B,C,EN)

// ===== Motor 5 =====
MagneticSensorPWM sensor5 = MagneticSensorPWM(18, 3, 941);
void doPWM5() { sensor5.handlePWM(); }
BLDCMotor motor5 = BLDCMotor(11);
BLDCDriver3PWM driver5 = BLDCDriver3PWM(33, 36, 37, 13); // (A,B,C,EN)


// ===== Control variables =====
// Target angles (from serial commands)
float target_angle_rad1 = 0.0f;
float target_angle_rad2 = 0.0f;
float target_angle_rad3 = 0.0f;
float target_angle_rad4 = 0.0f;
float target_angle_rad5 = 0.0f;

// Smoothed angles (interpolated)
float commanded_angle_rad1 = 0.0f;
float commanded_angle_rad2 = 0.0f;
float commanded_angle_rad3 = 0.0f;
float commanded_angle_rad4 = 0.0f;
float commanded_angle_rad5 = 0.0f;

float angle_offset_rad1 = 0.0f;
float angle_offset_rad2 = 0.0f;
float angle_offset_rad3 = 0.0f;
float angle_offset_rad4 = 0.0f;
float angle_offset_rad5 = 0.0f;

// ===== Smoothing Configuration =====
// Exponential smoothing: commanded = commanded + ALPHA * (target - commanded)
// SMOOTHING_ALPHA values:
//   0.02 = Ultra smooth with many steps (~150 cycles) - SMOOTHEST
//   0.03 = Very smooth with many steps (~100 cycles)
//   0.05 = Smooth (takes ~60 cycles)
//   0.10 = Moderate (takes ~30 cycles) - BALANCED
//   0.15 = Fast (takes ~20 cycles)
//   0.25 = Very fast (takes ~12 cycles)
//   1.00 = Instant (no smoothing)
// Note: The FOC loop runs at ~1000+ Hz, so even 0.18 is very responsive!
const float SMOOTHING_ALPHA = 0.18f;  // Fast + smooth response (takes ~16 cycles to reach 95%)

bool need_prompt = true;
int prompt_motor = 1;

// ===== Safety limits (degrees) =====
const float ANGLE_MIN_DEG = -80.0f;  // Can go -80 degrees
const float ANGLE_MAX_DEG = 0.0f;    // Can never go positive

// Clamp angle to safe range
float clampAngle(float angle_deg) {
  if (angle_deg > ANGLE_MAX_DEG) return ANGLE_MAX_DEG;
  if (angle_deg < ANGLE_MIN_DEG) return ANGLE_MIN_DEG;
  return angle_deg;
}

String readLineOnce() {
  if (!Serial.available()) return String();
  String s = Serial.readStringUntil('\n'); s.trim(); return s;
}

void setupMotor(BLDCMotor& motor, BLDCDriver3PWM& driver, MagneticSensorPWM& sensor) {
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller     = MotionControlType::angle;

  motor.PID_velocity.P  = 0.5;
  motor.PID_velocity.I  = 2.0f;
  motor.PID_velocity.D  = 0.001f;
  motor.LPF_velocity.Tf = 0.1f;

  motor.P_angle.P       = 20.0f;
  motor.voltage_limit   = 6.0f;
  motor.velocity_limit  = 10.0f;

  motor.init();
  motor.initFOC();
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  analogReadResolution(12);

  // Attach PWM sensor interrupts
  attachInterrupt(digitalPinToInterrupt(15), doPWM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(14), doPWM2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(16), doPWM3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(17), doPWM4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), doPWM5, CHANGE);

  // Init motors
  setupMotor(motor1, driver1, sensor1);
  setupMotor(motor2, driver2, sensor2);
  setupMotor(motor3, driver3, sensor3);
  setupMotor(motor4, driver4, sensor4);
  setupMotor(motor5, driver5, sensor5);

  // Zero at current positions (AFTER initFOC completes)
  angle_offset_rad1 = motor1.shaft_angle;
  angle_offset_rad2 = motor2.shaft_angle;
  angle_offset_rad3 = motor3.shaft_angle;
  angle_offset_rad4 = motor4.shaft_angle;
  angle_offset_rad5 = motor5.shaft_angle;

  target_angle_rad1 = angle_offset_rad1;
  target_angle_rad2 = angle_offset_rad2;
  target_angle_rad3 = angle_offset_rad3;
  target_angle_rad4 = angle_offset_rad4;
  target_angle_rad5 = angle_offset_rad5;

  commanded_angle_rad1 = angle_offset_rad1;
  commanded_angle_rad2 = angle_offset_rad2;
  commanded_angle_rad3 = angle_offset_rad3;
  commanded_angle_rad4 = angle_offset_rad4;
  commanded_angle_rad5 = angle_offset_rad5;

  motor1.target = commanded_angle_rad1;
  motor2.target = commanded_angle_rad2;
  motor3.target = commanded_angle_rad3;
  motor4.target = commanded_angle_rad4;
  motor5.target = commanded_angle_rad5;

  need_prompt = true;
  prompt_motor = 1;

  Serial.println(F("Commands:"));
  Serial.println(F("  - Rotating prompt (enter a single angle for M1 -> M2 -> M3 -> M4 -> M5)"));
  Serial.println(F("  - CSV for all five (deg): d1,d2,d3,d4,d5   e.g., 0,-20,-40,-60,-80"));
  Serial.println(F(""));
  Serial.println(F("SAFETY LIMITS: All angles clamped to [-80° to 0°]"));
  Serial.println(F("  - Motors can NEVER go positive (max 0°)"));
  Serial.println(F("  - Motors can go negative up to -80° (min -80°)"));
}

void loop() {
  // FOC update
  motor1.loopFOC(); motor2.loopFOC(); motor3.loopFOC(); motor4.loopFOC(); motor5.loopFOC();

  // Smooth interpolation (exponential smoothing)
  commanded_angle_rad1 += SMOOTHING_ALPHA * (target_angle_rad1 - commanded_angle_rad1);
  commanded_angle_rad2 += SMOOTHING_ALPHA * (target_angle_rad2 - commanded_angle_rad2);
  commanded_angle_rad3 += SMOOTHING_ALPHA * (target_angle_rad3 - commanded_angle_rad3);
  commanded_angle_rad4 += SMOOTHING_ALPHA * (target_angle_rad4 - commanded_angle_rad4);
  commanded_angle_rad5 += SMOOTHING_ALPHA * (target_angle_rad5 - commanded_angle_rad5);

  // Move to smoothed commanded angles
  motor1.move(commanded_angle_rad1);
  motor2.move(commanded_angle_rad2);
  motor3.move(commanded_angle_rad3);
  motor4.move(commanded_angle_rad4);
  motor5.move(2 * angle_offset_rad5 - commanded_angle_rad5); // Motor 5 reversed

  // Print angles every 1s
  static uint32_t lastPrint_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms - lastPrint_ms >= 1000) {
    lastPrint_ms = now_ms;

    float cur_deg1 = (motor1.shaft_angle - angle_offset_rad1) * (180.0f / PI);
    float cmd_deg1 = (commanded_angle_rad1 - angle_offset_rad1) * (180.0f / PI);
    float cur_deg2 = (motor2.shaft_angle - angle_offset_rad2) * (180.0f / PI);
    float cmd_deg2 = (commanded_angle_rad2 - angle_offset_rad2) * (180.0f / PI);
    float cur_deg3 = (motor3.shaft_angle - angle_offset_rad3) * (180.0f / PI);
    float cmd_deg3 = (commanded_angle_rad3 - angle_offset_rad3) * (180.0f / PI);
    float cur_deg4 = (motor4.shaft_angle - angle_offset_rad4) * (180.0f / PI);
    float cmd_deg4 = (commanded_angle_rad4 - angle_offset_rad4) * (180.0f / PI);
    float cur_deg5 = (motor5.shaft_angle - angle_offset_rad5) * (180.0f / PI);
    float cmd_deg5 = (commanded_angle_rad5 - angle_offset_rad5) * (180.0f / PI);

    Serial.print("M1: "); Serial.print(cur_deg1, 1); Serial.print("° (Cmd "); Serial.print(cmd_deg1, 1); Serial.print("°) | ");
    Serial.print("M2: "); Serial.print(cur_deg2, 1); Serial.print("° (Cmd "); Serial.print(cmd_deg2, 1); Serial.print("°) | ");
    Serial.print("M3: "); Serial.print(cur_deg3, 1); Serial.print("° (Cmd "); Serial.print(cmd_deg3, 1); Serial.print("°) | ");
    Serial.print("M4: "); Serial.print(cur_deg4, 1); Serial.print("° (Cmd "); Serial.print(cmd_deg4, 1); Serial.print("°) | ");
    Serial.print("M5: "); Serial.print(cur_deg5, 1); Serial.print("° (Cmd "); Serial.print(cmd_deg5, 1); Serial.println("°)");
  }

  // Prompt
  if (need_prompt) {
    Serial.print("Enter M");
    Serial.print(prompt_motor);
    Serial.println(" angle (deg) OR CSV d1,d2,d3,d4,d5 :");
    need_prompt = false;
  }

  // Handle input
  String line = readLineOnce();
  if (line.length()) {
    // CSV?
    if (line.indexOf(',') >= 0) {
      float d1, d2, d3, d4, d5;
      int n = sscanf(line.c_str(), "%f,%f,%f,%f,%f", &d1, &d2, &d3, &d4, &d5);
      if (n == 5) {
        // Apply safety limits
        d1 = clampAngle(d1);
        d2 = clampAngle(d2);
        d3 = clampAngle(d3);
        d4 = clampAngle(d4);
        d5 = clampAngle(d5);
        
        // Set target angles (will be smoothly interpolated in loop)
        target_angle_rad1 = d1 * (PI / 180.0f) + angle_offset_rad1;
        target_angle_rad2 = d2 * (PI / 180.0f) + angle_offset_rad2;
        target_angle_rad3 = d3 * (PI / 180.0f) + angle_offset_rad3;
        target_angle_rad4 = d4 * (PI / 180.0f) + angle_offset_rad4;
        target_angle_rad5 = d5 * (PI / 180.0f) + angle_offset_rad5;
        
        Serial.print(F("OK: CSV set (clamped): "));
        Serial.print(d1); Serial.print(",");
        Serial.print(d2); Serial.print(",");
        Serial.print(d3); Serial.print(",");
        Serial.print(d4); Serial.print(",");
        Serial.println(d5);
      } else {
        Serial.println(F("ERR: CSV format is d1,d2,d3,d4,d5  e.g., 0,-5,-10,-15,-20"));
      }
      need_prompt = true;
      return;
    }

    // Single-angle rotating prompt
    float angle_deg = line.toFloat();
    // Apply safety limits
    angle_deg = clampAngle(angle_deg);
    float angle_rad = angle_deg * (PI / 180.0f);
    
    if (prompt_motor == 1) {
      target_angle_rad1 = angle_rad + angle_offset_rad1; prompt_motor = 2;
    } else if (prompt_motor == 2) {
      target_angle_rad2 = angle_rad + angle_offset_rad2; prompt_motor = 3;
    } else if (prompt_motor == 3) {
      target_angle_rad3 = angle_rad + angle_offset_rad3; prompt_motor = 4;
    } else if (prompt_motor == 4) {
      target_angle_rad4 = angle_rad + angle_offset_rad4; prompt_motor = 5;
    } else {
      target_angle_rad5 = angle_rad + angle_offset_rad5; prompt_motor = 1;
    }
    
    Serial.print(F("OK: M"));
    Serial.print(prompt_motor == 1 ? 5 : prompt_motor - 1);
    Serial.print(F(" set to "));
    Serial.print(angle_deg);
    Serial.println(F("° (clamped if needed)"));
    
    need_prompt = true;
  }
}
