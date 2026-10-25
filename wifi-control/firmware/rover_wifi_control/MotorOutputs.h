// Same motor outputs as the verified USB firmware.
#pragma once
struct Motor {
  uint8_t enable, in1, in2;
  int polarity, applied = 0;
  bool pausing = false;
  uint32_t resumeAt = 0;
};
Motor leftMotor{LEFT_EN, LEFT_IN1, LEFT_IN2, LEFT_POLARITY};
Motor rightMotor{RIGHT_EN, RIGHT_IN1, RIGHT_IN2, RIGHT_POLARITY};

void motorOff(Motor &motor) {
  if (pwmReady) ledcWrite(motor.enable, 0);
  else digitalWrite(motor.enable, LOW);
  digitalWrite(motor.in1, LOW);
  digitalWrite(motor.in2, LOW);
  motor.applied = 0;
}
void stopMotors() {
  control.stop();
  motorOff(leftMotor); motorOff(rightMotor);
  leftMotor.pausing = rightMotor.pausing = false;
}
bool setMotor(Motor &motor, int target, uint32_t now) {
  if (!target) { motorOff(motor); motor.pausing = false; return true; }
  if (motor.pausing) {
    if (int32_t(now - motor.resumeAt) < 0) return true;
    motor.pausing = false;
  }
  if ((motor.applied > 0 && target < 0) || (motor.applied < 0 && target > 0)) {
    motorOff(motor);
    motor.pausing = true; motor.resumeAt = now + REVERSAL_PAUSE_MS;
    return true;
  }
  int physical = target * motor.polarity;
  digitalWrite(motor.in1, physical > 0 ? HIGH : LOW);
  digitalWrite(motor.in2, physical < 0 ? HIGH : LOW);
  if (!ledcWrite(motor.enable, abs(target))) return false;
  motor.applied = target;
  return true;
}
