package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public enum IntakeState {
  ON,
  OFF,
  REVERSE;

  public AngularVelocity targetSpeed() {
    switch (this) {
      case ON:
        return RotationsPerSecond.of(1);
      case REVERSE:
        return RotationsPerSecond.of(-1);
      default:
        return RotationsPerSecond.of(0);
    }
  }
}
