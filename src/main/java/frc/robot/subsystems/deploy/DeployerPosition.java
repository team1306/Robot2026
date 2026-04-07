package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public enum DeployerPosition {
  RETRACTED(Rotations.of(1.34)),
  EXTENDED(Rotations.of(12.26)),
  TEST_POSITION(Rotations.of(6));

  private Angle angle;

  private DeployerPosition(Angle angle) {
    this.angle = angle;
  }

  public Angle getAngle() {
    return angle;
  }
}
