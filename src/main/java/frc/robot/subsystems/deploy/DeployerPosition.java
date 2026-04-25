package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public enum DeployerPosition {
  EXTENDED(Rotations.of(0.01)),
  DUMP(Rotations.of(.2)),
  ;
  private Angle angle;

  private DeployerPosition(Angle angle) {
    this.angle = angle;
  }

  public Angle getAngle() {
    return angle;
  }
}
