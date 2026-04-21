package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public enum DeployerPosition {
  RETRACTED(Rotations.of(.7)),
  EXTENDED(Rotations.of(0.01)),
  DUMP(Rotations.of(.45)),
  ;
  private Angle angle;

  private DeployerPosition(Angle angle) {
    this.angle = angle;
  }

  public Angle getAngle() {
    return angle;
  }
}
