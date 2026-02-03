package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public enum DeployerPosition {
  Retracted(Degrees.of(0)),
  // Replace with actual angle once it is known
  Extended(Degrees.of(0));

  private Angle angle;

  private DeployerPosition(Angle a) {
    this.angle = a;
  }

  public Angle deployerPosition() {
    return angle;
  }
}
