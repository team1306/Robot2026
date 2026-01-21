package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public AngularVelocity leftMotorAngularVelocity = RotationsPerSecond.of(0);
    public AngularVelocity rightMotorAngularVelocity = RotationsPerSecond.of(0);
    public boolean isLeftMotorConnected = false;
    public boolean isRightMotorConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void set(IntakeState state) {}
}
