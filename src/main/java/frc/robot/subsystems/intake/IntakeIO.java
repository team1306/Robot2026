package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean isLeftMotorConnected = false;
    public boolean isRightMotorConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void set(double speed) {}
}
