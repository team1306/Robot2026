package frc.robot.subsystems.latch;

import org.littletonrobotics.junction.AutoLog;

public interface LatchIO {
  @AutoLog
  public static class LatchIOInputs {
    public boolean isMotorConnected = false;
  }

  public default void updateInputs(LatchIOInputs inputs) {}

  public default void extend() {}
}
