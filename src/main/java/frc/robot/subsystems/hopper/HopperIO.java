package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean isHopperOpen = false;

    public boolean isHopperSensorConnected = false;
    public boolean isHopperMotorConnected = false;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void set(double speed) {}
}
