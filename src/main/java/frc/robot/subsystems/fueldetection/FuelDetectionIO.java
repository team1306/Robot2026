package frc.robot.subsystems.fueldetection;

import org.littletonrobotics.junction.AutoLog;

public interface FuelDetectionIO {
  @AutoLog
  public static class FuelDetectionInputs {
    public ObjectTarget[] targets = new ObjectTarget[0];
    public ObjectTarget bestTarget = new ObjectTarget(0, 0, 0);
    public boolean isConnected = false;
  }

  default void updateInputs(FuelDetectionInputs inputs) {}
}
