package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double currentVelocity;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void changeVelocity(double velocity) {}
}
