package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean isConnected = false;
    public AngularVelocity velocity = RPM.of(0);
    public Temperature temp = Celsius.of(0);
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setPower(double velocity) {}
}
