package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean isLeftMotorConnected = false;
    public boolean isRightMotorConnected = false;
    public AngularVelocity leftVelocity = RPM.of(0);
    public AngularVelocity rightVelocity = RPM.of(0);
    public Temperature leftTemp = Celsius.of(0);
    public Temperature rightTemp = Celsius.of(0);
    public Current leftSupplyCurrent = Amps.of(0);
    public Current rightSupplyCurrent = Amps.of(0);
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setDutyCycle(double dutyCycle) {}
}
