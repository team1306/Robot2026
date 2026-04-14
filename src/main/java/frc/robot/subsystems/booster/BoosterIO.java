package frc.robot.subsystems.booster;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface BoosterIO {
  @AutoLog
  public static class BoosterIOInputs {
    public boolean isMotorConnected = false;
    public AngularVelocity velocity = RPM.of(0);
    public Temperature temp = Celsius.of(0);
    public Current supplyCurrent = Amps.of(0);
  }

  public default void updateInputs(BoosterIOInputs inputs) {}

  public default void setDutyCycle(double dutyCycle) {}
}
