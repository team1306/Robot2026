package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface DeployIO {
  @AutoLog
  public static class DeployIOInputs {
    public boolean isDeployerMotorConnected = false;
    public boolean isDeployerEncoderConnected = false;
    public Angle deployerPosition = Degrees.of(0);
    public Temperature deployerTemp = Celsius.of(0);
    public Current deployerSupplyCurrent = Amps.of(0);
    public Current deployerStatorCurrent = Amps.of(0);
    public Angle deployerEncoderPosition = Degrees.of(0);
    public double positionError = 0;
  }

  public default void updateInputs(DeployIOInputs inputs) {}

  public default void setPosition(Angle angle) {}

  public default void setDutyCycle(double dutyCycle) {}

  public default void adjustTarget(Angle angle) {}
}
