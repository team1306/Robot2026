package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean isLeftMotorConnected = false;
    public boolean isRightMotorConnected = false;
    public boolean isDeployerMotorConnected = false;
    public AngularVelocity leftVelocity = RPM.of(0);
    public AngularVelocity rightVelocity = RPM.of(0);
    public Angle deployerPosition = Degrees.of(0);
    public Temperature leftTemp = Celsius.of(0);
    public Temperature rightTemp = Celsius.of(0);
    public Temperature deployerTemp = Celsius.of(0);
    public Current leftSupplyCurrent = Amps.of(0);
    public Current rightSupplyCurrent = Amps.of(0);
    public Current deployerSupplyCurrent = Amps.of(0);
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setDutyCycle(double dutyCycle) {}

  public default void setDeployerPosition(Angle angle) {}

  public default void setDeployerDutyCycle(double dutyCycle) {}
}
