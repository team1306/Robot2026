package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public AngularVelocity shooterTopMotorSpeed = RotationsPerSecond.of(0);
    public AngularVelocity shooterBottomMotorSpeed = RotationsPerSecond.of(0);

    public boolean isShooterTopMotorConnected = false;
    public boolean isShooterBottomMotorConnected = false;

    public Voltage shooterTopMotorAppliedVoltage = Volts.of(0);
    public Voltage shooterBottomMotorAppliedVoltage = Volts.of(0);
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(Voltage voltage) {}

  public default void setVelocity(AngularVelocity velocity) {}
}
