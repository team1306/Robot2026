package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public AngularVelocity shooterLeftTopMotorSpeed = RotationsPerSecond.of(0);
    public AngularVelocity shooterLeftBottomMotorSpeed = RotationsPerSecond.of(0);
    public AngularVelocity shooterRightTopMotorSpeed = RotationsPerSecond.of(0);
    public AngularVelocity shooterRightBottomMotorSpeed = RotationsPerSecond.of(0);

    public boolean isShooterLeftTopMotorConnected = false;
    public boolean isShooterLeftBottomMotorConnected = false;
    public boolean isShooterRightTopMotorConnected = false;
    public boolean isShooterRightBottomMotorConnected = false;

    public Voltage shooterLeftTopMotorAppliedVoltage = Volts.of(0);
    public Voltage shooterLeftBottomMotorAppliedVoltage = Volts.of(0);

    public Voltage shooterRightTopMotorAppliedVoltage = Volts.of(0);
    public Voltage shooterRightBottomMotorAppliedVoltage = Volts.of(0);

    public Temperature shooterRightTopTemperature = Celsius.of(0);
    public Temperature shooterRightBottomTemperature = Celsius.of(0);
    public Temperature shooterLeftTopTemperature = Celsius.of(0);
    public Temperature shooterLeftBottomTemperature = Celsius.of(0);
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(Voltage voltage) {}

  public default void setVelocity(AngularVelocity velocity) {}

  public default boolean isAtSpeed() {return true;}
}
