package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public AngularVelocity shooterLeftTopMotorSpeed = RotationsPerSecond.of(0);
    public AngularVelocity shooterLeftBottomMotorSpeed = RotationsPerSecond.of(0);
    public AngularVelocity shooterRightTopMotorSpeed = RotationsPerSecond.of(0);
    public AngularVelocity shooterRightBottomMotorSpeed = RotationsPerSecond.of(0);
    public AngularVelocity encoderSpeed = RotationsPerSecond.of(0);

    public boolean isShooterLeftTopMotorConnected = false;
    public boolean isShooterLeftBottomMotorConnected = false;
    public boolean isShooterRightTopMotorConnected = false;
    public boolean isShooterRightBottomMotorConnected = false;
    public boolean isEncoderConnected = false;

    public Current shooterLeftTopMotorSupplyCurrent = Amps.of(0);
    public Current shooterLeftBottomMotorSupplyCurrent = Amps.of(0);
    public Current shooterRightTopMotorSupplyCurrent = Amps.of(0);
    public Current shooterRightBottomMotorSupplyCurrent = Amps.of(0);

    public Temperature shooterRightTopTemperature = Celsius.of(0);
    public Temperature shooterRightBottomTemperature = Celsius.of(0);
    public Temperature shooterLeftTopTemperature = Celsius.of(0);
    public Temperature shooterLeftBottomTemperature = Celsius.of(0);

    public double shooterLeftTopClosedLoopError = 0;
    public double shooterLeftBottomClosedLoopError = 0;
    public double shooterRightTopClosedLoopError = 0;
    public double shooterRightBottomClosedLoopError = 0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVelocity(AngularVelocity velocity) {}

  default void setIdle() {}

  default void runCharacterization(Voltage voltage) {}
  ;
}
