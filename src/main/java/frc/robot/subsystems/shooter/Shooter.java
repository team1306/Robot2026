package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterCommands;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {

  public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public final ShooterIO shooterIO;

  public LoggedNetworkNumber velocityFromDashboard =
      new LoggedNetworkNumber("Shooter/targetVelocity", 0);

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;

    setDefaultCommand(
        ShooterCommands.getShootSpeedCommand(
            this, () -> RotationsPerSecond.of(velocityFromDashboard.getAsDouble())));
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setVelocity(AngularVelocity velocity) {
    shooterIO.setVelocity(velocity);
    Logger.recordOutput("Shooter/Velocity Setpoint", velocity);
  }

  public void setVoltage(Voltage voltage) {
    shooterIO.setVoltage(voltage);
    Logger.recordOutput("Shooter/Voltage Setpoint", voltage);
  }

  public void setDutyCycle(double dutyCycle) {
    shooterIO.setDutyCycle(dutyCycle);
  }
}
