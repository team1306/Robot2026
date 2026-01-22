package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public final ShooterIO shooterIO;

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
  }

  public void setVelocity(AngularVelocity velocity) {
    shooterIO.setVelocity(velocity);
  }

  public void setVoltage(Voltage voltage) {
    shooterIO.setVoltage(voltage);
  }
}
