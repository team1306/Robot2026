package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public final ShooterIO shooterIO;

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
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
}
