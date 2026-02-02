package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
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

  public void setIdle() {
    shooterIO.setIdle();
    Logger.recordOutput("Shooter/Velocity Setpoint", 0);
  }

  public double getAvgClosedLoopError() {
    boolean[] connectedMotors =
        new boolean[] {
          inputs.isShooterLeftTopMotorConnected,
          inputs.isShooterLeftBottomMotorConnected,
          inputs.isShooterRightTopMotorConnected,
          inputs.isShooterRightBottomMotorConnected
        };
    double[] error =
        new double[] {
          inputs.shooterLeftTopClosedLoopError,
          inputs.shooterLeftBottomClosedLoopError,
          inputs.shooterRightTopClosedLoopError,
          inputs.shooterRightBottomClosedLoopError
        };

    double sum = 0;
    int count = 0;
    for (int i = 0; i < connectedMotors.length; i++) {
      if (connectedMotors[i]) {
        sum += error[i];
        count++;
      }
    }

    return sum / count;
  }

  /**
   * Constructs a {@link BooleanSupplier} for the shooter. It is true when the shooter is at the
   * requested velocity, and
   *
   * @return a new boolean supplier
   */
  public boolean isAtRequestedSpeed() {
    return getAvgClosedLoopError() < ShooterConstants.ERROR_THRESHOLD;
  }

  public Trigger isAtRequestedSpeedTrigger() {
    return new Trigger(() -> isAtRequestedSpeed());
  }
}
