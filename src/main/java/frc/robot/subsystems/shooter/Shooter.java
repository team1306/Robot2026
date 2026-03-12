package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public final ShooterIO shooterIO;
  private AngularVelocity speedOverride = RotationsPerSecond.of(0);

  private final SysIdRoutine sysId;

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(this::runCharacterization, null, this));
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setVelocity(AngularVelocity velocity) {
    double multiplier = Math.signum(velocity.in(RotationsPerSecond));

    shooterIO.setVelocity(velocity.plus(speedOverride.times(multiplier)));
    Logger.recordOutput("Shooter/Velocity Setpoint", velocity);
  }

  private void setDutyCycle(double dutyCycle) {
    shooterIO.setDutyCycle(dutyCycle);
    Logger.recordOutput("Shooter/DutyCycle", dutyCycle);
  }

  public void runAtSpeed(AngularVelocity velocity) {
    Logger.recordOutput("Persistant error", velocity.minus(getAvgVelocity()));
    if (velocity.minus(getAvgVelocity()).gt(ShooterConstants.AIM_THRESHOLD)) {
      // if below setpoint, duty cycle = 1; if above setpoint, duty cycle = 0
      setDutyCycle(1);
      // wouldn't normally show the setpoint
      Logger.recordOutput("Shooter/Velocity Setpoint", velocity);
    } else {
      setVelocity(velocity);
      // no longer care about duty cycle
      Logger.recordOutput("Shooter/DutyCycle", 0D);
    }
  }

  public void changeVelocityOverride(AngularVelocity velocity) {
    speedOverride = speedOverride.plus(velocity);
    Logger.recordOutput("Shooter/Velocity Override", velocity);
  }

  public void resetVelocityOverride() {
    speedOverride = RotationsPerSecond.of(0);
    Logger.recordOutput("Shooter/Velocity Override", 0);
  }

  public void setIdle() {
    shooterIO.setIdle();
    Logger.recordOutput("Shooter/Velocity Setpoint", 0.0);
  }

  public void runCharacterization(Voltage output) {
    shooterIO.runCharacterization(output);
  }

  public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.of(0)))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.of(0)))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }

  public AngularVelocity getAvgClosedLoopError() {
    boolean[] connectedMotors =
        new boolean[] {
          inputs.isShooterLeftTopMotorConnected,
          inputs.isShooterLeftBottomMotorConnected,
          inputs.isShooterRightTopMotorConnected,
          inputs.isShooterRightBottomMotorConnected
        };
    AngularVelocity[] error =
        new AngularVelocity[] {
          RotationsPerSecond.of(inputs.shooterLeftTopClosedLoopError),
          RotationsPerSecond.of(inputs.shooterLeftBottomClosedLoopError),
          RotationsPerSecond.of(inputs.shooterRightTopClosedLoopError),
          RotationsPerSecond.of(inputs.shooterRightBottomClosedLoopError)
        };

    AngularVelocity sum = RotationsPerSecond.zero();
    int count = 0;
    for (int i = 0; i < connectedMotors.length; i++) {
      if (connectedMotors[i]) {
        sum = sum.plus(error[i]);
        count++;
      }
    }

    Logger.recordOutput("Shooter/Setpoint Error", sum.div(count));

    return sum.div(count);
  }

  public AngularVelocity getAvgVelocity() {
    boolean[] connectedMotors =
        new boolean[] {
          inputs.isShooterLeftTopMotorConnected,
          inputs.isShooterLeftBottomMotorConnected,
          inputs.isShooterRightTopMotorConnected,
          inputs.isShooterRightBottomMotorConnected
        };
    AngularVelocity[] velocity =
        new AngularVelocity[] {
          inputs.shooterLeftTopMotorSpeed,
          inputs.shooterLeftBottomMotorSpeed,
          inputs.shooterRightTopMotorSpeed,
          inputs.shooterRightBottomMotorSpeed
        };

    AngularVelocity sum = RotationsPerSecond.zero();
    int count = 0;
    for (int i = 0; i < connectedMotors.length; i++) {
      if (connectedMotors[i]) {
        sum = sum.plus(velocity[i]);
        count++;
      }
    }

    Logger.recordOutput("Shooter/Average Speed", sum.div(count));

    return sum.div(count);
  }

  /**
   * Constructs a {@link BooleanSupplier} for the shooter. It is true when the shooter is at the
   * requested velocity, and
   *
   * @return a new boolean supplier
   */
  public BooleanSupplier isAtRequestedSpeed() {
    return () -> getAvgClosedLoopError().lt(ShooterConstants.AIM_THRESHOLD);
  }

  public Trigger isAtRequestedSpeedTrigger() {
    return new Trigger(isAtRequestedSpeed());
  }
}
