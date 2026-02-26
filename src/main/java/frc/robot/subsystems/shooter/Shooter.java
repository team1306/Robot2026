package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.BangBangController;
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

  private final SysIdRoutine sysId;

  private final BangBangController controller;

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

    controller = new BangBangController(ShooterConstants.ERROR_THRESHOLD);
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

  public void runBangBang(AngularVelocity setpoint) {
    shooterIO.setDutyCycle(
        controller.calculate(
            getAvgVelocity().in(RotationsPerSecond), setpoint.in(RotationsPerSecond)));
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

  public AngularVelocity getAvgVelocity() {
    boolean[] connectedMotors =
        new boolean[] {
          inputs.isShooterLeftTopMotorConnected,
          inputs.isShooterLeftBottomMotorConnected,
          inputs.isShooterRightTopMotorConnected,
          inputs.isShooterRightBottomMotorConnected
        };
    AngularVelocity[] error =
        new AngularVelocity[] {
          inputs.shooterLeftTopMotorSpeed,
          inputs.shooterLeftBottomMotorSpeed,
          inputs.shooterRightTopMotorSpeed,
          inputs.shooterRightBottomMotorSpeed
        };

    AngularVelocity sum = RotationsPerSecond.of(0);
    int count = 0;
    for (int i = 0; i < connectedMotors.length; i++) {
      if (connectedMotors[i]) {
        sum.plus(error[i]);
        count++;
      }
    }

    return sum.div(count);
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
  public BooleanSupplier isAtRequestedSpeed() {
    return () -> getAvgClosedLoopError() < ShooterConstants.ERROR_THRESHOLD;
  }

  public Trigger isAtRequestedSpeedTrigger() {
    return new Trigger(isAtRequestedSpeed());
  }
}
