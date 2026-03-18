package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO intakeIO;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setDutyCycle(double dutyCycle) {
    this.intakeIO.setDutyCycle(dutyCycle);
    Logger.recordOutput("Intake/Duty Cycle Setpoint", dutyCycle);
  }

  @Deprecated
  public void setDeployerPosition(Angle angle) {
    intakeIO.setDeployerPosition(angle);
    Logger.recordOutput("Intake/Deployer Position Setpoint", angle);
  }

  @Deprecated
  public void setDeployerPosition(DeployerPosition position) {
    this.setDeployerPosition(position.deployerPosition());
  }

  public void setDeployerDutyCycle(double dutyCycle) {
    Logger.recordOutput("Intake/Deployer Duty Cycle Setpoint", dutyCycle);
    intakeIO.setDeployerDutyCycle(dutyCycle);
  }

  public Command intakeAtDutyCycleCommand(double dutyCycle) {
    return new InstantCommand(() -> this.setDutyCycle(dutyCycle), this);
  }

  @Deprecated
  public Command positionDeployerCommand(Angle angle) {
    return new InstantCommand(() -> this.setDeployerPosition(angle), this);
  }

  @Deprecated
  public Command positionDeployerCommand(DeployerPosition position) {
    return new InstantCommand(() -> this.setDeployerPosition(position), this);
  }

  public Command deployAtDutyCycleCommand(double dutyCycle) {
    return new StartEndCommand(
        () -> this.setDeployerDutyCycle(dutyCycle), () -> this.setDeployerDutyCycle(0));
  }

  public Command deployCommand() {
    return (deployAtDutyCycleCommand(1).withDeadline(Commands.waitSeconds(.5)))
        .andThen(deployAtDutyCycleCommand(.5).withDeadline(Commands.waitSeconds(.25)));
  }

  public Command intakeUntilInterruptedCommand(double dutyCycleWhileOn) {
    return intakeUntilInterruptedCommand(() -> dutyCycleWhileOn);
  }

  public Command intakeUntilInterruptedCommand(DoubleSupplier dutyCycleWhileOn) {
    return Commands.runEnd(
        () -> this.setDutyCycle(dutyCycleWhileOn.getAsDouble()), () -> this.setDutyCycle(0), this);
  }

  public Command shakeIntake() {
    final Time ON_DURATION = Seconds.of(0.75);
    final Time OFF_DURATION = Seconds.of(0.25);
    Timer shakeTimer = new Timer();

    return Commands.runEnd(
            () -> {
              if (!shakeTimer.hasElapsed(ON_DURATION)) {
                setDutyCycle(1);
              } else if (!shakeTimer.hasElapsed(ON_DURATION.plus(OFF_DURATION))) {
                setDutyCycle(0);
              } else {
                shakeTimer.restart();
              }
            },
            () -> setDutyCycle(0),
            this)
        .beforeStarting(() -> shakeTimer.start());
  }

  public Command intakeUntilInterruptedCommand() {
    return this.intakeUntilInterruptedCommand(1);
  }
}
