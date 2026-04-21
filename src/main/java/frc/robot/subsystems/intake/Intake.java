package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  public Command intakeAtDutyCycleCommand(double dutyCycle) {
    return new InstantCommand(() -> this.setDutyCycle(dutyCycle), this);
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
