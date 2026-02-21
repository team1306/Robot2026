package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public void setDeployerPosition(Angle angle) {

    Logger.recordOutput("Intake/Deployer Position Setpoint", angle);
  }

  public void setDeployerPosition(DeployerPosition position) {
    this.setDeployerPosition(position.deployerPosition());
  }

  public Command intakeAtDutyCycleCommand(double dutyCycle) {
    return new InstantCommand(() -> this.setDutyCycle(dutyCycle), this);
  }

  public Command positionDeployerCommand(Angle angle) {
    return new InstantCommand(() -> this.setDeployerPosition(angle), this);
  }

  public Command positionDeployerCommand(DeployerPosition position) {
    return new InstantCommand(() -> this.setDeployerPosition(position), this);
  }

  public Command intakeUntilInterruptedCommand(double dutyCycleWhileOn) {
    return Commands.startEnd(
        () -> this.setDutyCycle(dutyCycleWhileOn), () -> this.setDutyCycle(0), this);
  }

  public Command intakeUntilInterruptedCommand() {
    return this.intakeUntilInterruptedCommand(1);
  }
}
