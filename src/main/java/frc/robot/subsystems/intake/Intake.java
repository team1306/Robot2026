package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
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

  public void setPower(double dutyCycle) {
    this.intakeIO.set(dutyCycle);
    Logger.recordOutput("Intake/Duty Cycle", dutyCycle);
  }

  public void setDeployerPosition(Angle angle) {
    intakeIO.setDeployerPosition(angle);
    Logger.recordOutput("Intake/Deployer Position", angle);
  }

  public Command intakePowerCommand(double dutyCycle) {
    return new InstantCommand(() -> this.setPower(dutyCycle));
  }

  public Command deployerPositionCommand(Angle angle) {
    return new InstantCommand(() -> this.setDeployerPosition(angle));
  }
}
