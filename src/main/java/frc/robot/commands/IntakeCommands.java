package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command positionDeployerCommand(Intake intake, Angle angle) {
    return new InstantCommand(() -> intake.setLatchPosition(angle));
  }

  public static Command intakeAtPower(Intake intake, double power) {
    return new InstantCommand(() -> intake.set(power));
  }
}
