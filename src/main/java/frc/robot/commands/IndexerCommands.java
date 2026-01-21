package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;

public class IndexerCommands extends Command {
  private final Hopper hopper;
  private double targetSpeed;

  public IndexerCommands(Hopper hopper) {
    this.hopper = hopper;

    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.changeSpeed(targetSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void changeTargetSpeed(double speed) {
    targetSpeed = speed;
  }
}
