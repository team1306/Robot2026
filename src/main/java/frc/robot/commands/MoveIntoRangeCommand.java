package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class MoveIntoRangeCommand extends Command {

  private final Drive drive;
  private final Translation2d target;
  private final Distance distance;

  public MoveIntoRangeCommand(Drive drive, Translation2d target, Distance distance) {
    this.drive = drive;
    this.target = target;
    this.distance = distance;
  }

  @Override
  public void execute() {
    drive.runVelocity(new ChassisSpeeds(3, 0, 0));
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return drive.fartherThan(target, distance);
  }
}
