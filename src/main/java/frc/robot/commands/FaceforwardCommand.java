package frc.robot.commands;

import static frc.robot.commands.DriveCommands.DEADBAND;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class FaceforwardCommand extends Command {

  private final Command driveAtAngleCommand;

  private Rotation2d lastRotation = Rotation2d.fromDegrees(0);

  public FaceforwardCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    driveAtAngleCommand =
        new DriveAtAngleCommand(
            drive,
            xSupplier,
            ySupplier,
            () -> {
              double x = xSupplier.getAsDouble();
              double y = ySupplier.getAsDouble();
              if (Math.abs(x) < DEADBAND) {
                x = 0;
              }
              if (Math.abs(y) < DEADBAND) {
                y = 0;
              }

              if (x == 0 && y == 0) {
                return lastRotation;
              }

              lastRotation =
                  Rotation2d.fromRadians(
                      Math.atan2(y, x)); // may need to add PI, depending on orientation

              return lastRotation;
            });
  }

  @Override
  public void initialize() {
    driveAtAngleCommand.initialize();
  }

  @Override
  public void execute() {
    driveAtAngleCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    driveAtAngleCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return driveAtAngleCommand.isFinished();
  }
}
