package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LocationUtils;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveAimLockedCommand extends ParallelCommandGroup {

  private final DriveAtAngleCommand driveAtAngleCommand;

  public DriveAimLockedCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> pose,
      boolean isBackward) {

    driveAtAngleCommand =
        new DriveAtAngleCommand(
            drive,
            xSupplier,
            ySupplier,
            () ->
                LocationUtils.getDirectionToLocation(drive.getPose().getTranslation(), pose.get())
                    .plus(isBackward ? Rotation2d.k180deg : Rotation2d.kZero));

    addCommands(driveAtAngleCommand);
  }

  public double getPIDOutput() {
    return driveAtAngleCommand.getPIDOutput();
  }
}
