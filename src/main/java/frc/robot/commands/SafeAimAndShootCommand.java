package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SafeAimAndShootCommand extends ParallelCommandGroup {

  private final DriveAimLockedCommand driveCommand;

  public SafeAimAndShootCommand(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      Hood hood,
      Leds leds,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> positionSupplier,
      Rotation2d angleTolerance,
      BooleanSupplier overrideAngleSafeguard,
      BooleanSupplier overrideVelocitySafeguard,
      BooleanSupplier overrideHubActive,
      BooleanSupplier overrideAutoRanging) {

    Command safeShootCommand =
        new SafeShootCommand(
            drive,
            shooter,
            indexer,
            intake,
            hood,
            leds,
            positionSupplier,
            angleTolerance,
            overrideAngleSafeguard,
            overrideVelocitySafeguard,
            overrideHubActive,
            overrideAutoRanging);

    driveCommand = new DriveAimLockedCommand(drive, xSupplier, ySupplier, positionSupplier, true);

    addCommands(safeShootCommand, driveCommand);
  }

  public double getPIDOutput(boolean flipped) {
    return driveCommand.getPIDOutput(flipped);
  }
}
