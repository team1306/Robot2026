package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.RebuiltUtils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SafeShootCommand extends ParallelCommandGroup {

  public SafeShootCommand(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> positionSupplier) {

    Command shootAtDistanceCommand =
        ShooterCommands.shootAtDistanceCommand(
            shooter,
            () -> Meters.of(drive.getPose().getTranslation().getDistance(positionSupplier.get())));
    Command driveAtAngleCommand =
        DriveCommands.driveAimLockedCommand(
            drive,
            xSupplier,
            ySupplier,
            () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d());
    Command indexerCommand = indexer.indexUntilCancelledCommand(1);

    BooleanSupplier shooterVelocityCondition = shooter.isAtRequestedSpeed();

    BooleanSupplier driveAngleCondition = () -> drive.isLocked(drive, positionSupplier.get());

    Command guardedIndexerCommand =
        new GuardedCommand(indexerCommand, shooterVelocityCondition, driveAngleCondition);

    addCommands(shootAtDistanceCommand, driveAtAngleCommand, guardedIndexerCommand);
  }
}
