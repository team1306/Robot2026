package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import badgerutils.networktables.LoggedNetworkTablesBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SafeShootCommand extends ParallelCommandGroup {

  private static final Distance MINIMUM_SHOT_DISTANCE = Feet.of(8);

  public SafeShootCommand(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> positionSupplier,
      BooleanSupplier override) {

    BooleanSupplier autoRangingOverride =
        LoggedNetworkTablesBuilder.createLoggedButton("Controls/Disable Auto Ranging");

    Command shootAtDistanceCommand =
        ShooterCommands.shootAtDistanceCommand(
            shooter,
            () -> Meters.of(drive.getPose().getTranslation().getDistance(positionSupplier.get())));
    Command driveAtAngleCommand =
        DriveCommands.driveAimLockedCommand(drive, xSupplier, ySupplier, positionSupplier, true);
    Command indexerCommand = indexer.indexUntilCancelledCommand(1);
    Command intakeCommand = intake.jumbleIntake();
    Command moveIntoRangeCommand =
        new MoveIntoRangeCommand(drive, positionSupplier.get(), MINIMUM_SHOT_DISTANCE);

    BooleanSupplier shooterVelocityCondition = shooter.isAtRequestedSpeed();
    BooleanSupplier driveAngleCondition = () -> drive.isLocked(drive, positionSupplier.get(), true);
    BooleanSupplier fartherThanCondition =
        () -> drive.fartherThan(positionSupplier.get(), MINIMUM_SHOT_DISTANCE);

    BooleanSupplier shootCondition =
        () ->
            override.getAsBoolean()
                || (shooterVelocityCondition.getAsBoolean()
                    && driveAngleCondition.getAsBoolean()
                    && (fartherThanCondition.getAsBoolean() != autoRangingOverride.getAsBoolean()));

    Command guardedIndexerCommand = new GuardedCommand(indexerCommand, shootCondition);
    Command guardedIntakeCommand = new GuardedCommand(intakeCommand, shootCondition);
    Command guardedRangeCommand =
        new GuardedCommand(
            moveIntoRangeCommand,
            driveAngleCondition,
            () -> !fartherThanCondition.getAsBoolean(),
            () -> !autoRangingOverride.getAsBoolean());

    Command loggedGuardCommand =
        Commands.run(
            () ->
                logConditions(
                    shootCondition,
                    shooterVelocityCondition,
                    driveAngleCondition,
                    fartherThanCondition));

    addCommands(
        shootAtDistanceCommand,
        driveAtAngleCommand,
        guardedIndexerCommand,
        guardedIntakeCommand,
        loggedGuardCommand,
        guardedRangeCommand);
  }

  private void logConditions(
      BooleanSupplier canShootCondition,
      BooleanSupplier shooterVelocityCondition,
      BooleanSupplier driveAngleCondition,
      BooleanSupplier fartherThanCondition) {
    Logger.recordOutput("Controls/Ready To Shoot", canShootCondition.getAsBoolean());
    Logger.recordOutput(
        "Controls/Shooter Velocity Condition", shooterVelocityCondition.getAsBoolean());
    Logger.recordOutput("Controls/Drive Angle Condition", driveAngleCondition.getAsBoolean());
    Logger.recordOutput("Controls/Within Distance Condition", fartherThanCondition.getAsBoolean());
  }
}
