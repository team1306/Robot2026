package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private static Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(5);

  private static double INDEXER_SPEED = 0.5;
  private static double INTAKE_SPEED = 0.5;

  public SafeShootCommand(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> positionSupplier,
      BooleanSupplier override) {

    Command shootAtDistanceCommand =
        ShooterCommands.shootAtDistanceCommand(
            shooter,
            () -> Meters.of(drive.getPose().getTranslation().getDistance(positionSupplier.get())));
    Command driveAtAngleCommand =
        DriveCommands.driveAimLockedCommand(drive, xSupplier, ySupplier, positionSupplier, true);
    Command indexerCommand = indexer.indexUntilCancelledCommand(INDEXER_SPEED);
    Command intakeCommand = intake.intakeUntilInterruptedCommand(INTAKE_SPEED);

    BooleanSupplier shooterVelocityCondition = shooter.isAtRequestedSpeed();

    BooleanSupplier driveAngleCondition =
        () -> drive.isLocked(drive, positionSupplier.get(), true, ANGLE_TOLERANCE);

    BooleanSupplier shootCondition =
        () ->
            override.getAsBoolean()
                || (shooterVelocityCondition.getAsBoolean() && driveAngleCondition.getAsBoolean());

    Command guardedIndexerCommand = new GuardedCommand(indexerCommand, shootCondition);

    Command guardedIntakeCommand = new GuardedCommand(intakeCommand, shootCondition);

    Command loggedGuardCommand =
        Commands.run(() -> logConditions(shooterVelocityCondition, driveAngleCondition));

    addCommands(
        shootAtDistanceCommand,
        driveAtAngleCommand,
        guardedIndexerCommand,
        guardedIntakeCommand,
        loggedGuardCommand);
  }

  private void logConditions(
      BooleanSupplier shooterVelocityCondition, BooleanSupplier driveAngleCondition) {
    Logger.recordOutput(
        "Controls/Ready To Shoot",
        shooterVelocityCondition.getAsBoolean() && driveAngleCondition.getAsBoolean());
    Logger.recordOutput(
        "Controls/Shooter Velocity Condition", shooterVelocityCondition.getAsBoolean());
    Logger.recordOutput("Controls/Drive Angle Condition", driveAngleCondition.getAsBoolean());
  }
}
