package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.RebuiltUtils;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SafeShootCommand extends ParallelCommandGroup {
  private static Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(5);

  private static double INDEXER_SPEED = 0.5;
  private static double INTAKE_SPEED = 0.5;

  private boolean isActive;

  public SafeShootCommand(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      Supplier<Translation2d> positionSupplier,
      BooleanSupplier overrideAllSafeguards,
      BooleanSupplier overrideHubActive) {

    BooleanSupplier shooterVelocityCondition = shooter.isAtRequestedSpeed();

    BooleanSupplier driveAngleCondition =
        () -> drive.isLocked(drive, positionSupplier.get(), true, ANGLE_TOLERANCE);

    BooleanSupplier hubActiveCondition =
        () ->
            RebuiltUtils.isHubActiveOffset(
                    ShooterCommands.interpolateSetpoints(
                            ShooterCommands.SETPOINTS,
                            Meters.of(
                                drive
                                    .getPose()
                                    .getTranslation()
                                    .getDistance(positionSupplier.get())))
                        .time()
                        .in(Seconds))
                || !RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation());
    BooleanSupplier shootCondition =
        () ->
            overrideAllSafeguards.getAsBoolean()
                || (shooterVelocityCondition.getAsBoolean()
                    && driveAngleCondition.getAsBoolean()
                    && (overrideHubActive.getAsBoolean() || hubActiveCondition.getAsBoolean()));
    Command guardedIndexerCommand =
        new GuardedCommand(indexer.indexUntilCancelledCommand(INDEXER_SPEED), shootCondition);

    Command shootAtDistanceCommand =
        ShooterCommands.shootAtDistanceCommand(
                shooter,
                () ->
                    Meters.of(drive.getPose().getTranslation().getDistance(positionSupplier.get())))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    Command intakeCommand = intake.intakeUntilInterruptedCommand(INTAKE_SPEED);

    Trigger intakeRescheduler =
        new Trigger(
                () ->
                    this.isActive
                        && (intake.getCurrentCommand() == null
                            || intake.getCurrentCommand() == intake.getDefaultCommand()))
            .onTrue(intakeCommand);
    Trigger indexerRescheduler =
        new Trigger(
                () ->
                    this.isActive
                        && (indexer.getCurrentCommand() == null
                            || indexer.getCurrentCommand() == indexer.getDefaultCommand()))
            .onTrue(guardedIndexerCommand);

    Command loggedGuardCommand =
        Commands.run(
            () -> logConditions(shooterVelocityCondition, driveAngleCondition, hubActiveCondition));

    Command activityTracker = Commands.startEnd(() -> isActive = true, () -> isActive = false);

    addCommands(
        activityTracker,
        shootAtDistanceCommand,
        guardedIndexerCommand.asProxy(),
        loggedGuardCommand,
        intakeCommand.asProxy());
  }

  private void logConditions(
      BooleanSupplier shooterVelocityCondition,
      BooleanSupplier driveAngleCondition,
      BooleanSupplier hubActiveCondition) {
    Logger.recordOutput(
        "Controls/Ready To Shoot",
        shooterVelocityCondition.getAsBoolean() && driveAngleCondition.getAsBoolean());
    Logger.recordOutput(
        "Controls/Shooter Velocity Condition", shooterVelocityCondition.getAsBoolean());
    Logger.recordOutput("Controls/Drive Angle Condition", driveAngleCondition.getAsBoolean());
    Logger.recordOutput("Controls/HubActive", hubActiveCondition.getAsBoolean());
  }
}
