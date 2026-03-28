package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.RebuiltUtils;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SafeShootCommand extends ParallelCommandGroup {

  private static double INDEXER_SPEED = 1;

  private boolean isActive;

  public SafeShootCommand(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      Leds leds,
      Supplier<Translation2d> positionSupplier,
      Rotation2d angleTolerance,
      BooleanSupplier overrideAngleSafeguard,
      BooleanSupplier overrideVelocitySafeguard,
      BooleanSupplier overrideHubActive) {

    BooleanSupplier shooterVelocityCondition =
        shooter.isAtRequestedSpeed(Constants.Tolerances.NORMAL_SPEED_TOLERANCE);

    BooleanSupplier driveAngleCondition =
        () -> drive.isLocked(drive, positionSupplier.get(), true, angleTolerance);

    BooleanSupplier hubActiveCondition =
        () ->
            RebuiltUtils.isHubActiveOffset(2, 2)
                || !RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation());
    Logger.recordOutput("Controls/Hub Active Condition", hubActiveCondition.getAsBoolean());

    BooleanSupplier combinedCondition =
        () ->
            (shooterVelocityCondition.getAsBoolean() || overrideVelocitySafeguard.getAsBoolean())
                && (driveAngleCondition.getAsBoolean() || overrideAngleSafeguard.getAsBoolean())
                && (hubActiveCondition.getAsBoolean() || overrideHubActive.getAsBoolean());

    Command guardedIndexerCommand =
        new GuardedCommand(
                Commands.waitUntil(
                        () ->
                            shooter
                                    .isAtRequestedSpeed(
                                        Constants.Tolerances.INITIAL_SPEED_TOLERANCE)
                                    .getAsBoolean()
                                || overrideVelocitySafeguard.getAsBoolean())
                    .andThen(indexer.indexUntilCancelledCommand(INDEXER_SPEED)),
                combinedCondition)
            .withName("Guarded Indexer Command");

    Command shootAtDistanceCommand =
        ShooterCommands.shootAtDistanceCommand(
                shooter,
                () ->
                    Meters.of(drive.getPose().getTranslation().getDistance(positionSupplier.get())))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .withName("Shoot At Distance Command");

    Command intakeCommand = intake.shakeIntake().withName("Shake Intake");

    @SuppressWarnings("unused")
    Trigger intakeRescheduler =
        new Trigger(
                () ->
                    this.isActive
                        && (intake.getCurrentCommand() == null
                            || intake.getCurrentCommand() == intake.getDefaultCommand()))
            .onTrue(intakeCommand);
    @SuppressWarnings("unused")
    Trigger indexerRescheduler =
        new Trigger(
                () ->
                    this.isActive
                        && (indexer.getCurrentCommand() == null
                            || indexer.getCurrentCommand() == indexer.getDefaultCommand()))
            .onTrue(guardedIndexerCommand);

    Command guardedLoggingCommand =
        Commands.run(
                () ->
                    logConditions(
                        shooterVelocityCondition, driveAngleCondition, hubActiveCondition))
            .withName("Guarded Logging");

    Command activityTracker =
        Commands.startEnd(() -> isActive = true, () -> isActive = false)
            .withName("Activity Tracker");

    Trigger isShooting = new Trigger(() -> isActive);
    Trigger combinedTrigger = new Trigger(combinedCondition);

    combinedTrigger.whileTrue(
        Commands.runEnd(
            () -> leds.isInShootingTolerance = true, () -> leds.isInShootingTolerance = false));
    isShooting.whileTrue(
        Commands.runEnd(
            () -> leds.isShooting = true,
            () -> {
              leds.isShooting = false;
            }));

    setName("Safe Shoot Command");
    addCommands(
        activityTracker,
        shootAtDistanceCommand,
        guardedIndexerCommand.asProxy(),
        guardedLoggingCommand,
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
