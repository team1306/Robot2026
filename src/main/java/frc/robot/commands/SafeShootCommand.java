package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.booster.Booster;
import frc.robot.subsystems.deploy.Deploy;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LocationUtils;
import frc.robot.util.RebuiltUtils;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SafeShootCommand extends ParallelCommandGroup {

  private static double INDEXER_SPEED = 1;
  private static Time RETRACT_DELAY = Seconds.of(1);
  private static final double BOOSTER_SPEED = 1;
  private static final Distance MINIMUM_SHOT_DISTANCE = Meters.of(2.2);

  private boolean isActive;
  private boolean hasStartedShooting;

  public SafeShootCommand(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      Deploy deploy,
      Booster booster,
      Hood hood,
      Leds leds,
      Supplier<Translation2d> positionSupplier,
      BooleanSupplier isScoring,
      BooleanSupplier overrideAngleSafeguard,
      BooleanSupplier overrideVelocitySafeguard,
      BooleanSupplier overrideHubActive,
      BooleanSupplier overrideAutoRanging,
      BooleanSupplier additionalDeployCondition) {

    Logger.recordOutput(
        "Shooter/Distance to Target",
        LocationUtils.getDistanceToLocation(
            drive.getPose().getTranslation(), positionSupplier.get()));

    BooleanSupplier shooterVelocityCondition =
        shooter.isAtRequestedSpeed(Constants.Tolerances.NORMAL_SPEED_TOLERANCE);

    BooleanSupplier driveAngleCondition =
        () ->
            drive.isLocked(
                drive,
                positionSupplier.get(),
                true,
                isScoring.getAsBoolean()
                    ? Constants.Tolerances.SCORING_ANGLE_TOLERANCE
                    : Constants.Tolerances.PASSING_ANGLE_TOLERANCE);

    BooleanSupplier hubActiveCondition =
        () ->
            RebuiltUtils.isHubActiveOffset(2, 2)
                || !RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation());
    Logger.recordOutput("Controls/Hub Active Condition", hubActiveCondition.getAsBoolean());

    BooleanSupplier autoRangeCondition =
        () -> drive.fartherThan(positionSupplier.get(), MINIMUM_SHOT_DISTANCE);

    BooleanSupplier combinedCondition =
        () ->
            (shooterVelocityCondition.getAsBoolean()
                    || overrideVelocitySafeguard.getAsBoolean())
                && (driveAngleCondition.getAsBoolean() || overrideAngleSafeguard.getAsBoolean())
                && (hubActiveCondition.getAsBoolean() || overrideHubActive.getAsBoolean())
                && (autoRangeCondition.getAsBoolean() || overrideAutoRanging.getAsBoolean());

    Command guardedIndexerCommand =
        new GuardedCommand(
            Commands.waitUntil(
                    () ->
                        shooter
                                .isAtRequestedSpeed(Constants.Tolerances.INITIAL_SPEED_TOLERANCE)
                                .getAsBoolean()
                            || overrideVelocitySafeguard.getAsBoolean())
                .andThen(Commands.runOnce(() -> hasStartedShooting = true))
                .andThen(indexer.indexUntilCancelledCommand(INDEXER_SPEED)),
            combinedCondition);

    Command deployCommand =
            Commands.waitUntil(() -> hasStartedShooting)
            .andThen(
                Commands.waitTime(RETRACT_DELAY)
                    )
            .andThen(
                new GuardedCommand(
                    deploy
                        .crunchCommand()
                        ,
                    additionalDeployCondition));

    Supplier<Distance> distanceSupplier =
        () -> Meters.of(drive.getPose().getTranslation().getDistance(positionSupplier.get()));

    Command shootAtDistanceCommand =
        ShooterCommands.shootAtDistanceCommand(
                shooter,
                distanceSupplier,
                () ->
                    isScoring.getAsBoolean()
                        ? ShooterCommands.HUB_SETPOINTS
                        : ShooterCommands.PASSING_SETPOINTS)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    Command boosterCommand =
        booster
            .boostCommand(BOOSTER_SPEED)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    Command hoodCommand =
        hood.angleFromDistance(
                distanceSupplier,
                () ->
                    isScoring.getAsBoolean()
                        ? ShooterCommands.HUB_SETPOINTS
                        : ShooterCommands.PASSING_SETPOINTS)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    @SuppressWarnings("unused")
    Trigger indexerRescheduler =
        new Trigger(
                () ->
                    this.isActive
                        && (indexer.getCurrentCommand() == null
                            || indexer.getCurrentCommand() == indexer.getDefaultCommand()))
            .onTrue(guardedIndexerCommand);

    Command loggedGuardCommand =
        Commands.run(
            () ->
                logConditions(
                    shooterVelocityCondition,
                    driveAngleCondition,
                    hubActiveCondition,
                    autoRangeCondition, additionalDeployCondition));

    Command activityTracker =
        Commands.startEnd(
            () -> {
              isActive = true;
              hasStartedShooting = false;
            },
            () -> {
              isActive = false;
              guardedIndexerCommand.cancel();
              shootAtDistanceCommand.cancel();
            });

    Trigger isShooting = new Trigger(() -> isActive);
    Trigger combinedTrigger = new Trigger(combinedCondition);

    combinedTrigger.whileTrue(
        Commands.runEnd(
            () -> leds.isInShootingTolerance = true, () -> leds.isInShootingTolerance = false));
    isShooting.whileTrue(
        Commands.runEnd(() -> leds.isShooting = true, () -> leds.isShooting = false));
    addCommands(
        activityTracker,
        shootAtDistanceCommand.asProxy(),
        guardedIndexerCommand.asProxy(),
        deployCommand.asProxy(),
        boosterCommand.asProxy(),
        hoodCommand,
        loggedGuardCommand);
  }

  private void logConditions(
      BooleanSupplier shooterVelocityCondition,
      BooleanSupplier driveAngleCondition,
      BooleanSupplier hubActiveCondition,
      BooleanSupplier withinRangeCondition, BooleanSupplier additionalDeployCondition) {
    Logger.recordOutput(
        "Controls/Ready To Shoot",
        shooterVelocityCondition.getAsBoolean() && driveAngleCondition.getAsBoolean());
    Logger.recordOutput(
        "Controls/Shooter Velocity Condition", shooterVelocityCondition.getAsBoolean());
    Logger.recordOutput("Controls/Drive Angle Condition", driveAngleCondition.getAsBoolean());
    Logger.recordOutput("Controls/HubActive", hubActiveCondition.getAsBoolean());
    Logger.recordOutput("Controls/Within Range Condition", withinRangeCondition.getAsBoolean());
    Logger.recordOutput("Controls/Additional Deploy Condition", additionalDeployCondition.getAsBoolean());
  }
}
