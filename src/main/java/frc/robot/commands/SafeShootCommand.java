package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(5);
  private static final AngularVelocity INITIAL_SPEED_TOLERANCE = RotationsPerSecond.of(0.5);
  public static final AngularVelocity NORMAL_SPEED_TOLERANCE = RotationsPerSecond.of(2);

  private static double INDEXER_SPEED = 1;

  private boolean isActive;

  public SafeShootCommand(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      Leds leds,
      Supplier<Translation2d> positionSupplier,
      BooleanSupplier overrideAngleSafeguard,
      BooleanSupplier overrideVelocitySafeguard,
      BooleanSupplier overrideHubActive) {

    BooleanSupplier shooterVelocityCondition = shooter.isAtRequestedSpeed(NORMAL_SPEED_TOLERANCE);

    BooleanSupplier driveAngleCondition =
        () -> drive.isLocked(drive, positionSupplier.get(), true, ANGLE_TOLERANCE);

    BooleanSupplier hubActiveCondition =
        () ->
            RebuiltUtils.isHubActive()
                || !RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation());
    Logger.recordOutput("Controls/Hub Active Condition", hubActiveCondition.getAsBoolean());

    BooleanSupplier combinedCondition =
        () ->
            (shooterVelocityCondition.getAsBoolean() || overrideVelocitySafeguard.getAsBoolean())
                && (driveAngleCondition.getAsBoolean() || overrideAngleSafeguard.getAsBoolean())
        /* && (hubActiveCondition.getAsBoolean() || overrideHubActive.getAsBoolean()) */ ;

    Command guardedIndexerCommand =
        new GuardedCommand(
            Commands.waitUntil(shooter.isAtRequestedSpeed(INITIAL_SPEED_TOLERANCE))
                .andThen(indexer.indexUntilCancelledCommand(INDEXER_SPEED)),
            combinedCondition);

    Command shootAtDistanceCommand =
        ShooterCommands.shootAtDistanceCommand(
                shooter,
                () ->
                    Meters.of(drive.getPose().getTranslation().getDistance(positionSupplier.get())))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    Command intakeCommand = intake.shakeIntake();

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

    Command loggedGuardCommand =
        Commands.run(
            () -> logConditions(shooterVelocityCondition, driveAngleCondition, hubActiveCondition));

    Command activityTracker = Commands.startEnd(() -> isActive = true, () -> isActive = false);

    Trigger isShooting = new Trigger(() -> isActive);
    Trigger combinedTrigger = new Trigger(combinedCondition);

    combinedTrigger.whileTrue(
        Commands.runEnd(
            () -> leds.isInShootingTolerance = true, () -> leds.isInShootingTolerance = false));
    isShooting.whileTrue(
        Commands.runEnd(() -> leds.isShooting = true, () -> leds.isShooting = false));
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
