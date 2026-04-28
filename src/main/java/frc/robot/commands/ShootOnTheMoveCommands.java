package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShooterCommands.ShooterSetpoint;
import frc.robot.subsystems.booster.Booster;
import frc.robot.subsystems.deploy.Deploy;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LocationUtils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMoveCommands {
  private static double SLOWDOWN_FACTOR = 0.75;

  public static Command shootOnTheMoveAutoCommand(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      Deploy deploy,
      Booster booster,
      Hood hood,
      Leds leds,
      Supplier<Translation2d> target,
      BooleanSupplier isScoring,
      BooleanSupplier overrideAngleSafeguard,
      BooleanSupplier overrideVelocitySafeguard,
      BooleanSupplier overrideHubActive,
      BooleanSupplier overrideAutoRanging) {
    Supplier<Translation2d> leadTarget =
        () ->
            calculateLeadTarget(
                drive,
                target,
                () ->
                    isScoring.getAsBoolean()
                        ? ShooterCommands.HUB_SETPOINTS
                        : ShooterCommands.PASSING_SETPOINTS);

    SafeShootCommand shootCommand =
        new SafeShootCommand(
            drive,
            shooter,
            indexer,
            deploy,
            booster,
            hood,
            leds,
            leadTarget,
            isScoring,
            overrideAngleSafeguard,
            overrideVelocitySafeguard,
            overrideHubActive,
            overrideAutoRanging,
            () -> false);

    DriveAimLockedCommand driveCommand =
        new DriveAimLockedCommand(drive, () -> 0, () -> 0, leadTarget, true);

    return shootCommand.alongWith(
        Commands.startEnd(
            () -> {
              driveCommand.resetPID();
              PPHolonomicDriveController.overrideRotationFeedback(
                  () -> driveCommand.getPIDOutput(false));
            },
            () -> PPHolonomicDriveController.clearRotationFeedbackOverride()));
  }

  public static Command aimAndShootOnTheMoveCommand(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      Deploy deploy,
      Booster booster,
      Hood hood,
      Leds leds,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> target,
      BooleanSupplier isScoring,
      BooleanSupplier overrideAngleSafeguard,
      BooleanSupplier overrideVelocitySafeguard,
      BooleanSupplier overrideHubActive,
      BooleanSupplier overrideAutoRanging) {
    return new SafeAimAndShootCommand(
        drive,
        shooter,
        indexer,
        deploy,
        booster,
        hood,
        leds,
        () -> xSupplier.getAsDouble() * SLOWDOWN_FACTOR,
        () -> ySupplier.getAsDouble() * SLOWDOWN_FACTOR,
        () ->
            calculateLeadTarget(
                drive,
                target,
                () ->
                    isScoring.getAsBoolean()
                        ? ShooterCommands.HUB_SETPOINTS
                        : ShooterCommands.PASSING_SETPOINTS),
        isScoring,
        overrideAngleSafeguard,
        overrideVelocitySafeguard,
        overrideHubActive,
        overrideAutoRanging,
        () -> Math.abs(xSupplier.getAsDouble()) < 0.05 && Math.abs(ySupplier.getAsDouble()) < 0.05);
  }

  private static Translation2d calculateLeadTarget(
      Drive drive, Supplier<Translation2d> target, Supplier<ShooterSetpoint[]> setpoints) {
    Translation2d robotPos = drive.getPose().getTranslation();
    Translation2d targetPos = target.get();

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
    Translation2d velocityVec =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    Distance distance = LocationUtils.getDistanceToLocation(robotPos, targetPos);
    Time timeOfFlight = ShooterCommands.interpolateSetpoints(setpoints.get(), distance).time();
    Translation2d aimPoint = new Translation2d();

    for (int i = 0; i < 20; i++) {

      Translation2d motionOffset = velocityVec.times(timeOfFlight.in(Seconds));

      aimPoint = targetPos.minus(motionOffset);

      Distance newDistance = LocationUtils.getDistanceToLocation(aimPoint, robotPos);
      timeOfFlight = ShooterCommands.interpolateSetpoints(setpoints.get(), newDistance).time();
    }

    Logger.recordOutput("ShootOnTheMove/Target", new Pose2d(aimPoint, Rotation2d.kZero));

    return aimPoint;
  }
}
