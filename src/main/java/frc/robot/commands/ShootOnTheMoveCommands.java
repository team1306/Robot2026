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
import frc.robot.subsystems.deploy.Deploy;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
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
      Leds leds,
      Supplier<Translation2d> target,
      Rotation2d angleTolerance,
      BooleanSupplier overrideAngleSafeguard,
      BooleanSupplier overrideVelocitySafeguard,
      BooleanSupplier overrideHubActive,
      BooleanSupplier overrideAutoRanging) {
    SafeShootCommand shootCommand =
        new SafeShootCommand(
            drive,
            shooter,
            indexer,
            deploy,
            leds,
            () -> calculateLeadTarget(drive, target),
            angleTolerance,
            overrideAngleSafeguard,
            overrideVelocitySafeguard,
            overrideHubActive,
            overrideAutoRanging);

    DriveAimLockedCommand driveCommand =
        new DriveAimLockedCommand(
            drive, () -> 0, () -> 0, () -> calculateLeadTarget(drive, target), true);

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
      Leds leds,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> target,
      Rotation2d angleTolerance,
      BooleanSupplier overrideAngleSafeguard,
      BooleanSupplier overrideVelocitySafeguard,
      BooleanSupplier overrideHubActive,
      BooleanSupplier overrideAutoRanging) {
    return new SafeAimAndShootCommand(
        drive,
        shooter,
        indexer,
        deploy,
        leds,
        () -> xSupplier.getAsDouble() * SLOWDOWN_FACTOR,
        () -> ySupplier.getAsDouble() * SLOWDOWN_FACTOR,
        () -> calculateLeadTarget(drive, target),
        angleTolerance,
        overrideAngleSafeguard,
        overrideVelocitySafeguard,
        overrideHubActive,
        overrideAutoRanging);
  }

  private static Translation2d calculateLeadTarget(Drive drive, Supplier<Translation2d> target) {
    Translation2d robotPos = drive.getPose().getTranslation();
    Translation2d targetPos = target.get();

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
    Translation2d velocityVec =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    Distance distance = LocationUtils.getDistanceToLocation(robotPos, targetPos);
    Time timeOfFlight =
        ShooterCommands.interpolateSetpoints(ShooterCommands.SETPOINTS, distance).time();
    Translation2d aimPoint = new Translation2d();

    for (int i = 0; i < 20; i++) {

      Translation2d motionOffset = velocityVec.times(timeOfFlight.in(Seconds));

      aimPoint = targetPos.minus(motionOffset);

      Distance newDistance = LocationUtils.getDistanceToLocation(aimPoint, robotPos);
      timeOfFlight =
          ShooterCommands.interpolateSetpoints(ShooterCommands.SETPOINTS, newDistance).time();
    }

    Logger.recordOutput("ShootOnTheMove/Target", new Pose2d(aimPoint, Rotation2d.kZero));

    return aimPoint;
  }
}
