package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LocationUtils;
import frc.robot.util.RebuiltUtils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMove {
  public static Command shootOnTheMoveCommand(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> target,
      BooleanSupplier override) {
    return new SafeShootCommand(
        drive,
        shooter,
        indexer,
        xSupplier,
        ySupplier,
        () -> calculateLeadTarget(drive),
        override);
  }

  private static Translation2d calculateLeadTarget(Drive drive) {
    Translation2d robotPos = drive.getPose().getTranslation();
    Translation2d targetPos = RebuiltUtils.getCurrentHubLocation().toTranslation2d();

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
