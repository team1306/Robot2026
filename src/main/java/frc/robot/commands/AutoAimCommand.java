package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import badgerutils.triggers.AllianceTriggers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class AutoAimCommand extends ParallelCommandGroup {

  public AutoAimCommand(
      Drive drive, Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    Command lockedAngleCommand =
        DriveCommands.driveAimLockedCommand(
            drive,
            xSupplier,
            ySupplier,
            () -> new Translation2d(getResultantVector(drive, getHubTranslation())),
            true);

    Command shooterSpeedCommand =
        ShooterCommands.shootAtDistanceCommand(
            shooter, () -> Meters.of(getResultantVector(drive, getHubTranslation()).norm()));

    addCommands(lockedAngleCommand, shooterSpeedCommand);
  }

  private static Translation2d getHubTranslation() {
    Translation3d target3d =
        AllianceTriggers.isBlueAlliance()
            ? Constants.Locations.blueHub
            : Constants.Locations.redHub;
    return target3d.toTranslation2d();
  }

  public static Vector<N2> getResultantVector(Drive drive, Translation2d target) {
    ChassisSpeeds chassisSpeeds = drive.getChassisSpeeds();

    Vector<N2> targetVector = target.toVector();
    Vector<N2> velocityVector =
        VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .times(0.02);

    return targetVector.plus(velocityVector);
  }

  private static final int iterations = 1;

  public static Vector<N2> iterateOnVelocity(Vector<N2> velocityVector, Vector<N2> targetVector) {
    double time =
        ShooterCommands.interpolateSetpoints(
                ShooterCommands.SETPOINTS, Meters.of(targetVector.norm()))
            .time()
            .in(Seconds);

    Vector<N2> previousError = VecBuilder.fill(0, 0);
    Vector<N2> endPosition = VecBuilder.fill(0, 0);
    for (int i = 0; i < iterations; ++i) {
      Vector<N2> newTarget = velocityVector.times(time).plus(targetVector).plus(previousError);
      double newDistance = newTarget.norm();

      ShooterCommands.ShooterSetpoint newPoint =
          ShooterCommands.interpolateSetpoints(ShooterCommands.SETPOINTS, Meters.of(newDistance));
      double newTime = newPoint.time().in(Seconds);
      double exitVelocity = newDistance / newTime;
      Vector<N2> realVelocity = newTarget.unit().times(exitVelocity);
      endPosition = realVelocity.plus(velocityVector).times(newTime);
      previousError = endPosition.minus(targetVector);
    }

    return endPosition;
  }
}
