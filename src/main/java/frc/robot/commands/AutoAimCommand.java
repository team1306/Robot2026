package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import badgerutils.triggers.AllianceTriggers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ShooterCommands.ShooterSetpoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AutoAimCommand extends ParallelCommandGroup {

  public AutoAimCommand(
      Drive drive, Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    Command lockedAngleCommand =
        DriveCommands.driveAimLocked(
            drive,
            xSupplier,
            ySupplier,
            () ->
                new Translation3d(findRealPosition(drive, getHubTranslation())).toTranslation2d());

    Command shooterSpeedCommand =
        ShooterCommands.getShootSpeedDistanceRelativeCommand(
            shooter, () -> Meters.of(findRealPosition(drive, getHubTranslation()).norm()));

    addCommands(lockedAngleCommand, shooterSpeedCommand);
  }

  private static final double speedIgnoreThreshold = 0.01;
  private static final int numIterations = 10;
  private static final Distance shooterHeight = Meters.of(0.5);

  private static Vector<N3> findRealPosition(Drive drive, Translation3d target) {
    ChassisSpeeds chassisSpeeds = drive.getChassisSpeeds();

    Vector<N3> velocityVector =
        VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0);

    Vector<N3> targetVector = target.toVector();

    if (velocityVector.norm() < speedIgnoreThreshold) {
      return targetVector;
    }

    Distance iterationDistance =
        Meters.of(target.toTranslation2d().getDistance(drive.getPose().getTranslation()));
    ShooterSetpoint iterationPoint;
    Vector<N3> actualPosition = VecBuilder.fill(0, 0, 0);

    for (int i = 0; i < numIterations; i++) {
      iterationPoint =
          ShooterCommands.interpolateSetpoints(ShooterCommands.SETPOINTS, iterationDistance);

      actualPosition =
          findActualPosition(
              velocityVector, iterationPoint, target.getZ() - shooterHeight.in(Meters));
      Vector<N2> strippedPosition = stripZ(actualPosition);

      iterationDistance = Meters.of(strippedPosition.norm());
    }

    actualPosition = actualPosition.plus(targetVector);

    Logger.recordOutput("Auto Aim/Iteration Distance", iterationDistance);
    Logger.recordOutput(
        "Auto Aim/Actual Position",
        new Pose3d(new Translation3d(actualPosition), Rotation3d.kZero));

    return actualPosition;
  }

  private static Vector<N2> stripZ(Vector<N3> vector) {
    return VecBuilder.fill(vector.get(0), vector.get(1));
  }

  private static Vector<N3> findActualPosition(
      Vector<N3> robotVelocity, ShooterSetpoint initialSetpoint, double height) {
    double time = initialSetpoint.time().in(Seconds);
    double distance = initialSetpoint.distance().in(Meters);

    double top = -4.9 * square(time) - height;
    Vector<N3> initialVelocity = VecBuilder.fill(distance / time, 0, top / time);

    Vector<N3> realVelocity = initialVelocity.plus(robotVelocity.times(0.02));

    return realVelocity.times(time);
  }

  private static double square(double input) {
    return input * input;
  }

  private static Translation3d getHubTranslation() {
    return AllianceTriggers.isBlueAlliance()
        ? Constants.Locations.blueHub
        : Constants.Locations.redHub;
  }
}
