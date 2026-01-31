package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class LocationUtils {
  public static Rotation2d getDirectionToLocation(
      Translation2d startPosition, Translation2d endPosition) {
    Translation2d difference = endPosition.minus(startPosition);
    return difference.getAngle();
  }

  public static Distance getDistanceToLocation(
    Translation2d position1, Translation2d position2) {
      return Meters.of(position1.getDistance(position2));
  }
}
