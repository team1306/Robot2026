package unittests;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LocationUtils;
import org.junit.jupiter.api.Test;

public class LocationUtilsTest {
  private static final double EPSILON = 1e-9;

  @Test
  void getDistanceToLocationHandlesCoincidentLinearAndDiagonalPoints() {
    assertDistance(0.0, new Translation2d(2.0, -3.0), new Translation2d(2.0, -3.0));
    assertDistance(1.0, Translation2d.kZero, new Translation2d(0.0, 1.0));
    assertDistance(5.0, Translation2d.kZero, new Translation2d(3.0, 4.0));
    assertDistance(5.0, new Translation2d(1.0, 1.0), new Translation2d(4.0, 5.0));
  }

  @Test
  void distanceIsSymmetric() {
    Translation2d first = new Translation2d(-2.5, 7.0);
    Translation2d second = new Translation2d(4.0, -1.0);

    assertEquals(
        LocationUtils.getDistanceToLocation(first, second).in(Meters),
        LocationUtils.getDistanceToLocation(second, first).in(Meters),
        EPSILON);
  }

  @Test
  void getDirectionToLocationHandlesAllQuadrantsAndNonzeroOrigins() {
    assertDirection(0.0, Translation2d.kZero, new Translation2d(1.0, 0.0));
    assertDirection(45.0, Translation2d.kZero, new Translation2d(1.0, 1.0));
    assertDirection(135.0, Translation2d.kZero, new Translation2d(-1.0, 1.0));
    assertDirection(-135.0, Translation2d.kZero, new Translation2d(-1.0, -1.0));
    assertDirection(-45.0, new Translation2d(2.0, 2.0), new Translation2d(3.0, 1.0));
  }

  private static void assertDistance(
      double expectedMeters, Translation2d first, Translation2d second) {
    Distance distance = LocationUtils.getDistanceToLocation(first, second);
    assertEquals(expectedMeters, distance.in(Meters), EPSILON);
  }

  private static void assertDirection(
      double expectedDegrees, Translation2d start, Translation2d end) {
    Rotation2d direction = LocationUtils.getDirectionToLocation(start, end);
    assertEquals(expectedDegrees, direction.getDegrees(), EPSILON);
  }
}
