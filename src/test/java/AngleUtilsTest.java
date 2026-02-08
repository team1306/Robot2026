import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LocationUtils;
import org.junit.jupiter.api.Test;

public class AngleUtilsTest {
  @Test
  private void testGetDistanceToPosition() {
    // testing linear
    Translation2d p1 = new Translation2d(0, 0);
    Translation2d p2 = new Translation2d(0, 1);
    Distance distance = LocationUtils.getDistanceToLocation(p1, p2);
    assertEquals(1, distance.in(Meters), 1e-9);

    // testing diagonal - 3-4-5 triangle
    p1 = new Translation2d(0, 0);
    p2 = new Translation2d(3.0, 4.0);
    distance = LocationUtils.getDistanceToLocation(p1, p2);
    assertEquals(5.0, distance.in(Meters), 1e-9);

    // testing non-zero start
    p1 = new Translation2d(1.0, 1.0);
    p2 = new Translation2d(4.0, 5.0);
    distance = LocationUtils.getDistanceToLocation(p1, p2);
    assertEquals(5.0, distance.in(Meters), 1e-9);
  }
}
