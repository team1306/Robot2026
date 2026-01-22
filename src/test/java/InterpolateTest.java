import static org.junit.jupiter.api.Assertions.*;

import frc.robot.util.Interpolation;
import org.junit.jupiter.api.Test;

public class InterpolateTest {
  private static final double delta = 0.000001;

  private static final InterpolatePoint[] points =
      new InterpolatePoint[] {
        new InterpolatePoint(0, 7),
        new InterpolatePoint(0.25, 20.5),
        new InterpolatePoint(0.5, 34),
        new InterpolatePoint(1, 61),
      };

  private static final double start = 7;
  private static final double end = 61;

  @Test
  public void interpolateTest() {
    for (InterpolatePoint interpolatePoint : points) {
      assertEquals(
          interpolatePoint.out, Interpolation.lerp(start, end, interpolatePoint.time), delta);
    }
  }

  @Test
  public void inverseInterpolateTest() {
    for (InterpolatePoint interpolatePoint : points) {
      assertEquals(
          interpolatePoint.time,
          Interpolation.inverseLerp(start, end, interpolatePoint.out),
          delta);
    }
  }

  private record InterpolatePoint(double time, double out) {}
}
