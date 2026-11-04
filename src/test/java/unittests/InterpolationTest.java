package unittests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.util.Interpolation;
import org.junit.jupiter.api.Test;

class InterpolationTest {
  private static final double DELTA = 1e-9;

  @Test
  void lerpInterpolatesAndExtrapolates() {
    assertEquals(10.0, Interpolation.lerp(10.0, 20.0, 0.0), DELTA);
    assertEquals(15.0, Interpolation.lerp(10.0, 20.0, 0.5), DELTA);
    assertEquals(20.0, Interpolation.lerp(10.0, 20.0, 1.0), DELTA);
    assertEquals(5.0, Interpolation.lerp(10.0, 20.0, -0.5), DELTA);
    assertEquals(25.0, Interpolation.lerp(10.0, 20.0, 1.5), DELTA);
  }

  @Test
  void inverseLerpHandlesAscendingAndDescendingRanges() {
    assertEquals(0.25, Interpolation.inverseLerp(10.0, 20.0, 12.5), DELTA);
    assertEquals(0.25, Interpolation.inverseLerp(20.0, 10.0, 17.5), DELTA);
    assertEquals(-0.5, Interpolation.inverseLerp(10.0, 20.0, 5.0), DELTA);
  }
}
