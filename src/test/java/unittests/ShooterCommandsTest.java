package unittests;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterCommands.ShooterSetpoint;
import org.junit.jupiter.api.Test;

public class ShooterCommandsTest {
  private static final double DELTA = 0.000001;

  private static final ShooterSetpoint[] testSetpoints =
      new ShooterSetpoint[] {
        new ShooterSetpoint(Meters.of(0), Rotations.of(0), RotationsPerSecond.of(0), Seconds.of(1)),
        new ShooterSetpoint(
            Meters.of(5), Rotations.of(0.5), RotationsPerSecond.of(0.5), Seconds.of(0.5)),
        new ShooterSetpoint(
            Meters.of(10), Rotations.of(1), RotationsPerSecond.of(1), Seconds.of(1)),
      };

  @Test
  void interpolatesEverySetpointField() {
    ShooterSetpoint result = ShooterCommands.interpolateSetpoints(testSetpoints, Meters.of(2.5));

    assertEquals(2.5, result.distance().in(Meters), DELTA);
    assertEquals(0.25, result.hoodAngle().in(Rotations), DELTA);
    assertEquals(0.25, result.velocity().in(RotationsPerSecond), DELTA);
    assertEquals(0.75, result.time().in(Seconds), DELTA);
  }

  @Test
  void clampsDistancesOutsideTheSetpointRange() {
    assertSame(
        testSetpoints[0], ShooterCommands.interpolateSetpoints(testSetpoints, Meters.of(-5.0)));
    assertSame(
        testSetpoints[2], ShooterCommands.interpolateSetpoints(testSetpoints, Meters.of(15.0)));
  }

  @Test
  void returnsZeroSetpointForEmptyInput() {
    ShooterSetpoint result =
        ShooterCommands.interpolateSetpoints(new ShooterSetpoint[0], Meters.of(3.0));

    assertEquals(0.0, result.distance().in(Meters), DELTA);
    assertEquals(0.0, result.hoodAngle().in(Rotations), DELTA);
    assertEquals(0.0, result.velocity().in(RotationsPerSecond), DELTA);
    assertEquals(0.0, result.time().in(Seconds), DELTA);
  }

  @Test
  void setpointsCompareByDistance() {
    assertEquals(-1, Integer.signum(testSetpoints[0].compareTo(testSetpoints[1])));
    assertEquals(0, testSetpoints[1].compareTo(testSetpoints[1]));
    assertEquals(1, Integer.signum(testSetpoints[2].compareTo(testSetpoints[1])));
  }
}
