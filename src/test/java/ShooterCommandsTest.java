import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterCommands.ShooterSetpoint;
import org.junit.jupiter.api.Test;

public class ShooterCommandsTest {
  private static final double delta = 0.000001;

  private static final ShooterSetpoint[] testSetpoints =
      new ShooterSetpoint[] {
        new ShooterSetpoint(Meters.of(0), RotationsPerSecond.of(0), Seconds.of(0)),
        new ShooterSetpoint(Meters.of(5), RotationsPerSecond.of(0.5), Seconds.of(0.5)),
        new ShooterSetpoint(Meters.of(10), RotationsPerSecond.of(1), Seconds.of(1)),
      };

  @Test
  void distanceInterpolationTest() {
    assertEquals(
        0,
        ShooterCommands.interpolateSetpointsForDistance(testSetpoints, Meters.of(-5)).in(RotationsPerSecond),
        delta);
    assertEquals(
        0,
        ShooterCommands.interpolateSetpointsForDistance(testSetpoints, Meters.of(0)).in(RotationsPerSecond),
        delta);

    assertEquals(
        0.25,
        ShooterCommands.interpolateSetpointsForDistance(testSetpoints, Meters.of(2.5)).in(RotationsPerSecond),
        delta);
    assertEquals(
        0.75,
        ShooterCommands.interpolateSetpointsForDistance(testSetpoints, Meters.of(7.5)).in(RotationsPerSecond),
        delta);

    assertEquals(
        1,
        ShooterCommands.interpolateSetpointsForDistance(testSetpoints, Meters.of(10)).in(RotationsPerSecond),
        delta);
    assertEquals(
        1,
        ShooterCommands.interpolateSetpointsForDistance(testSetpoints, Meters.of(15)).in(RotationsPerSecond),
        delta);
  }
}
