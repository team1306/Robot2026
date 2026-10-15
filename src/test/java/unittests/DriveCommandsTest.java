package unittests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.DriveCommands;
import org.junit.jupiter.api.Test;

class DriveCommandsTest {
  private static final double DELTA = 1e-9;

  @Test
  void joystickValuesInsideDeadbandProduceNoVelocity() {
    double insideDeadband = DriveCommands.DEADBAND / 2.0;

    assertTranslation(
        0.0, 0.0, DriveCommands.getLinearVelocityFromJoysticks(insideDeadband, -insideDeadband));
  }

  @Test
  void fullJoystickMagnitudePreservesDirection() {
    assertTranslation(1.0, 0.0, DriveCommands.getLinearVelocityFromJoysticks(1.0, 0.0));
    assertTranslation(-1.0, 0.0, DriveCommands.getLinearVelocityFromJoysticks(-1.0, 0.0));
    assertTranslation(0.6, 0.8, DriveCommands.getLinearVelocityFromJoysticks(0.6, 0.8));
  }

  @Test
  void joystickMagnitudeIsDeadbandAdjustedAndSquared() {
    double inputForHalfMagnitude = DriveCommands.DEADBAND + (1.0 - DriveCommands.DEADBAND) / 2.0;

    assertTranslation(
        0.25, 0.0, DriveCommands.getLinearVelocityFromJoysticks(inputForHalfMagnitude, 0.0));
    assertTranslation(
        0.0, -0.25, DriveCommands.getLinearVelocityFromJoysticks(0.0, -inputForHalfMagnitude));
  }

  private static void assertTranslation(double expectedX, double expectedY, Translation2d actual) {
    assertEquals(expectedX, actual.getX(), DELTA);
    assertEquals(expectedY, actual.getY(), DELTA);
  }
}
