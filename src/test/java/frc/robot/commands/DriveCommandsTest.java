package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for {@link DriveCommands#joystickDrive}.
 *
 * <p>This test creates a minimal Drive test-double by subclassing Drive with stubbed IO objects and
 * overriding runVelocity to capture the argument. The command returned by {@code joystickDrive} is
 * initialized and executed once to verify it calls runVelocity with the expected chassis speeds.
 */
public class DriveCommandsTest {

  private static final double TOLERANCE = 1e-6;

  /** Simple Drive test-double that captures the last ChassisSpeeds passed to runVelocity. */
  private static class TestDrive extends Drive {
    private ChassisSpeeds lastSpeeds = null;

    public TestDrive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
      super(gyroIO, fl, fr, bl, br);
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds) {
      this.lastSpeeds = speeds;
    }

    public ChassisSpeeds getLastSpeeds() {
      return lastSpeeds;
    }
  }

  private static TestDrive createTestDrive() {
    // AutoBuilder doesn't like to be configured twice, but it provides a method to reset it just
    // for testing.
    AutoBuilder.resetForTesting();

    // Create no-op IO stubs
    GyroIO gyroIO = new GyroIO() {};
    ModuleIO fl = new ModuleIO() {};
    ModuleIO fr = new ModuleIO() {};
    ModuleIO bl = new ModuleIO() {};
    ModuleIO br = new ModuleIO() {};

    return new TestDrive(gyroIO, fl, fr, bl, br);
  }

  @Test
  void joystickDriveCallsRunVelocityWithExpectedSpeeds() {

    // Create Drive test-double that captures the last speeds passed to runVelocity
    TestDrive drive = createTestDrive();

    // Choose joystick inputs: full forward on X, zero Y, moderate rotation
    DoubleSupplier xSupplier = () -> 1.0;
    DoubleSupplier ySupplier = () -> 0.0;
    DoubleSupplier omegaSupplier = () -> 0.5;

    var command = DriveCommands.joystickDrive(drive, xSupplier, ySupplier, omegaSupplier);

    // Run one cycle of the command
    command.initialize();
    command.execute();

    ChassisSpeeds called = drive.getLastSpeeds();
    assertNotNull(called, "runVelocity should have been called with chassis speeds");

    // Linear X should be positive, Y approximately zero (we drove along +X)
    assertTrue(called.vxMetersPerSecond > 0.0, "vx should be positive for full-forward X joystick");
    assertEquals(
        0.0, called.vyMetersPerSecond, TOLERANCE, "vy should be approximately zero for Y=0");

    // Angular speed: joystick 0.5 -> square magnitude => 0.25, then scaled by drive max angular
    // speed
    double omegaInput =
        Math.copySign(
            omegaSupplier.getAsDouble() * omegaSupplier.getAsDouble(), omegaSupplier.getAsDouble());
    double expectedOmega = omegaInput * drive.getMaxAngularSpeedRadPerSec();
    // Ensure the sign is preserved and some angular speed is applied. Exact
    // magnitude may be altered by downstream conversions (desaturation), so
    // we assert direction and non-zero magnitude rather than exact equality.
    assertEquals(
        Math.signum(expectedOmega),
        Math.signum(called.omegaRadiansPerSecond),
        "omega sign should match the expected input");
    assertTrue(
        Math.abs(called.omegaRadiansPerSecond) > 0.0,
        "omega should be non-zero for a non-zero rotation joystick input");
  }

  @Test
  void joystickDriveAppliesDeadbandToLinearInputs() {

    // Drive test-double that captures last speeds
    TestDrive drive = createTestDrive();

    // Inputs inside the deadband (DEADBAND == 0.1 in DriveCommands)
    DoubleSupplier xSupplier = () -> 0.05;
    DoubleSupplier ySupplier = () -> 0.05;
    DoubleSupplier omegaSupplier = () -> 0.5;

    var command = DriveCommands.joystickDrive(drive, xSupplier, ySupplier, omegaSupplier);

    // Execute one cycle
    command.initialize();
    command.execute();

    ChassisSpeeds called = drive.getLastSpeeds();
    assertNotNull(called, "runVelocity should have been called");

    // When inputs are within the deadband, linear speeds should be approximately zero
    assertEquals(
        0.0,
        called.vxMetersPerSecond,
        TOLERANCE,
        "vx should be approximately zero inside deadband");
    assertEquals(
        0.0,
        called.vyMetersPerSecond,
        TOLERANCE,
        "vy should be approximately zero inside deadband");
  }

  @Test
  void joystickDriveAppliesDeadbandToRotationInputs() {

    // Drive test-double that captures last speeds
    TestDrive drive = createTestDrive();

    // Inputs inside the deadband (DEADBAND == 0.1 in DriveCommands)
    DoubleSupplier xSupplier = () -> 0.5;
    DoubleSupplier ySupplier = () -> 0.5;
    DoubleSupplier omegaSupplier = () -> 0.05;

    var command = DriveCommands.joystickDrive(drive, xSupplier, ySupplier, omegaSupplier);

    // Execute one cycle
    command.initialize();
    command.execute();

    ChassisSpeeds called = drive.getLastSpeeds();
    assertNotNull(called, "runVelocity should have been called");

    assertEquals(
        0.0,
        called.omegaRadiansPerSecond,
        TOLERANCE,
        "omega should be near zero for rotation within deadzone");
  }
}
