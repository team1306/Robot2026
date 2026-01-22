import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.util.AngleUtils;
import org.junit.jupiter.api.Test;

public class AutoAimTest {

  static class AutoAimDrive extends Drive {

    private ChassisSpeeds speeds = new ChassisSpeeds();

    public AutoAimDrive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO) {
      super(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
      return speeds;
    }

    public void setInputSpeeds(ChassisSpeeds inputSpeeds) {
      speeds = inputSpeeds;
    }
  }

  private static AutoAimDrive createTestDrive() {
    // AutoBuilder doesn't like to be configured twice, but it provides a method to reset it just
    // for testing.
    AutoBuilder.resetForTesting();

    // Create no-op IO stubs
    GyroIO gyroIO = new GyroIO() {};
    ModuleIO fl = new ModuleIO() {};
    ModuleIO fr = new ModuleIO() {};
    ModuleIO bl = new ModuleIO() {};
    ModuleIO br = new ModuleIO() {};

    return new AutoAimDrive(gyroIO, fl, fr, bl, br);
  }
  /*
    @Test
    void validateResultantVector() {
      AutoAimDrive testDrive = createTestDrive();

      testDrive.setInputSpeeds(new ChassisSpeeds(0, 0, 0));

      Vector<N2> vector = AutoAimCommand.getResultantVector(testDrive, new Translation2d(0, 1));
      assertEquals(VecBuilder.fill(0, 1), vector);

      testDrive.setInputSpeeds(new ChassisSpeeds(4, 0, 0));

      vector = AutoAimCommand.getResultantVector(testDrive, new Translation2d(0, 1));
      assertEquals(VecBuilder.fill(-4, 1), vector);

      testDrive.setInputSpeeds(new ChassisSpeeds(-1, -1, 0));

    vector = AutoAimCommand.getResultantVector(testDrive, new Translation2d(3, 1));
    assertEquals(VecBuilder.fill(4, 2), vector);
  }

  @Test
  void validateAngle() {
    Rotation2d angle =
        AngleUtils.getDirectionToPosition(new Translation2d(0, 0), new Translation2d(1, 1));
    assertEquals(45, angle.getDegrees());

    angle = AngleUtils.getDirectionToPosition(new Translation2d(0, 0), new Translation2d(-1, 1));
    assertEquals(135, angle.getDegrees());

    angle = AngleUtils.getDirectionToPosition(new Translation2d(0, 0), new Translation2d(1, -1));
    assertEquals(-45, angle.getDegrees());

    angle = AngleUtils.getDirectionToPosition(new Translation2d(0, 0), new Translation2d(-1, -1));
    assertEquals(-135, angle.getDegrees());
  }
}
