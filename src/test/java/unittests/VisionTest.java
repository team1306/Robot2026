package unittests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class VisionTest {
  private static final double DELTA = 1e-9;

  @Test
  void periodicRejectsObservationsWithInvalidTagDataOrHeight() {
    UnitTestVisionIO io =
        new UnitTestVisionIO(
            observation(new Pose3d(2.0, 2.0, 0.0, Rotation3d.kZero), 0.0, 0),
            observation(new Pose3d(2.0, 2.0, 0.0, Rotation3d.kZero), 0.31, 1),
            observation(new Pose3d(2.0, 2.0, 0.76, Rotation3d.kZero), 0.0, 2));
    AtomicInteger consumedCount = new AtomicInteger();
    Vision vision =
        new Vision((pose, timestamp, standardDeviations) -> consumedCount.incrementAndGet(), io);

    vision.periodic();

    assertEquals(0, consumedCount.get());
    assertEquals(1, io.updateCount);
  }

  @Test
  void getTargetXReturnsLatestIoObservation() {
    UnitTestVisionIO io = new UnitTestVisionIO();
    io.targetX = Rotation2d.fromDegrees(-17.5);
    Vision vision = new Vision((pose, timestamp, standardDeviations) -> {}, io);

    vision.periodic();

    assertEquals(-17.5, vision.getTargetX(0).getDegrees(), DELTA);
  }

  private static PoseObservation observation(Pose3d pose, double ambiguity, int tagCount) {
    return new PoseObservation(1.0, pose, ambiguity, tagCount, 1.0);
  }

  private static class UnitTestVisionIO implements VisionIO {
    private final PoseObservation[] observations;
    private Rotation2d targetX = Rotation2d.kZero;
    private int updateCount;

    UnitTestVisionIO(PoseObservation... observations) {
      this.observations = observations;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      updateCount++;
      inputs.cameraName = "test-camera";
      inputs.connected = true;
      inputs.latestTargetObservation = new TargetObservation(targetX, Rotation2d.kZero);
      inputs.poseObservations = observations;
    }
  }
}
