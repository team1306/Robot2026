package frc.robot.subsystems.fueldetection;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class FuelDetectionReal implements FuelDetectionIO {
  private final PhotonCamera camera;

  private boolean warningReported = false;

  public FuelDetectionReal() {
    camera = new PhotonCamera(FuelDetectionConstants.FUEL_DETECTION_CAMERA_NAME);
  }

  @Override
  public void updateInputs(FuelDetectionInputs inputs) {
    inputs.isConnected = camera.isConnected();

    if (!camera.isConnected()) {
      inputs.bestTarget = new ObjectTarget(0, 0, -1);
      inputs.targets = new ObjectTarget[0];

      if (!warningReported)
        DriverStation.reportWarning("Fuel Detection Camera Not Connected", false);
      warningReported = true;
      return;
    }
    warningReported = false;

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget bestTarget = result.getBestTarget();

        inputs.bestTarget =
            new ObjectTarget(bestTarget.getYaw(), bestTarget.getPitch(), bestTarget.getArea());
        inputs.targets =
            targets.stream()
                .map((value) -> new ObjectTarget(value.yaw, value.pitch, value.area))
                .toArray(ObjectTarget[]::new);
      } else {
        inputs.bestTarget = new ObjectTarget(0, 0, -1);
      }
    }
  }
}
