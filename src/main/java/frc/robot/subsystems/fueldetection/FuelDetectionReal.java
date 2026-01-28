package frc.robot.subsystems.fueldetection;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class FuelDetectionReal implements FuelDetectionIO {
  private final PhotonCamera camera;

  public FuelDetectionReal() {
    camera = new PhotonCamera(FuelDetectionConstants.fuelDetectionCameraName);
  }

  @Override
  public void updateInputs(FuelDetectionInputs inputs) {
    inputs.isConnected = camera.isConnected();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget bestTarget = result.getBestTarget();

        inputs.bestTarget =
            new ObjectTarget(bestTarget.getYaw(), bestTarget.getPitch(), bestTarget.getArea());
        inputs.targets =
            targets.stream()
                .map((value) -> new ObjectTarget(value.yaw, value.pitch, value.area))
                .toArray(ObjectTarget[]::new);
      }
    }
  }
}
