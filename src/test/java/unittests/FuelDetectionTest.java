package unittests;

import static org.junit.jupiter.api.Assertions.assertSame;

import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.fueldetection.FuelDetectionIO;
import frc.robot.subsystems.fueldetection.ObjectTarget;
import org.junit.jupiter.api.Test;

class FuelDetectionTest {
  @Test
  void returnsTargetProvidedByIo() {
    ObjectTarget target = new ObjectTarget(12.0, -3.0, 20.0);
    FuelDetection fuelDetection =
        new FuelDetection(
            new FuelDetectionIO() {
              @Override
              public void updateInputs(FuelDetectionInputs inputs) {
                inputs.bestTarget = target;
                inputs.isConnected = true;
              }
            });

    fuelDetection.periodic();

    assertSame(target, fuelDetection.getBestTarget());
  }
}
