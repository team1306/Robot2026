package frc.robot.subsystems.fueldetection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FuelDetector extends SubsystemBase {

  private final FuelDetectionInputsAutoLogged inputs = new FuelDetectionInputsAutoLogged();
  private final FuelDetectionIO io;

  public FuelDetector(FuelDetectionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Fuel Detection", inputs);
  }

  public ObjectTarget getBestTarget() {
      return inputs.bestTarget;
  }
}
