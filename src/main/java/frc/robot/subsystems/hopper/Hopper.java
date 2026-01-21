package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  public final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  public final HopperIO hopperIO;
  private double targetSpeed = 0;

  public Hopper(HopperIO hopperIO) {
    this.hopperIO = hopperIO;
  }

  @Override
  public void periodic() {
    hopperIO.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    hopperIO.set(targetSpeed);
  }

  public void changeSpeed(double speed) {
    targetSpeed = speed;
  }
}
