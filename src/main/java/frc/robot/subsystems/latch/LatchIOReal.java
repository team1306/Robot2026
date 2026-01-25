package frc.robot.subsystems.latch;

import com.ctre.phoenix6.hardware.TalonFX;

public class LatchIOReal implements LatchIO {
  private final TalonFX motor;

  public LatchIOReal() {
    this.motor = new TalonFX(LatchConstants.latchMotorId);
  }

  @Override
  public void updateInputs(LatchIOInputs inputs) {
    inputs.isMotorConnected = motor.isConnected();
  }

  @Override
  public void extend() {
    // go to a position when design figures out how far it'll need to go
  }
}
