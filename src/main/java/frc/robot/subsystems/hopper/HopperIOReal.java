package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;

public class HopperIOReal implements HopperIO {

  private final TalonFX hopperMotor;
  private double targetSpeed;

  public HopperIOReal() {
    hopperMotor = new TalonFX(HopperConstants.hopperMotorId);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    hopperMotor.set(targetSpeed);
    System.out.println(targetSpeed);
  }

  @Override
  public void set(double speed) {
    targetSpeed = speed;
  }
}
