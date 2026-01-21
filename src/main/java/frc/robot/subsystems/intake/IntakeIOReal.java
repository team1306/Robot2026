package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  public IntakeIOReal() {
    leftMotor = new TalonFX(IntakeConstants.intakeLeftMotorId);
    rightMotor = new TalonFX(IntakeConstants.intakeRightMotorId);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isLeftMotorConnected = leftMotor.isConnected();
    inputs.isRightMotorConnected = rightMotor.isConnected();
  }

  @Override
  public void set(IntakeState state) {}
}
