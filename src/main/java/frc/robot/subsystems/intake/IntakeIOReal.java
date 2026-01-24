package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DutyCycleOut dutyCycle;

  public IntakeIOReal() {
    leftMotor = new TalonFX(IntakeConstants.intakeLeftMotorId);
    rightMotor = new TalonFX(IntakeConstants.intakeRightMotorId);
    dutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isLeftMotorConnected = leftMotor.isConnected();
    inputs.isRightMotorConnected = rightMotor.isConnected();
    inputs.leftRPM = leftMotor.getVelocity().getValueAsDouble();
    inputs.dutyCycle = leftMotor.getDutyCycle().getValueAsDouble();
  }

  @Override
  public void set(double power) {
    leftMotor.setControl(dutyCycle.withOutput(power));
  }
}
