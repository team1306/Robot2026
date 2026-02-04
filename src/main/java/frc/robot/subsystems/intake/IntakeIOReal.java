package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DutyCycleOut dutyCycle;

  private final TalonFX deployerMotor;
  private final PositionTorqueCurrentFOC deployerPositionRequest;

  public IntakeIOReal() {
    leftMotor = new TalonFX(Constants.CanIds.INTAKE_LEFT_MOTOR_ID);
    rightMotor = new TalonFX(Constants.CanIds.INTAKE_RIGHT_MOTOR_ID);
    deployerMotor = new TalonFX(Constants.CanIds.DEPLOYER_MOTOR_ID);

    leftMotor.getConfigurator().apply(IntakeConstants.CW_INTAKE_MOTOR_CONFIGS);
    rightMotor.getConfigurator().apply(IntakeConstants.CCW_INTAKE_MOTOR_CONFIGS);
    deployerMotor.getConfigurator().apply(IntakeConstants.DEPLOYER_MOTOR_CONFIGS);

    deployerPositionRequest = new PositionTorqueCurrentFOC(Degrees.of(0));

    dutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isLeftMotorConnected = leftMotor.isConnected();
    inputs.isRightMotorConnected = rightMotor.isConnected();
    inputs.isDeployerMotorConnected = deployerMotor.isConnected();
    inputs.leftVelocity = leftMotor.getVelocity().getValue();
    inputs.rightVelocity = rightMotor.getVelocity().getValue();
    inputs.deployerPosition = deployerMotor.getPosition().getValue();
    inputs.leftTemp = leftMotor.getDeviceTemp().getValue();
    inputs.rightTemp = rightMotor.getDeviceTemp().getValue();
    inputs.deployerTemp = deployerMotor.getDeviceTemp().getValue();
    inputs.leftSupplyCurrent = leftMotor.getSupplyCurrent().getValue();
    inputs.rightSupplyCurrent = rightMotor.getSupplyCurrent().getValue();
    inputs.deployerSupplyCurrent = deployerMotor.getSupplyCurrent().getValue();
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    leftMotor.setControl(this.dutyCycle.withOutput(dutyCycle));
    rightMotor.setControl(this.dutyCycle.withOutput(dutyCycle));
  }

  @Override
  public void setDeployerPosition(Angle angle) {
    deployerMotor.setControl(deployerPositionRequest.withPosition(angle));
  }
}
