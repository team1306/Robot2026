package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  // motors
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX deployerMotor;

  // status signals
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Current> deployerSupplyCurrent;

  private final StatusSignal<Temperature> leftTemperature;
  private final StatusSignal<Temperature> rightTemperature;
  private final StatusSignal<Temperature> deployerTemperature;

  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Angle> deployerPosition;

  // control
  private final PositionTorqueCurrentFOC deployerPositionRequest;
  private final DutyCycleOut dutyCycleRequest;

  public IntakeIOReal() {
    // motors
    leftMotor = new TalonFX(Constants.CanIds.INTAKE_LEFT_MOTOR_ID);
    rightMotor = new TalonFX(Constants.CanIds.INTAKE_RIGHT_MOTOR_ID);
    deployerMotor = new TalonFX(Constants.CanIds.DEPLOYER_MOTOR_ID);

    // configs
    leftMotor.getConfigurator().apply(IntakeConstants.CW_INTAKE_MOTOR_CONFIGS);
    rightMotor.getConfigurator().apply(IntakeConstants.CCW_INTAKE_MOTOR_CONFIGS);
    deployerMotor.getConfigurator().apply(IntakeConstants.DEPLOYER_MOTOR_CONFIGS);

    leftSupplyCurrent = leftMotor.getSupplyCurrent();
    rightSupplyCurrent = rightMotor.getSupplyCurrent();
    deployerSupplyCurrent = deployerMotor.getSupplyCurrent();

    leftTemperature = leftMotor.getDeviceTemp();
    rightTemperature = rightMotor.getDeviceTemp();
    deployerTemperature = deployerMotor.getDeviceTemp();

    leftVelocity = leftMotor.getVelocity();
    rightVelocity = rightMotor.getVelocity();
    deployerPosition = deployerMotor.getPosition();

    // control
    deployerPositionRequest = new PositionTorqueCurrentFOC(Degrees.of(0));
    dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    StatusCode leftMotorStatus =
        BaseStatusSignal.refreshAll(leftSupplyCurrent, leftTemperature, leftVelocity);
    StatusCode rightMotorStatus =
        BaseStatusSignal.refreshAll(rightSupplyCurrent, rightTemperature, rightVelocity);
    StatusCode deployerMotorStatus =
        BaseStatusSignal.refreshAll(deployerSupplyCurrent, deployerTemperature, deployerPosition);

    inputs.isLeftMotorConnected = leftMotorStatus.isOK();
    inputs.isRightMotorConnected = rightMotorStatus.isOK();
    inputs.isDeployerMotorConnected = deployerMotorStatus.isOK();

    inputs.leftSupplyCurrent = leftSupplyCurrent.getValue();
    inputs.rightSupplyCurrent = rightSupplyCurrent.getValue();
    inputs.deployerSupplyCurrent = deployerSupplyCurrent.getValue();

    inputs.leftTemp = leftTemperature.getValue();
    inputs.rightTemp = rightTemperature.getValue();
    inputs.deployerTemp = deployerTemperature.getValue();

    inputs.leftVelocity = leftVelocity.getValue();
    inputs.rightVelocity = rightVelocity.getValue();
    inputs.deployerPosition = deployerPosition.getValue();
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    leftMotor.setControl(this.dutyCycleRequest.withOutput(dutyCycle));
    rightMotor.setControl(this.dutyCycleRequest.withOutput(dutyCycle));
  }

  @Override
  public void setDeployerPosition(Angle angle) {
    deployerMotor.setControl(deployerPositionRequest.withPosition(angle));
  }
}
