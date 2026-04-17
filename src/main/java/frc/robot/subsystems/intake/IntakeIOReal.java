package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  // motors
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  // status signals
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> rightSupplyCurrent;

  private final StatusSignal<Temperature> leftTemperature;
  private final StatusSignal<Temperature> rightTemperature;

  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<AngularVelocity> rightVelocity;

  // control
  private final DutyCycleOut dutyCycleRequest;

  public IntakeIOReal() {
    // motors
    leftMotor = new TalonFX(Constants.CanIds.INTAKE_LEFT_MOTOR_ID);
    rightMotor = new TalonFX(Constants.CanIds.INTAKE_RIGHT_MOTOR_ID);

    // configs
    leftMotor.getConfigurator().apply(IntakeConstants.CCW_INTAKE_MOTOR_CONFIGS);
    rightMotor.getConfigurator().apply(IntakeConstants.CW_INTAKE_MOTOR_CONFIGS);

    leftSupplyCurrent = leftMotor.getSupplyCurrent();
    rightSupplyCurrent = rightMotor.getSupplyCurrent();

    leftTemperature = leftMotor.getDeviceTemp();
    rightTemperature = rightMotor.getDeviceTemp();

    leftVelocity = leftMotor.getVelocity();
    rightVelocity = rightMotor.getVelocity();

    // control
    dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    StatusCode leftMotorStatus =
        BaseStatusSignal.refreshAll(leftSupplyCurrent, leftTemperature, leftVelocity);
    StatusCode rightMotorStatus =
        BaseStatusSignal.refreshAll(rightSupplyCurrent, rightTemperature, rightVelocity);

    inputs.isLeftMotorConnected = leftMotorStatus.isOK();
    inputs.isRightMotorConnected = rightMotorStatus.isOK();

    inputs.leftSupplyCurrent = leftSupplyCurrent.getValue();
    inputs.rightSupplyCurrent = rightSupplyCurrent.getValue();

    inputs.leftTemp = leftTemperature.getValue();
    inputs.rightTemp = rightTemperature.getValue();

    inputs.leftVelocity = leftVelocity.getValue();
    inputs.rightVelocity = rightVelocity.getValue();
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    leftMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    rightMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
  }
}
