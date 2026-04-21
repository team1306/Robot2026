package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;

public class DeployIOReal implements DeployIO {

  // devices
  private final TalonFX motor;
  private final CANcoder encoder;

  // status signals
  private final StatusSignal<Current> deployerSupplyCurrent;
  private final StatusSignal<Current> deployerStatorCurrent;
  private final StatusSignal<Temperature> deployerTemperature;

  private final StatusSignal<Angle> deployerMotorPosition;

  private final StatusSignal<Angle> deployerEncoderPosition;
  private final StatusSignal<Double> positionError;

  // control
  private final PositionTorqueCurrentFOC positionRequest;
  private final DutyCycleOut dutyCycleRequest;

  private DeployerPosition lastKnownPosition;

  public DeployIOReal() {
    // devices
    motor = new TalonFX(Constants.CanIds.DEPLOYER_MOTOR_ID);
    encoder = new CANcoder(Constants.CanIds.DEPLOYER_ENCODER_ID);

    // configs
    motor.getConfigurator().apply(DeployConstants.DEPLOYER_MOTOR_CONFIGS);

    deployerSupplyCurrent = motor.getSupplyCurrent();
    deployerTemperature = motor.getDeviceTemp();
    deployerMotorPosition = motor.getPosition();
    deployerEncoderPosition = encoder.getPosition();
    positionError = motor.getClosedLoopError();
    deployerStatorCurrent = motor.getStatorCurrent();

    // control
    positionRequest = new PositionTorqueCurrentFOC(Degrees.of(0));
    dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
  }

  @Override
  public void updateInputs(DeployIOInputs inputs) {
    StatusCode motorStatus =
        BaseStatusSignal.refreshAll(
            deployerSupplyCurrent, deployerTemperature, deployerMotorPosition, positionError);

    inputs.isDeployerMotorConnected = motorStatus.isOK();

    inputs.deployerSupplyCurrent = deployerSupplyCurrent.getValue();
    inputs.deployerStatorCurrent = deployerStatorCurrent.getValue();
    inputs.deployerTemp = deployerTemperature.getValue();

    inputs.deployerPosition = deployerMotorPosition.getValue();
    inputs.positionError = positionError.getValue();

    StatusCode encoderStatus = BaseStatusSignal.refreshAll(deployerEncoderPosition);
    inputs.isDeployerEncoderConnected = encoderStatus.isOK();
    inputs.deployerEncoderPosition = deployerEncoderPosition.getValue();
  }

  @Override
  public void setPosition(DeployerPosition position) {
    int slot = 0;

    if (lastKnownPosition == DeployerPosition.EXTENDED && position == DeployerPosition.DUMP)
      slot = 1;
    if (lastKnownPosition == DeployerPosition.DUMP && position == DeployerPosition.EXTENDED)
      slot = 2;

    motor.setControl(positionRequest.withPosition(position.getAngle()).withSlot(slot));

    lastKnownPosition = position;
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    motor.setControl(dutyCycleRequest.withOutput(dutyCycle));
  }

  @Override
  public void adjustTarget(Angle angle) {}
}
