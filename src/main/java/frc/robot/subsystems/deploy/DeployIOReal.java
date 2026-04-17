package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.Degrees;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;
import frc.robot.util.LoggedNetworkNumberPlus;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.AutoLogOutput;

public class DeployIOReal implements DeployIO {

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KP_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Deploy KP", DeployConstants.KP);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KD_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Deploy KD", DeployConstants.KD);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KS_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Deploy KS", DeployConstants.KS);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KG_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Deploy KG", DeployConstants.KG);

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

    DoubleConsumer applyConfigs =
        value ->
            motor
                .getConfigurator()
                .apply(
                    DeployConstants.DEPLOYER_MOTOR_CONFIGS.withSlot0(
                        MotorConfigUtils.createPidConfig(
                            KP_SUPPLIER.get(),
                            0,
                            KD_SUPPLIER.get(),
                            KS_SUPPLIER.get(),
                            0,
                            KG_SUPPLIER.get(),
                            0,
                            GravityTypeValue.Arm_Cosine)));

    KP_SUPPLIER.addSubscriber(applyConfigs);
    KD_SUPPLIER.addSubscriber(applyConfigs);
    KS_SUPPLIER.addSubscriber(applyConfigs);
    KG_SUPPLIER.addSubscriber(applyConfigs);
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
  public void setPosition(Angle angle) {
    motor.setControl(positionRequest.withPosition(angle));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    motor.setControl(dutyCycleRequest.withOutput(dutyCycle));
  }

  @Override
  public void adjustTarget(Angle angle) {}
}
