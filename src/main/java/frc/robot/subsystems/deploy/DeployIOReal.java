package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
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

  // motors
  private final TalonFX motor;

  // status signals
  private final StatusSignal<Current> deployerSupplyCurrent;

  private final StatusSignal<Temperature> deployerTemperature;

  private final StatusSignal<Angle> deployerPosition;

  // control
  private final PositionTorqueCurrentFOC positionRequest;

  public DeployIOReal() {
    // motors
    motor = new TalonFX(Constants.CanIds.DEPLOYER_MOTOR_ID);

    // configs
    motor.getConfigurator().apply(DeployConstants.DEPLOYER_MOTOR_CONFIGS);

    deployerSupplyCurrent = motor.getSupplyCurrent();
    deployerTemperature = motor.getDeviceTemp();
    deployerPosition = motor.getPosition();

    // control
    positionRequest = new PositionTorqueCurrentFOC(Degrees.of(0));

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
                            0,
                            0,
                            0,
                            0,
                            GravityTypeValue.Arm_Cosine)));

    KP_SUPPLIER.addSubscriber(applyConfigs);
    KD_SUPPLIER.addSubscriber(applyConfigs);
  }

  @Override
  public void updateInputs(DeployIOInputs inputs) {
    StatusCode motorStatus =
        BaseStatusSignal.refreshAll(deployerSupplyCurrent, deployerTemperature, deployerPosition);

    inputs.isDeployerMotorConnected = motorStatus.isOK();

    inputs.deployerSupplyCurrent = deployerSupplyCurrent.getValue();

    inputs.deployerTemp = deployerTemperature.getValue();

    inputs.deployerPosition = deployerPosition.getValue();
  }

  @Override
  public void setPosition(Angle angle) {
    motor.setControl(positionRequest.withPosition(angle));
  }

  @Override
  public void adjustTarget(Angle angle) {
    
  }
}
