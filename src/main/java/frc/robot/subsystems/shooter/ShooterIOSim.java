package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.LoggedNetworkNumberPlus;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterIOSim implements ShooterIO {

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KP_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Shooter KP", ShooterConstants.KP);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KI_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Shooter KI", ShooterConstants.KI);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KD_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Shooter KD", ShooterConstants.KD);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KV_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Shooter KV", ShooterConstants.KV);

  private final DCMotorSim simModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), 0.001, ShooterConstants.ROTOR_TO_SENSOR_RATIO),
          DCMotor.getKrakenX60Foc(1));

  private final StatusSignal<Current> leftTopMotorSupplyCurrent;
  private final StatusSignal<Current> leftBottomMotorSupplyCurrent;
  private final StatusSignal<Current> rightTopMotorSupplyCurrent;
  private final StatusSignal<Current> rightBottomMotorSupplyCurrent;

  private final StatusSignal<AngularVelocity> leftTopMotorVelocity;
  private final StatusSignal<AngularVelocity> leftBottomMotorVelocity;
  private final StatusSignal<AngularVelocity> rightTopMotorVelocity;
  private final StatusSignal<AngularVelocity> rightBottomMotorVelocity;

  private final StatusSignal<Temperature> leftTopMotorTemperature;
  private final StatusSignal<Temperature> leftBottomMotorTemperature;
  private final StatusSignal<Temperature> rightTopMotorTemperature;
  private final StatusSignal<Temperature> rightBottomMotorTemperature;

  private final StatusSignal<Double> leftTopMotorError;
  private final StatusSignal<Double> leftBottomMotorError;
  private final StatusSignal<Double> rightTopMotorError;
  private final StatusSignal<Double> rightBottomMotorError;

  private final StatusSignal<AngularVelocity> encoderVelocity;

  private final TalonFX leftTopMotor;
  private final TalonFX leftBottomMotor;
  private final TalonFX rightTopMotor;
  private final TalonFX rightBottomMotor;

  private final CANcoder encoder;

  private final VelocityTorqueCurrentFOC velocityRequest;
  private final NeutralOut neutralRequest;

  public ShooterIOSim() {
    KP_SUPPLIER.addSubscriber(value -> updatePIDFromTunables());
    KI_SUPPLIER.addSubscriber(value -> updatePIDFromTunables());
    KD_SUPPLIER.addSubscriber(value -> updatePIDFromTunables());
    KV_SUPPLIER.addSubscriber(value -> updatePIDFromTunables());

    // Request Initialization
    velocityRequest = new VelocityTorqueCurrentFOC(0);
    neutralRequest = new NeutralOut();

    // CAN Device Initialization
    leftTopMotor = new TalonFX(Constants.CanIds.SHOOTER_LEFT_TOP_MOTOR_ID);
    leftBottomMotor = new TalonFX(Constants.CanIds.SHOOTER_LEFT_BOTTOM_MOTOR_ID);

    rightTopMotor = new TalonFX(Constants.CanIds.SHOOTER_RIGHT_TOP_MOTOR_ID);
    rightBottomMotor = new TalonFX(Constants.CanIds.SHOOTER_RIGHT_BOTTOM_MOTOR_ID);

    encoder = new CANcoder(Constants.CanIds.SHOOTER_ENCODER_ID);
    encoderVelocity = encoder.getVelocity();

    // Apply configurations
    leftTopMotor.getConfigurator().apply(ShooterConstants.CW_SHOOTER_MOTOR_CONFIGS);
    leftBottomMotor.getConfigurator().apply(ShooterConstants.CW_SHOOTER_MOTOR_CONFIGS);

    rightTopMotor.getConfigurator().apply(ShooterConstants.CCW_SHOOTER_MOTOR_CONFIGS);
    rightBottomMotor.getConfigurator().apply(ShooterConstants.CCW_SHOOTER_MOTOR_CONFIGS);

    // Status Signals
    leftTopMotorSupplyCurrent = leftTopMotor.getSupplyCurrent();
    leftBottomMotorSupplyCurrent = leftBottomMotor.getSupplyCurrent();
    rightTopMotorSupplyCurrent = rightTopMotor.getSupplyCurrent();
    rightBottomMotorSupplyCurrent = rightBottomMotor.getSupplyCurrent();

    leftTopMotorVelocity = leftTopMotor.getVelocity();
    leftBottomMotorVelocity = leftBottomMotor.getVelocity();
    rightTopMotorVelocity = rightTopMotor.getVelocity();
    rightBottomMotorVelocity = rightBottomMotor.getVelocity();

    leftTopMotorTemperature = leftTopMotor.getDeviceTemp();
    leftBottomMotorTemperature = leftBottomMotor.getDeviceTemp();
    rightTopMotorTemperature = rightTopMotor.getDeviceTemp();
    rightBottomMotorTemperature = rightBottomMotor.getDeviceTemp();

    leftTopMotorError = leftTopMotor.getClosedLoopError();
    leftBottomMotorError = leftBottomMotor.getClosedLoopError();
    rightTopMotorError = rightTopMotor.getClosedLoopError();
    rightBottomMotorError = rightBottomMotor.getClosedLoopError();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    simModel.update(.02);

    // Motor Statuses
    StatusCode leftTopShooterStatus =
        BaseStatusSignal.refreshAll(
            leftTopMotorSupplyCurrent,
            leftTopMotorVelocity,
            leftTopMotorTemperature,
            leftTopMotorError);
    StatusCode leftBottomShooterStatus =
        BaseStatusSignal.refreshAll(
            leftBottomMotorSupplyCurrent,
            leftBottomMotorVelocity,
            leftBottomMotorTemperature,
            leftBottomMotorError);
    StatusCode rightTopShooterStatus =
        BaseStatusSignal.refreshAll(
            rightTopMotorSupplyCurrent,
            rightTopMotorVelocity,
            rightTopMotorTemperature,
            rightTopMotorError);
    StatusCode rightBottomShooterStatus =
        BaseStatusSignal.refreshAll(
            rightBottomMotorSupplyCurrent,
            rightBottomMotorVelocity,
            rightBottomMotorTemperature,
            rightBottomMotorError);

    // Motor Inputs
    inputs.isShooterLeftTopMotorConnected = leftTopShooterStatus.isOK();
    inputs.isShooterLeftBottomMotorConnected = leftBottomShooterStatus.isOK();
    inputs.isShooterRightTopMotorConnected = rightTopShooterStatus.isOK();
    inputs.isShooterRightBottomMotorConnected = rightBottomShooterStatus.isOK();

    inputs.shooterLeftTopMotorSupplyCurrent = leftTopMotorSupplyCurrent.getValue();
    inputs.shooterLeftBottomMotorSupplyCurrent = leftBottomMotorSupplyCurrent.getValue();
    inputs.shooterRightTopMotorSupplyCurrent = rightTopMotorSupplyCurrent.getValue();
    inputs.shooterRightBottomMotorSupplyCurrent = rightBottomMotorSupplyCurrent.getValue();

    inputs.shooterLeftTopMotorSpeed = leftTopMotorVelocity.getValue();
    inputs.shooterLeftBottomMotorSpeed = leftBottomMotorVelocity.getValue();
    inputs.shooterRightTopMotorSpeed = rightTopMotorVelocity.getValue();
    inputs.shooterRightBottomMotorSpeed = rightBottomMotorVelocity.getValue();

    inputs.shooterLeftTopTemperature = leftTopMotorTemperature.getValue();
    inputs.shooterLeftBottomTemperature = leftBottomMotorTemperature.getValue();
    inputs.shooterRightTopTemperature = rightTopMotorTemperature.getValue();
    inputs.shooterRightBottomTemperature = rightBottomMotorTemperature.getValue();

    inputs.shooterLeftTopClosedLoopError = leftTopMotorError.getValue();
    inputs.shooterLeftBottomClosedLoopError = leftBottomMotorError.getValue();
    inputs.shooterRightTopClosedLoopError = rightTopMotorError.getValue();
    inputs.shooterRightBottomClosedLoopError = rightBottomMotorError.getValue();

    // Encoder
    var encoderStatus = BaseStatusSignal.refreshAll(encoderVelocity);
    inputs.isEncoderConnected = encoderStatus.isOK();
    inputs.encoderSpeed = encoderVelocity.getValue();
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    simModel.setAngularVelocity(velocity.in(RotationsPerSecond));

    leftTopMotor.getSimState().setRotorVelocity(simModel.getAngularVelocity());
    leftBottomMotor.getSimState().setRotorVelocity(simModel.getAngularVelocity());
    rightTopMotor.getSimState().setRotorVelocity(simModel.getAngularVelocity());
    rightBottomMotor.getSimState().setRotorVelocity(simModel.getAngularVelocity());

    // leftTopMotor.setControl(velocityRequest.withVelocity(velocity));
    // leftBottomMotor.setControl(velocityRequest.withVelocity(velocity));
    // rightTopMotor.setControl(velocityRequest.withVelocity(velocity));
    // rightBottomMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setIdle() {
    leftTopMotor.setControl(neutralRequest);
    leftBottomMotor.setControl(neutralRequest);
    rightTopMotor.setControl(neutralRequest);
    rightBottomMotor.setControl(neutralRequest);
  }

  public void updatePIDFromTunables() {
    Slot0Configs config =
        MotorConfigUtils.createPidConfig(
            KP_SUPPLIER.get(),
            KI_SUPPLIER.get(),
            KD_SUPPLIER.get(),
            0,
            KV_SUPPLIER.get(),
            0,
            0,
            GravityTypeValue.Arm_Cosine);
    leftTopMotor.getConfigurator().apply(config);
    leftBottomMotor.getConfigurator().apply(config);
    rightTopMotor.getConfigurator().apply(config);
    rightBottomMotor.getConfigurator().apply(config);
  }
}
