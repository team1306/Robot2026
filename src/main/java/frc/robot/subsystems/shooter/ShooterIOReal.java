package frc.robot.subsystems.shooter;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.LoggedNetworkNumberPlus;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterIOReal implements ShooterIO {

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KP_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Shooter KP", ShooterConstants.KP);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KI_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Shooter KI", ShooterConstants.KI);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KD_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Shooter KD", ShooterConstants.KD);

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

  private final StatusSignal<Voltage> leftTopMotorVoltage;
  private final StatusSignal<Voltage> leftBottomMotorVoltage;
  private final StatusSignal<Voltage> rightTopMotorVoltage;
  private final StatusSignal<Voltage> rightBottomMotorVoltage;

  private final StatusSignal<Angle> leftTopMotorAngle;
  private final StatusSignal<Angle> leftBottomMotorAngle;
  private final StatusSignal<Angle> rightTopMotorAngle;
  private final StatusSignal<Angle> rightBottomMotorAngle;

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
  private final VoltageOut voltageRequest;

  public ShooterIOReal() {
    KP_SUPPLIER.addSubscriber(value -> updatePIDFromTunables());
    KI_SUPPLIER.addSubscriber(value -> updatePIDFromTunables());
    KD_SUPPLIER.addSubscriber(value -> updatePIDFromTunables());

    // Request Initialization
    velocityRequest = new VelocityTorqueCurrentFOC(0);
    neutralRequest = new NeutralOut();
    voltageRequest = new VoltageOut(0).withEnableFOC(true);

    // CAN Device Initialization
    leftTopMotor = new TalonFX(Constants.CanIds.SHOOTER_LEFT_TOP_MOTOR_ID);
    leftBottomMotor = new TalonFX(Constants.CanIds.SHOOTER_LEFT_BOTTOM_MOTOR_ID);

    rightTopMotor = new TalonFX(Constants.CanIds.SHOOTER_RIGHT_TOP_MOTOR_ID);
    rightBottomMotor = new TalonFX(Constants.CanIds.SHOOTER_RIGHT_BOTTOM_MOTOR_ID);

    encoder = new CANcoder(Constants.CanIds.SHOOTER_ENCODER_ID);
    encoderVelocity = encoder.getVelocity();

    // Apply configurations
    leftTopMotor.getConfigurator().apply(ShooterConstants.CCW_SHOOTER_MOTOR_CONFIGS);
    leftBottomMotor.getConfigurator().apply(ShooterConstants.CCW_SHOOTER_MOTOR_CONFIGS);

    rightTopMotor.getConfigurator().apply(ShooterConstants.CW_SHOOTER_MOTOR_CONFIGS);
    rightBottomMotor.getConfigurator().apply(ShooterConstants.CW_SHOOTER_MOTOR_CONFIGS);

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

    leftTopMotorVoltage = leftTopMotor.getMotorVoltage();
    leftBottomMotorVoltage = leftBottomMotor.getMotorVoltage();
    rightTopMotorVoltage = rightTopMotor.getMotorVoltage();
    rightBottomMotorVoltage = rightBottomMotor.getMotorVoltage();

    leftTopMotorAngle = leftTopMotor.getPosition();
    leftBottomMotorAngle = leftBottomMotor.getPosition();
    rightTopMotorAngle = rightTopMotor.getPosition();
    rightBottomMotorAngle = rightBottomMotor.getPosition();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Motor Statuses
    StatusCode leftTopShooterStatus =
        BaseStatusSignal.refreshAll(
            leftTopMotorSupplyCurrent,
            leftTopMotorVelocity,
            leftTopMotorTemperature,
            leftTopMotorError,
            leftTopMotorVoltage,
            leftTopMotorAngle);
    StatusCode leftBottomShooterStatus =
        BaseStatusSignal.refreshAll(
            leftBottomMotorSupplyCurrent,
            leftBottomMotorVelocity,
            leftBottomMotorTemperature,
            leftBottomMotorError,
            leftBottomMotorVoltage,
            leftBottomMotorAngle);
    StatusCode rightTopShooterStatus =
        BaseStatusSignal.refreshAll(
            rightTopMotorSupplyCurrent,
            rightTopMotorVelocity,
            rightTopMotorTemperature,
            rightTopMotorError,
            rightTopMotorVoltage,
            rightTopMotorAngle);
    StatusCode rightBottomShooterStatus =
        BaseStatusSignal.refreshAll(
            rightBottomMotorSupplyCurrent,
            rightBottomMotorVelocity,
            rightBottomMotorTemperature,
            rightBottomMotorError,
            rightBottomMotorVoltage,
            rightBottomMotorAngle);

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

    inputs.shooterLeftTopMotorVoltage = leftTopMotorVoltage.getValue();
    inputs.shooterLeftBottomMotorVoltage = leftBottomMotorVoltage.getValue();
    inputs.shooterRightTopMotorVoltage = rightTopMotorVoltage.getValue();
    inputs.shooterRightBottomMotorVoltage = rightBottomMotorVoltage.getValue();

    inputs.shooterLeftTopMotorAngle = leftTopMotorAngle.getValue();
    inputs.shooterLeftBottomMotorAngle = leftBottomMotorAngle.getValue();
    inputs.shooterRightTopMotorAngle = rightTopMotorAngle.getValue();
    inputs.shooterRightBottomMotorAngle = rightBottomMotorAngle.getValue();

    // Encoder
    var encoderStatus = BaseStatusSignal.refreshAll(encoderVelocity);
    inputs.isEncoderConnected = encoderStatus.isOK();
    inputs.encoderSpeed = encoderVelocity.getValue();
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    leftTopMotor.setControl(velocityRequest.withVelocity(velocity));
    leftBottomMotor.setControl(velocityRequest.withVelocity(velocity));
    rightTopMotor.setControl(velocityRequest.withVelocity(velocity));
    rightBottomMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setIdle() {
    leftTopMotor.setControl(neutralRequest);
    leftBottomMotor.setControl(neutralRequest);
    rightTopMotor.setControl(neutralRequest);
    rightBottomMotor.setControl(neutralRequest);
  }

  @Override
  public void runCharacterization(Voltage voltage) {
    leftTopMotor.setControl(voltageRequest.withOutput(voltage));
    leftBottomMotor.setControl(voltageRequest.withOutput(voltage));
    rightTopMotor.setControl(voltageRequest.withOutput(voltage));
    rightBottomMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public void updatePIDFromTunables() {
    Slot0Configs config =
        MotorConfigUtils.createPidConfig(
            KP_SUPPLIER.get(),
            KI_SUPPLIER.get(),
            KD_SUPPLIER.get(),
            ShooterConstants.KS,
            ShooterConstants.KV,
            0,
            0,
            GravityTypeValue.Arm_Cosine);
    leftTopMotor.getConfigurator().apply(config);
    leftBottomMotor.getConfigurator().apply(config);
    rightTopMotor.getConfigurator().apply(config);
    rightBottomMotor.getConfigurator().apply(config);
  }
}
