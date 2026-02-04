package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {

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

  public ShooterIOReal() {
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
    // Motor Statuses
    var leftTopShooterStatus =
        BaseStatusSignal.refreshAll(leftTopMotorSupplyCurrent, leftTopMotorVelocity);
    var leftBottomShooterStatus =
        BaseStatusSignal.refreshAll(leftBottomMotorSupplyCurrent, leftBottomMotorVelocity);
    var rightTopShooterStatus =
        BaseStatusSignal.refreshAll(rightTopMotorSupplyCurrent, rightTopMotorVelocity);
    var rightBottomShooterStatus =
        BaseStatusSignal.refreshAll(rightBottomMotorSupplyCurrent, rightBottomMotorVelocity);

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
}
