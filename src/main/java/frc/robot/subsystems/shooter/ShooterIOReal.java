package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

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
    leftTopMotor = new TalonFX(ShooterConstants.LEFT_TOP_MOTOR_ID);
    leftBottomMotor = new TalonFX(ShooterConstants.LEFT_BOTTOM_MOTOR_ID);

    rightTopMotor = new TalonFX(ShooterConstants.RIGHT_TOP_MOTOR_ID);
    rightBottomMotor = new TalonFX(ShooterConstants.RIGHT_BOTTOM_MOTOR_ID);

    encoder = new CANcoder(ShooterConstants.ENCODER_ID);
    encoderVelocity = encoder.getVelocity();

    // Configs
    TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = ShooterConstants.KP;
    pidConfigs.kI = ShooterConstants.KI;
    pidConfigs.kD = ShooterConstants.KD;
    pidConfigs.kV = ShooterConstants.KV;

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.FeedbackRemoteSensorID = ShooterConstants.ENCODER_ID;
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    feedbackConfigs.RotorToSensorRatio = ShooterConstants.ROTOR_TO_SENSOR_RATIO;

    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;

    shooterMotorConfig.Slot0 = pidConfigs;
    shooterMotorConfig.Feedback = feedbackConfigs;
    shooterMotorConfig.CurrentLimits = currentConfigs;

    MotorOutputConfigs motorOutputConfigsNonInvert = new MotorOutputConfigs();
    motorOutputConfigsNonInvert.NeutralMode = NeutralModeValue.Coast;
    motorOutputConfigsNonInvert.Inverted = InvertedValue.CounterClockwise_Positive;

    MotorOutputConfigs motorOutputConfigsInvert = new MotorOutputConfigs();
    motorOutputConfigsInvert.NeutralMode = NeutralModeValue.Coast;
    motorOutputConfigsInvert.Inverted = InvertedValue.Clockwise_Positive;

    // Apply configurations
    leftTopMotor
        .getConfigurator()
        .apply(shooterMotorConfig.withMotorOutput(motorOutputConfigsNonInvert));
    leftBottomMotor
        .getConfigurator()
        .apply(shooterMotorConfig.withMotorOutput(motorOutputConfigsNonInvert));

    rightTopMotor
        .getConfigurator()
        .apply(shooterMotorConfig.withMotorOutput(motorOutputConfigsInvert));
    rightBottomMotor
        .getConfigurator()
        .apply(shooterMotorConfig.withMotorOutput(motorOutputConfigsInvert));

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
