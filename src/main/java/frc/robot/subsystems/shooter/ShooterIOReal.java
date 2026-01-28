package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {

  private final TalonFX leftTopMotor;
  private final TalonFX leftBottomMotor;

  private final TalonFX rightTopMotor;
  private final TalonFX rightBottomMotor;

  private final StatusSignal<Voltage> leftTopMotorAppliedVoltage;
  private final StatusSignal<Voltage> leftBottomMotorAppliedVoltage;
  private final StatusSignal<Voltage> rightTopMotorAppliedVoltage;
  private final StatusSignal<Voltage> rightBottomMotorAppliedVoltage;

  private final StatusSignal<AngularVelocity> leftTopMotorVelocity;
  private final StatusSignal<AngularVelocity> leftBottomMotorVelocity;
  private final StatusSignal<AngularVelocity> rightTopMotorVelocity;
  private final StatusSignal<AngularVelocity> rightBottomMotorVelocity;

  private final VoltageOut voltageRequest;
  private final VelocityTorqueCurrentFOC velocityRequest;

  private final StatusSignal<Temperature> leftTopMotorTemperature;
  private final StatusSignal<Temperature> leftBottomMotorTemperature;
  private final StatusSignal<Temperature> rightTopMotorTemperature;
  private final StatusSignal<Temperature> rightBottomMotorTemperature;

  public ShooterIOReal() {
    leftTopMotor = new TalonFX(ShooterConstants.leftTopMotorId);
    leftBottomMotor = new TalonFX(ShooterConstants.leftBottomMotorId);

    rightTopMotor = new TalonFX(ShooterConstants.rightTopMotorId);
    rightBottomMotor = new TalonFX(ShooterConstants.rightBottomMotorId);

    TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = ShooterConstants.kP;
    pidConfigs.kI = ShooterConstants.kI;
    pidConfigs.kD = ShooterConstants.kD;
    pidConfigs.kV = ShooterConstants.kV;

    shooterMotorConfig.Slot0 = pidConfigs;

    leftTopMotor.getConfigurator().apply(shooterMotorConfig);
    leftBottomMotor.getConfigurator().apply(shooterMotorConfig);

    rightTopMotor.getConfigurator().apply(shooterMotorConfig);
    rightBottomMotor.getConfigurator().apply(shooterMotorConfig);

    voltageRequest = new VoltageOut(0);
    velocityRequest = new VelocityTorqueCurrentFOC(0);

    leftTopMotorAppliedVoltage = leftTopMotor.getMotorVoltage();
    leftBottomMotorAppliedVoltage = leftBottomMotor.getMotorVoltage();
    rightTopMotorAppliedVoltage = rightTopMotor.getMotorVoltage();
    rightBottomMotorAppliedVoltage = rightBottomMotor.getMotorVoltage();

    leftTopMotorVelocity = leftTopMotor.getVelocity();
    leftBottomMotorVelocity = leftBottomMotor.getVelocity();
    rightTopMotorVelocity = rightTopMotor.getVelocity();
    rightBottomMotorVelocity = rightBottomMotor.getVelocity();

    leftTopMotorTemperature = leftTopMotor.getDeviceTemp();
    leftBottomMotorTemperature = leftBottomMotor.getDeviceTemp();
    rightTopMotorTemperature = rightTopMotor.getDeviceTemp();
    rightBottomMotorTemperature = rightBottomMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var leftTopShooterStatus =
        BaseStatusSignal.refreshAll(leftTopMotorAppliedVoltage, leftTopMotorVelocity);
    var leftBottomShooterStatus =
        BaseStatusSignal.refreshAll(leftBottomMotorAppliedVoltage, leftBottomMotorVelocity);
    var rightTopShooterStatus =
        BaseStatusSignal.refreshAll(rightTopMotorAppliedVoltage, rightTopMotorVelocity);
    var rightBottomShooterStatus =
        BaseStatusSignal.refreshAll(rightBottomMotorAppliedVoltage, rightBottomMotorVelocity);

    inputs.isShooterLeftTopMotorConnected = leftTopShooterStatus.isOK();
    inputs.isShooterLeftBottomMotorConnected = leftBottomShooterStatus.isOK();
    inputs.isShooterRightTopMotorConnected = rightTopShooterStatus.isOK();
    inputs.isShooterRightBottomMotorConnected = rightBottomShooterStatus.isOK();

    inputs.shooterLeftTopMotorAppliedVoltage = leftTopMotorAppliedVoltage.getValue();
    inputs.shooterLeftBottomMotorAppliedVoltage = leftBottomMotorAppliedVoltage.getValue();
    inputs.shooterRightTopMotorAppliedVoltage = rightTopMotorAppliedVoltage.getValue();
    inputs.shooterRightBottomMotorAppliedVoltage = rightBottomMotorAppliedVoltage.getValue();

    inputs.shooterLeftTopMotorSpeed = leftTopMotorVelocity.getValue();
    inputs.shooterLeftBottomMotorSpeed = leftBottomMotorVelocity.getValue();
    inputs.shooterRightTopMotorSpeed = rightTopMotorVelocity.getValue();
    inputs.shooterRightBottomMotorSpeed = rightBottomMotorVelocity.getValue();

    inputs.shooterLeftTopTemperature = leftTopMotorTemperature.getValue();
    inputs.shooterLeftBottomTemperature = leftBottomMotorTemperature.getValue();
    inputs.shooterRightTopTemperature = rightTopMotorTemperature.getValue();
    inputs.shooterRightBottomTemperature = rightBottomMotorTemperature.getValue();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    leftTopMotor.setControl(voltageRequest.withOutput(voltage));
    leftBottomMotor.setControl(voltageRequest.withOutput(voltage));
    rightTopMotor.setControl(voltageRequest.withOutput(voltage));
    rightBottomMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    leftTopMotor.setControl(velocityRequest.withVelocity(velocity));
    leftBottomMotor.setControl(velocityRequest.withVelocity(velocity));
    rightTopMotor.setControl(velocityRequest.withVelocity(velocity));
    rightBottomMotor.setControl(velocityRequest.withVelocity(velocity));
  }
}
