package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
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


  private final TalonFX leftTopMotor;
  private final TalonFX leftBottomMotor;
  private final TalonFX rightTopMotor;
  private final TalonFX rightBottomMotor;

  private final VelocityTorqueCurrentFOC velocityRequest;

  public ShooterIOReal() {
    leftTopMotor = new TalonFX(ShooterConstants.LEFT_TOP_MOTOR_ID);
    leftBottomMotor = new TalonFX(ShooterConstants.LEFT_BOTTOM_MOTOR_ID);

    rightTopMotor = new TalonFX(ShooterConstants.RIGHT_TOP_MOTOR_ID);
    rightBottomMotor = new TalonFX(ShooterConstants.RIGHT_BOTTOM_MOTOR_ID);

    TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = ShooterConstants.KP;
    pidConfigs.kI = ShooterConstants.KI;
    pidConfigs.kD = ShooterConstants.KD;
    pidConfigs.kV = ShooterConstants.KV;

    shooterMotorConfig.Slot0 = pidConfigs;

    leftTopMotor.getConfigurator().apply(shooterMotorConfig);
    leftBottomMotor.getConfigurator().apply(shooterMotorConfig);

    rightTopMotor.getConfigurator().apply(shooterMotorConfig);
    rightBottomMotor.getConfigurator().apply(shooterMotorConfig);

    velocityRequest = new VelocityTorqueCurrentFOC(0);

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
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var leftTopShooterStatus =
        BaseStatusSignal.refreshAll(leftTopMotorSupplyCurrent, leftTopMotorVelocity);
    var leftBottomShooterStatus =
        BaseStatusSignal.refreshAll(leftBottomMotorSupplyCurrent, leftBottomMotorVelocity);
    var rightTopShooterStatus =
        BaseStatusSignal.refreshAll(rightTopMotorSupplyCurrent, rightTopMotorVelocity);
    var rightBottomShooterStatus =
        BaseStatusSignal.refreshAll(rightBottomMotorSupplyCurrent, rightBottomMotorVelocity);

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
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    leftTopMotor.setControl(velocityRequest.withVelocity(velocity));
    leftBottomMotor.setControl(velocityRequest.withVelocity(velocity));
    rightTopMotor.setControl(velocityRequest.withVelocity(velocity));
    rightBottomMotor.setControl(velocityRequest.withVelocity(velocity));
  }
}
