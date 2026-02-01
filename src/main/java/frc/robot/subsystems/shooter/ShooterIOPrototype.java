package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOPrototype implements ShooterIO {

  private final TalonFX leftTopMotor;

  private final TalonFX rightTopMotor;

  private final StatusSignal<Voltage> leftTopMotorAppliedVoltage;
  private final StatusSignal<Voltage> rightTopMotorAppliedVoltage;

  private final StatusSignal<AngularVelocity> leftTopMotorVelocity;
  private final StatusSignal<AngularVelocity> rightTopMotorVelocity;

  private final VoltageOut voltageRequest;
  private final VelocityTorqueCurrentFOC velocityRequest;

  private final StatusSignal<Temperature> leftTopMotorTemperature;
  private final StatusSignal<Temperature> rightTopMotorTemperature;

  public ShooterIOPrototype() {
    leftTopMotor = new TalonFX(ShooterConstants.leftTopMotorId);

    rightTopMotor = new TalonFX(ShooterConstants.rightTopMotorId);

    TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = ShooterConstants.kP;
    pidConfigs.kI = ShooterConstants.kI;
    pidConfigs.kD = ShooterConstants.kD;
    pidConfigs.kV = ShooterConstants.kV;

    shooterMotorConfig.Slot0 = pidConfigs;

    leftTopMotor.getConfigurator().apply(shooterMotorConfig);

    rightTopMotor
        .getConfigurator()
        .apply(
            shooterMotorConfig.withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));

    voltageRequest = new VoltageOut(0);
    velocityRequest = new VelocityTorqueCurrentFOC(0);

    leftTopMotorAppliedVoltage = leftTopMotor.getMotorVoltage();
    rightTopMotorAppliedVoltage = rightTopMotor.getMotorVoltage();

    leftTopMotorVelocity = leftTopMotor.getVelocity();
    rightTopMotorVelocity = rightTopMotor.getVelocity();

    leftTopMotorTemperature = leftTopMotor.getDeviceTemp();
    rightTopMotorTemperature = rightTopMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var leftTopShooterStatus =
        BaseStatusSignal.refreshAll(leftTopMotorAppliedVoltage, leftTopMotorVelocity);
    var rightTopShooterStatus =
        BaseStatusSignal.refreshAll(rightTopMotorAppliedVoltage, rightTopMotorVelocity);

    inputs.isShooterLeftTopMotorConnected = leftTopShooterStatus.isOK();
    inputs.isShooterLeftBottomMotorConnected = false;
    inputs.isShooterRightTopMotorConnected = rightTopShooterStatus.isOK();
    inputs.isShooterRightBottomMotorConnected = false;

    inputs.shooterLeftTopMotorAppliedVoltage = leftTopMotorAppliedVoltage.getValue();
    inputs.shooterRightTopMotorAppliedVoltage = rightTopMotorAppliedVoltage.getValue();

    inputs.shooterLeftTopMotorSpeed = leftTopMotorVelocity.getValue();
    inputs.shooterRightTopMotorSpeed = rightTopMotorVelocity.getValue();

    inputs.shooterLeftTopTemperature = leftTopMotorTemperature.getValue();
    inputs.shooterRightTopTemperature = rightTopMotorTemperature.getValue();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    leftTopMotor.setControl(voltageRequest.withOutput(voltage));
    rightTopMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    leftTopMotor.setControl(velocityRequest.withVelocity(velocity));
    rightTopMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    DutyCycleOut control = new DutyCycleOut(dutyCycle).withEnableFOC(true);
    leftTopMotor.setControl(control);
    rightTopMotor.setControl(control);
  }
}
