package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final StatusSignal<Voltage> topMotorAppliedVoltage;
  private final StatusSignal<Voltage> bottomMotorAppliedVoltage;

  private final StatusSignal<AngularVelocity> topMotorVelocity;
  private final StatusSignal<AngularVelocity> bottomMotorVelocity;

  private final VoltageOut voltageRequest;
  private final VelocityTorqueCurrentFOC velocityRequest;

  public ShooterIOReal(ShooterConstants shooterConstants) {
    topMotor = new TalonFX(shooterConstants.getTopMotorId());
    bottomMotor = new TalonFX(shooterConstants.getBottomMotorId());

    TalonFXConfiguration topMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration bottomMotorConfig = new TalonFXConfiguration();

    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = ShooterConstants.kP;
    pidConfigs.kI = ShooterConstants.kI;
    pidConfigs.kD = ShooterConstants.kD;
    pidConfigs.kV = ShooterConstants.kV;

    topMotorConfig.Slot0 = pidConfigs;
    bottomMotorConfig.Slot0 = pidConfigs;

    topMotor.getConfigurator().apply(topMotorConfig);
    bottomMotor.getConfigurator().apply(bottomMotorConfig);

    voltageRequest = new VoltageOut(0);
    velocityRequest = new VelocityTorqueCurrentFOC(0);

    topMotorAppliedVoltage = topMotor.getMotorVoltage();
    bottomMotorAppliedVoltage = bottomMotor.getMotorVoltage();

    topMotorVelocity = topMotor.getVelocity();
    bottomMotorVelocity = bottomMotor.getVelocity();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var topShooterStatus = BaseStatusSignal.refreshAll(topMotorAppliedVoltage, topMotorVelocity);
    var bottomShooterStatus =
        BaseStatusSignal.refreshAll(bottomMotorAppliedVoltage, bottomMotorVelocity);

    inputs.isShooterTopMotorConnected = topShooterStatus.isOK();
    inputs.isShooterBottomMotorConnected = bottomShooterStatus.isOK();

    inputs.shooterTopMotorAppliedVoltage = topMotorAppliedVoltage.getValue();
    inputs.shooterBottomMotorAppliedVoltage = bottomMotorAppliedVoltage.getValue();

    inputs.shooterTopMotorSpeed = topMotorVelocity.getValue();
    inputs.shooterBottomMotorSpeed = bottomMotorVelocity.getValue();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    topMotor.setControl(voltageRequest.withOutput(voltage));
    bottomMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    topMotor.setControl(velocityRequest.withVelocity(velocity));
    bottomMotor.setControl(velocityRequest.withVelocity(velocity));
  }
}
