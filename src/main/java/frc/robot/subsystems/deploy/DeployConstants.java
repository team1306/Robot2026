package frc.robot.subsystems.deploy;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class DeployConstants {
  public static final double KP = 0;
  public static final double KD = 0;

  public static final TalonFXConfiguration DEPLOYER_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.Clockwise_Positive, NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(false)
                  .withSupplyCurrentLimitEnable(false))
          .withSlot0(
              MotorConfigUtils.createPidConfig(KP, 0, KD, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine));
}
