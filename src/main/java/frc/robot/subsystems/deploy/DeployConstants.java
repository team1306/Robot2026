package frc.robot.subsystems.deploy;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class DeployConstants {
  public static final double KP_0 = 100;
  public static final double KD_0 = 15;
  public static final double KS_0 = 0;

  public static final double KP_1 = 300;
  public static final double KD_1 = 30;
  public static final double KS_1 = 0;

  private static final double ROTOR_TO_SENSOR_RATIO = (25D / 1D) * (18D / 18D);

  public static final TalonFXConfiguration DEPLOYER_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Constants.CurrentLimits.DEPLOY_STATOR)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Constants.CurrentLimits.DEPLOY_SUPPLY))
          .withSlot0(
              MotorConfigUtils.createPidConfig(
                  KP_0, 0, KD_0, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine))
          .withSlot1(new Slot1Configs().withKP(KP_1).withKD(KD_1).withKS(KS_1))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(Constants.CanIds.DEPLOYER_ENCODER_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO));
}
