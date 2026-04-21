package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.Amps;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class DeployConstants {
  public static final double KP_0 = 130;
  public static final double KD_0 = 5;
  public static final double KS_0 = 5;

  public static final double KP_1 = 130;
  public static final double KD_1 = 5;
  public static final double KS_1 = 5;

  public static final double KP_2 = 130;
  public static final double KD_2 = 5;
  public static final double KS_2 = 5;

  private static final double ROTOR_TO_SENSOR_RATIO = (15D / 1D) * (21D / 20D);

  public static final TalonFXConfiguration DEPLOYER_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40)))
          .withSlot0(
              MotorConfigUtils.createPidConfig(
                  KP_0, 0, KD_0, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine))
          .withSlot1(new Slot1Configs().withKP(KP_1).withKD(KD_1).withKS(KD_2))
          .withSlot2(new Slot2Configs().withKP(KP_2).withKD(KD_2).withKS(KS_2))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(Constants.CanIds.DEPLOYER_ENCODER_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO));
}
