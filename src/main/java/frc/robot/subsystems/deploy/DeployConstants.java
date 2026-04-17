package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.Amps;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class DeployConstants {
  public static final double KP = 130;
  public static final double KD = 5;
  public static final double KS = 5;
  public static final double KG = 0;

  private static final double ROTOR_TO_SENSOR_RATIO = (15D / 1D) * (21D / 20D);

  public static final TalonFXConfiguration DEPLOYER_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(50))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40)))
          .withSlot0(
              MotorConfigUtils.createPidConfig(KP, 0, KD, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(Constants.CanIds.DEPLOYER_ENCODER_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO));
}
