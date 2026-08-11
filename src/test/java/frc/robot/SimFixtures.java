package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class SimFixtures {
  public static SimFixture createShooterSimFixture(RobotSimHarness harness) {
    return new SimFixture(
        harness,
        new TalonFX[] {
          new TalonFX(Constants.CanIds.SHOOTER_LEFT_TOP_MOTOR_ID),
          new TalonFX(Constants.CanIds.SHOOTER_LEFT_BOTTOM_MOTOR_ID),
          new TalonFX(Constants.CanIds.SHOOTER_RIGHT_TOP_MOTOR_ID),
          new TalonFX(Constants.CanIds.SHOOTER_RIGHT_BOTTOM_MOTOR_ID)
        },
        new CANcoder[] {new CANcoder(Constants.CanIds.SHOOTER_ENCODER_ID)});
  }
}
