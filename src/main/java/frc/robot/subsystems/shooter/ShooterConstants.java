package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kV = 0;

  public static final AngularVelocity PID_TOLERANCE = RPM.of(50);

  public static final int leftTopMotorId = 0;
  public static final int leftBottomMotorId = 0;

  public static final int rightTopMotorId = 0;
  public static final int rightBottomMotorId = 0;
}
