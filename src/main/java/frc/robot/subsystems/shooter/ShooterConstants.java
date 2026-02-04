package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

public class ShooterConstants {

  public static final double kP = 1;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kV = 0;

  public static final int leftTopMotorId = 13;
  public static final int leftBottomMotorId = 14;

  public static final int rightTopMotorId = 15;
  public static final int rightBottomMotorId = 16;

  public static final CANBus CAN_BUS = new CANBus("35A40C2646324B532020204A0B1112FF");
}
