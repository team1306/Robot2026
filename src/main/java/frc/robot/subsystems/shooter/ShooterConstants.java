package frc.robot.subsystems.shooter;

public interface ShooterConstants {

  double kP = 0;
  double kI = 0;
  double kD = 0;

  double kV = 0;

  class LeftShooterConstants implements ShooterConstants {

    @Override
    public int getTopMotorId() {
      return 0;
    }

    @Override
    public int getBottomMotorId() {
      return 0;
    }
  }

  class RightShooterConstants implements ShooterConstants {

    @Override
    public int getTopMotorId() {
      return 0;
    }

    @Override
    public int getBottomMotorId() {
      return 0;
    }
  }

  int getTopMotorId();

  int getBottomMotorId();
}
