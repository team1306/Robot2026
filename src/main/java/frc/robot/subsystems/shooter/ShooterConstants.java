package frc.robot.subsystems.shooter;

public interface ShooterConstants {
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
