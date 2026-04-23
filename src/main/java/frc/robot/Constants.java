package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static class Locations {
    public static final Translation3d blueHub = new Translation3d(4.6, 4.034, 1.477);
    public static final Translation3d redHub;

    public static final Translation2d leftCornerBlueLocation = new Translation2d(1, 7);
    public static final Translation2d rightCornerBlueLocation = new Translation2d(1, 1);

    public static final Translation2d leftCornerRedLocation;
    public static final Translation2d rightCornerRedLocation;

    static {
      Translation2d redHubPosition = FlippingUtil.flipFieldPosition(blueHub.toTranslation2d());
      redHub =
          new Translation3d(
              redHubPosition.getMeasureX(), redHubPosition.getMeasureY(), blueHub.getMeasureZ());

      leftCornerRedLocation = FlippingUtil.flipFieldPosition(leftCornerBlueLocation);
      rightCornerRedLocation = FlippingUtil.flipFieldPosition(rightCornerBlueLocation);
    }
  }

  public static class Tolerances {
    public static final Rotation2d SCORING_ANGLE_TOLERANCE = Rotation2d.fromDegrees(5);
    public static final Rotation2d PASSING_ANGLE_TOLERANCE = Rotation2d.fromDegrees(15);
    public static final AngularVelocity INITIAL_SPEED_TOLERANCE = RotationsPerSecond.of(0.5);
    public static final AngularVelocity NORMAL_SPEED_TOLERANCE = RotationsPerSecond.of(2);
  }

  public static class CurrentLimits {
    public static final Current HIGH_DRIVE_STATOR = Amps.of(100);
    public static final Current HIGH_DRIVE_SUPPLY = Amps.of(80);
    public static final Current LOW_DRIVE_STATOR = Amps.of(60);
    public static final Current LOW_DRIVE_SUPPLY = Amps.of(50);
    public static final Current STEER_STATOR = Amps.of(60);

    public static final Current SHOOTER_SUPPLY = Amps.of(60);
    public static final Current BOOSTER_SUPPLY = Amps.of(60);
    public static final Current INTAKE_SUPPLY = Amps.of(60);
    public static final Current INDEXER_SUPPLY = Amps.of(40);

    public static final Current DEPLOY_STATOR = Amps.of(60);
    public static final Current DEPLOY_SUPPLY = Amps.of(40);
  }

  public static class CanIds {
    // shooter
    public static final int SHOOTER_LEFT_TOP_MOTOR_ID = 13;
    public static final int SHOOTER_LEFT_BOTTOM_MOTOR_ID = 14;
    public static final int SHOOTER_RIGHT_TOP_MOTOR_ID = 15;
    public static final int SHOOTER_RIGHT_BOTTOM_MOTOR_ID = 16;
    public static final int BOOSTER_MOTOR_ID = 28;

    public static final int SHOOTER_ENCODER_ID = 19;

    // indexer
    public static final int INDEXER_LEFT_MOTOR_ID = 21;
    public static final int INDEXER_RIGHT_MOTOR_ID = 22;

    // intake
    public static final int INTAKE_LEFT_MOTOR_ID = 23;
    public static final int INTAKE_RIGHT_MOTOR_ID = 24;
    public static final int DEPLOYER_MOTOR_ID = 25;
    public static final int DEPLOYER_ENCODER_ID = 27;

    public static final int CANDLE_ID = 26;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
