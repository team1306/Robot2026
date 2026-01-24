package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class RebuiltUtils {
  /**
   * @return The current HUB state on robots alliance
   */
  public static boolean isHubActive() {
    // Holy if statment
    String gameData = DriverStation.getGameSpecificMessage();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Double time = DriverStation.getMatchTime();

    if (gameData.length() > 0 && !DriverStation.isAutonomous() && alliance.isPresent()) {
      Boolean isShiftEven = getAllianceShift() % 2 == 0;
      switch (gameData.charAt(0)) {
        case 'B':
          if (alliance.get() == Alliance.Red) {
            return !isShiftEven;
          }
          if (alliance.get() == Alliance.Blue) {
            return isShiftEven;
          }
        case 'R':
          if (alliance.get() == Alliance.Red) {
            return isShiftEven;
          }
          if (alliance.get() == Alliance.Blue) {
            return !isShiftEven;
          }
        default:
          // This is corrupt data
          break;
      }
    }
    return false;
  }
  /**
   * @return The current alliance shift, if in AUTO, TRANSITION SHIFT or END GAME, returns 0
   */
  public static Integer getAllianceShift() {
    Double time = DriverStation.getMatchTime();
    if (DriverStation.isAutonomous()) {
      return 0;
    }
    if (time >= 130) {
      return 0;
    } // Transition Shift
    else if (time <= 130 && time >= 105) {
      return 1;
    } else if (time <= 105 && time >= 80) {
      return 2;
    } else if (time <= 80 && time >= 55) {
      return 3;
    } else if (time <= 55 && time >= 30) {
      return 4;
    } else if (time <= 30) {
      return 0;
    } // End Game
    else {
      return null;
    }
  }
}
