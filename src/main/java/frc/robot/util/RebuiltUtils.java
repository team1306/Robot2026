package frc.robot.util;

import badgerutils.triggers.AllianceTriggers;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import java.util.function.Supplier;

public class RebuiltUtils {
  /**
   * @return The current HUB state on robots alliance
   */
  public static boolean isHubActive() {
    // Holy if statment
    String gameData = DriverStation.getGameSpecificMessage();
    boolean isRedAlliance = AllianceTriggers.isRedAlliance();
    if (!gameData.isEmpty() && !DriverStation.isAutonomous()) {
      boolean isShiftEven = getAllianceShift() % 2 == 0;
      switch (gameData.charAt(0)) {
        case 'B':
          return isRedAlliance == !isShiftEven;
        case 'R':
          return isRedAlliance == isShiftEven;
      }
    }
    return true;
  }
  /**
   * @return The current alliance shift, if in AUTO, TRANSITION SHIFT or END GAME, returns 0
   */
  public static int getAllianceShift() {
    double time = DriverStation.getMatchTime();
    if (DriverStation.isAutonomous()) {
      return 0;
    }
    if (!DriverStation.isFMSAttached()) {
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

    return 0;
  }

  public static Translation3d getCurrentHubLocation() {
    return AllianceTriggers.isRedAlliance()
        ? Constants.Locations.redHub
        : Constants.Locations.blueHub;
  }

  public static Supplier<Translation3d> getCurrentHubLocationSupplier() {
    return () ->
        AllianceTriggers.isRedAlliance() ? Constants.Locations.redHub : Constants.Locations.blueHub;
  }
}
