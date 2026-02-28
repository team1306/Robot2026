package frc.robot.util;

import badgerutils.triggers.AllianceTriggers;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Locations;
import java.util.Arrays;
import java.util.function.Supplier;

public class RebuiltUtils {
  public enum AllianceShift {
    AUTO,
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    DISCONNECTED,
    UNKNOWN;
  }

  private static final boolean[] looseSchedule = {
    true, true, true, false, true, false, true, true, true
  };
  private static final boolean[] winSchedule = {
    true, true, false, true, false, true, true, true, true
  };

  /**
   * @return The current HUB state on robots alliance
   */
  public static boolean isHubActive() {
    boolean[] currentSchedule = new boolean[9];
    String gameData = DriverStation.getGameSpecificMessage();
    boolean isRedAlliance = AllianceTriggers.isRedAlliance();
    if (!gameData.isEmpty() && !DriverStation.isAutonomous()) {

      switch (gameData.charAt(0)) {
        case 'B':
          currentSchedule = isRedAlliance ? looseSchedule : winSchedule;
        case 'R':
          currentSchedule = isRedAlliance ? winSchedule : looseSchedule;
        default:
          Arrays.fill(currentSchedule, true);
      }
    }

    return currentSchedule[getAllianceShift().ordinal()];
  }
  /**
   * @return The current alliance shift, if in AUTO, TRANSITION SHIFT or END GAME, returns 0
   */
  public static AllianceShift getAllianceShift() {
    double time = DriverStation.getMatchTime();
    if (DriverStation.isAutonomous()) {
      return AllianceShift.AUTO;
    }
    if (!DriverStation.isFMSAttached()) {
      return AllianceShift.DISCONNECTED;
    }

    if (time >= 130) {
      return AllianceShift.TRANSITION;
    } // Transition Shift
    else if (time <= 130 && time >= 105) {

      return AllianceShift.SHIFT1;
    } else if (time <= 105 && time >= 80) {

      return AllianceShift.SHIFT2;
    } else if (time <= 80 && time >= 55) {

      return AllianceShift.SHIFT3;
    } else if (time <= 55 && time >= 30) {

      return AllianceShift.SHIFT4;
    } else if (time <= 30) {
      return AllianceShift.ENDGAME;
    } // End Game
    return AllianceShift.UNKNOWN;
  }

  public static double getShiftTime() {
    AllianceShift currentShift = getAllianceShift();
    double time = DriverStation.getMatchTime();
    if (switch (currentShift) {
      case AUTO, ENDGAME, DISCONNECTED, UNKNOWN -> false;
      default -> true;
    }) {
      switch (currentShift) {
        case TRANSITION:
          return time - 130;
        case SHIFT1:
          return time - 105;
        case SHIFT2:
          return time - 80;
        case SHIFT3:
          return time - 55;
        case SHIFT4:
          return time - 30;
      }
    }
    return -1;
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

  public static Translation2d getNearestAllianceCorner(Translation2d currentPosition) {
    Translation2d leftCorner =
        AllianceTriggers.isRedAlliance()
            ? Locations.leftCornerRedLocation
            : Locations.leftCornerBlueLocation;
    Translation2d rightCorner =
        AllianceTriggers.isRedAlliance()
            ? Locations.rightCornerRedLocation
            : Locations.rightCornerBlueLocation;

    boolean rightIsCloser =
        currentPosition.getSquaredDistance(rightCorner)
            < currentPosition.getSquaredDistance(leftCorner);

    return rightIsCloser ? rightCorner : leftCorner;
  }

  public static boolean isInAllianceZone(Translation2d position) {
    return position.getX() <= 4.02844 || position.getX() >= 16.540988 - 4.02844;
  }
}
