package frc.robot.autos;

import badgerutils.triggers.AllianceTriggers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Autos {
  private final Drive drivetrain;

  private LoggedDashboardChooser<String> autoChooser =
      new LoggedDashboardChooser<>("Auto/Auto Chooser");

  private LoggedNetworkNumber autoWaitTime =
      new LoggedNetworkNumber("Autos/Auto Wait Time Seconds");

  public Autos(Drive drivetrain) {
    this.drivetrain = drivetrain;

    autoChooser.addDefaultOption("None", "");

    for (String auto : AutoBuilder.getAllAutoNames()) {
      autoChooser.addOption(auto, auto);
    }

    // waiting on badgerutils
    //        BadgerLog.createAutoResettingButton("Autos/Reset Odometry",
    // CommandScheduler.getInstance().getDefaultButtonLoop())
    //                .onTrue(new InstantCommand(this::resetAutoOdometry).ignoringDisable(true));

    bindNamedCommands();
  }

  public Command createAutoCommand() {
    if (autoChooser.get().isEmpty()) {
      return new InstantCommand();
    }

    return new WaitCommand(autoWaitTime.get()).andThen(new PathPlannerAuto(autoChooser.get()));
  }

  private void resetAutoOdometry() {
    if (!DriverStation.isDisabled() || autoChooser.get().isEmpty()) return;

    Pose2d startingPosition = new PathPlannerAuto(autoChooser.get()).getStartingPose();

    drivetrain.setPose(
        AllianceTriggers.isBlueAlliance()
            ? startingPosition
            : FlippingUtil.flipFieldPose(startingPosition));
  }

  private void bindNamedCommands() {
    //        NamedCommands.registerCommand();
  }
}
