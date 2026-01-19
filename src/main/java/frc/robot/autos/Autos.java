package frc.robot.autos;

import badgerutils.networktables.LoggedNetworkTablesBuilder;
import badgerutils.triggers.AllianceTriggers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.controls.Controls;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Autos {
  private final Drive drivetrain;

  // Prefer to construct autos lazily to save limited memory. Required with many auto files
  private final LoggedDashboardChooser<String> autoChooser;

  private final LoggedNetworkNumber autoWaitTime =
      new LoggedNetworkNumber("Autos/Auto Wait Seconds");

  public Autos(Drive drivetrain) {
    this.drivetrain = drivetrain;
    autoWaitTime.set(0);

    autoChooser = new LoggedDashboardChooser<>("Autos/Auto Chooser");

    autoChooser.addDefaultOption("None", "");

    for (String auto : AutoBuilder.getAllAutoNames()) {
      autoChooser.addOption(auto, auto);
    }

    Controls.addPersistentTrigger(
        () ->
            LoggedNetworkTablesBuilder.createLoggedAutoResettingButton("Autos/Reset Odometry")
                .onTrue(new InstantCommand(this::resetAutoOdometry).ignoringDisable(true)));

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
