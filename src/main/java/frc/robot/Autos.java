package frc.robot;

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
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Autos {
  private final Drive drivetrain;

  // Prefer to construct autos lazily to save limited memory. Required with many auto files
  private final LoggedDashboardChooser<Auto> autoChooser;

  private final LoggedNetworkNumber autoWaitTime =
      new LoggedNetworkNumber("Autos/Auto Wait Seconds");

  private static final List<String> autoNames = AutoBuilder.getAllAutoNames();

  public Autos(Drive drivetrain) {
    this.drivetrain = drivetrain;
    autoWaitTime.set(0);

    autoChooser = new LoggedDashboardChooser<>("Autos/Auto Chooser");

    autoChooser.addDefaultOption("None", new Auto("Empty", new InstantCommand()));

    for (String auto : autoNames) {
      autoChooser.addOption(auto, new Auto(auto, auto));
    }

    Controls.addPersistentTrigger(
        () ->
            LoggedNetworkTablesBuilder.createLoggedAutoResettingButton("Autos/Reset Odometry")
                .onTrue(new InstantCommand(this::resetAutoOdometry).ignoringDisable(true)));

    bindNamedCommands();
  }

  public Command createCommandFromSelectedAuto() {
    Auto auto = autoChooser.get();
    return new WaitCommand(autoWaitTime.get()).andThen(auto.getCommand()).withName(auto.getName());
  }

  private void resetAutoOdometry() {
    Optional<PathPlannerAuto> autoOptional = autoChooser.get().getAuto();

    if (!DriverStation.isDisabled() || autoOptional.isEmpty()) return;

    Pose2d startingPosition = autoOptional.get().getStartingPose();

    drivetrain.setPose(
        AllianceTriggers.isBlueAlliance()
            ? startingPosition
            : FlippingUtil.flipFieldPose(startingPosition));
  }

  private void bindNamedCommands() {
    //        NamedCommands.registerCommand();
  }

  public static final class Auto {
    private final String autoName;
    private final String name;
    private final Command command;

    public Auto(String name, String autoName) {
      this.command = new InstantCommand();
      this.name = name;
      this.autoName = autoName;
    }

    public Auto(String name, Command command) {
      this.command = command;
      this.name = name;
      this.autoName = null;
    }

    public String getName() {
      return name;
    }

    public Optional<PathPlannerAuto> getAuto() {
      if (autoName == null) return Optional.empty();
      if (!autoNames.contains(autoName)) return Optional.empty();

      return Optional.of(new PathPlannerAuto(autoName));
    }

    public Command getCommand() {
      if (autoName != null) {
        return new PathPlannerAuto(autoName);
      } else if (command != null) {
        return command;
      }
      return new InstantCommand();
    }
  }
}
