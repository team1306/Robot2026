package frc.robot;

import badgerutils.networktables.LoggedNetworkTablesBuilder;
import badgerutils.triggers.AllianceTriggers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.controls.Controls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.DeployerPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Autos {
  private static final Time STARTING_FUEL_SHOOT_DURATION = Seconds.of(3);

  private final Drive drivetrain;
  private final Indexer indexer;
  private final Intake intake;
  private final Shooter shooter;

  // Prefer to construct autos lazily to save limited memory. Required with many auto files
  private final LoggedDashboardChooser<Auto> autoChooser;

  private final LoggedNetworkNumber autoWaitTime =
      new LoggedNetworkNumber("Autos/Auto Wait Seconds");

  private static final List<String> autoNames = AutoBuilder.getAllAutoNames();

  public Autos(Drive drivetrain, Indexer indexer, Intake intake, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
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
    NamedCommands.registerCommand("shoot-8", new ParallelCommandGroup(
      DriveCommands.driveAimLockedCommand(drivetrain, () -> 0, () -> 0, FlippingUtil.flipFieldPosition(Constants.Locations.blueHub.toTranslation2d())),
      ShooterCommands.shootAtDistanceCommand(shooter, null), //TODO: make distance to hub util
      new WaitUntilCommand(shooter.isAtRequestedSpeed()).andThen((indexer.indexUntilCancelledCommand(() -> 1).withDeadline(new WaitCommand(STARTING_FUEL_SHOOT_DURATION))))
    ));

    NamedCommands.registerCommand("intake", intake.intakeAtDutyCycleCommand(1));

    NamedCommands.registerCommand("stop-intake", intake.intakeAtDutyCycleCommand(0));

    NamedCommands.registerCommand("spool-shooter", ShooterCommands.shootAtDistanceCommand(shooter, null)); //TODO: distance to hub

    NamedCommands.registerCommand("deploy-intake", intake.positionDeployerCommand(DeployerPosition.EXTENDED));

    NamedCommands.registerCommand("shoot-until-done", new ParallelCommandGroup(
      DriveCommands.driveAimLockedCommand(drivetrain, () -> 0, () -> 0, FlippingUtil.flipFieldPosition(Constants.Locations.blueHub.toTranslation2d())),
      indexer.indexUntilCancelledCommand(() -> 1))
    );
    
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
