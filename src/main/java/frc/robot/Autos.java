package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import badgerutils.networktables.LoggedNetworkTablesBuilder;
import badgerutils.triggers.AllianceTriggers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SafeAimAndShootCommand;
import frc.robot.commands.ShootOnTheMoveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.controls.Controls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.RebuiltUtils;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Autos {
  private static final Time STARTING_FUEL_SHOOT_DURATION = Seconds.of(1);
  private static final Time SMALL_HOPPER_SHOOT_DURATION = Seconds.of(3);

  private final BooleanSupplier inAllianceZoneSupplier;

  private final Drive drive;
  private final Indexer indexer;
  private final Intake intake;
  private final Shooter shooter;
  private final Leds leds;

  // Prefer to construct autos lazily to save limited memory. Required with many auto files
  private final LoggedDashboardChooser<Auto> autoChooser;

  private final LoggedNetworkNumber autoWaitTime =
      new LoggedNetworkNumber("Autos/Auto Wait Seconds");

  private static final List<String> autoNames = AutoBuilder.getAllAutoNames();

  public Autos(Drive drive, Indexer indexer, Intake intake, Shooter shooter, Leds leds) {
    this.drive = drive;
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
    this.leds = leds;

    inAllianceZoneSupplier = () -> RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation());

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
    bindEventMarkers();
  }

  public Command createCommandFromSelectedAuto() {
    Auto auto = autoChooser.get();
    return new WaitCommand(autoWaitTime.get()).andThen(auto.getCommand()).andThen(Commands.print("Auto Complete")).withName(auto.getName());
  }

  private void resetAutoOdometry() {
    Optional<PathPlannerAuto> autoOptional = autoChooser.get().getAuto();

    if (!DriverStation.isDisabled() || autoOptional.isEmpty()) return;

    Pose2d startingPosition = autoOptional.get().getStartingPose();

    drive.setPose(
        AllianceTriggers.isBlueAlliance()
            ? startingPosition
            : FlippingUtil.flipFieldPose(startingPosition));
  }

  private void bindNamedCommands() {
    NamedCommands.registerCommand(
        "shoot-8",
        new ConditionalCommand(
                new SafeAimAndShootCommand(
                    drive,
                    shooter,
                    indexer,
                    intake,
                    leds,
                    () -> 0,
                    () -> 0,
                    () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d(),
                    Constants.Tolerances.SCORING_ANGLE_TOLERANCE,
                    () -> false,
                    () -> false,
                    () -> true),
                Commands.none(),
                inAllianceZoneSupplier)
            .withDeadline(Commands.waitTime(STARTING_FUEL_SHOOT_DURATION)).asProxy());

    NamedCommands.registerCommand("intake", intake.intakeAtDutyCycleCommand(1).asProxy());

    NamedCommands.registerCommand("stop-intake", intake.intakeAtDutyCycleCommand(0).asProxy());

    NamedCommands.registerCommand(
        "spool-shooter",
        ShooterCommands.shootAtDistanceCommand(
            shooter,
            () ->
                Meters.of(
                    drive
                        .getPose()
                        .getTranslation()
                        .getDistance(RebuiltUtils.getCurrentHubLocation().toTranslation2d()))).asProxy());

    NamedCommands.registerCommand("deploy-intake", intake.deployCommand().asProxy());

    NamedCommands.registerCommand(
        "shoot-until-done",
        new ConditionalCommand(
            new SafeAimAndShootCommand(
                drive,
                shooter,
                indexer,
                intake,
                leds,
                () -> 0,
                () -> 0,
                () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d(),
                Constants.Tolerances.SCORING_ANGLE_TOLERANCE,
                () -> false,
                () -> false,
                () -> true),
            Commands.none(),
            inAllianceZoneSupplier).asProxy());

    NamedCommands.registerCommand(
        "shoot-small-hopper",
        new ConditionalCommand(
                new SafeAimAndShootCommand(
                    drive,
                    shooter,
                    indexer,
                    intake,
                    leds,
                    () -> 0,
                    () -> 0,
                    () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d(),
                    Constants.Tolerances.SCORING_ANGLE_TOLERANCE,
                    () -> false,
                    () -> false,
                    () -> true),
                Commands.none(),
                inAllianceZoneSupplier)
            .withDeadline(Commands.waitTime(SMALL_HOPPER_SHOOT_DURATION)).asProxy());

    NamedCommands.registerCommand(
        "stop-shooting", Commands.runOnce(() -> shooter.getCurrentCommand().cancel()));
  }

  private void bindEventMarkers() {
    // NONE OF THESE SHOULD REQUIRE THE DRIVE SUBSYSTEM

    new EventTrigger("shoot-until-done")
        .onTrue(
            new ConditionalCommand(
                ShootOnTheMoveCommands.shootOnTheMoveAutoCommand(
                    drive,
                    shooter,
                    indexer,
                    intake,
                    leds,
                    () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d(),
                    Constants.Tolerances.SCORING_ANGLE_TOLERANCE,
                    () -> false,
                    () -> false,
                    () -> true),
                Commands.none(),
                inAllianceZoneSupplier));

    new EventTrigger("shoot-8")
        .onTrue(
            new ConditionalCommand(
                    ShootOnTheMoveCommands.shootOnTheMoveAutoCommand(
                        drive,
                        shooter,
                        indexer,
                        intake,
                        leds,
                        () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d(),
                        Constants.Tolerances.SCORING_ANGLE_TOLERANCE,
                        () -> false,
                        () -> false,
                        () -> true),
                    Commands.none(),
                    inAllianceZoneSupplier)
                .withDeadline(Commands.waitTime(STARTING_FUEL_SHOOT_DURATION)));

    new EventTrigger("intake").onTrue(intake.intakeAtDutyCycleCommand(1));

    new EventTrigger("stop-intake").onTrue(intake.intakeAtDutyCycleCommand(0));

    new EventTrigger("spool-shooter")
        .onTrue(
            ShooterCommands.shootAtDistanceCommand(
                shooter,
                () ->
                    Meters.of(
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(RebuiltUtils.getCurrentHubLocation().toTranslation2d()))));

    new EventTrigger("deploy-intake").onTrue(intake.deployCommand());

    new EventTrigger("shoot-small-hopper")
        .onTrue(
            new ConditionalCommand(
                ShootOnTheMoveCommands.shootOnTheMoveAutoCommand(
                        drive,
                        shooter,
                        indexer,
                        intake,
                        leds,
                        () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d(),
                        Constants.Tolerances.SCORING_ANGLE_TOLERANCE,
                        () -> false,
                        () -> false,
                        () -> true)
                    .withDeadline(Commands.waitTime(SMALL_HOPPER_SHOOT_DURATION)),
                Commands.none(),
                inAllianceZoneSupplier));
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
