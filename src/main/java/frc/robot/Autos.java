package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.SafeAimAndShootCommand;
import frc.robot.commands.ShootOnTheMoveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.booster.Booster;
import frc.robot.subsystems.deploy.Deploy;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.RebuiltUtils;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Autos {
  private static final Time SMALL_HOPPER_SHOOT_DURATION = Seconds.of(3);

  private final BooleanSupplier inAllianceZoneSupplier;

  private final Drive drive;
  private final Indexer indexer;
  private final Intake intake;
  private final Shooter shooter;
  private final Booster booster;
  private final Hood hood;
  private final Leds leds;
  private final Deploy deploy;

  private final Command sotmSmallHopperCommand;

  private final Command shootUntilDoneCommand;
  private final Command spoolShooterCommand;
  private final Command startIntakeCommand;
  private final Command stopIntakeCommand;
  private final Command deployCommand;

  // Auto chooser setup
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final LoggedNetworkNumber autoWaitTime =
      new LoggedNetworkNumber("Autos/Auto Wait Seconds");
  private final Field2d visualField = new Field2d();

  public Autos(
      Drive drive,
      Indexer indexer,
      Intake intake,
      Shooter shooter,
      Booster booster,
      Hood hood,
      Leds leds,
      Deploy deploy) {
    this.drive = drive;
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
    this.booster = booster;
    this.hood = hood;
    this.leds = leds;
    this.deploy = deploy;

    FollowPath.setDoubleLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
    FollowPath.setBooleanLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
    FollowPath.setPoseLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
    FollowPath.setTranslationListLoggingConsumer(
        p -> Logger.recordOutput(p.getFirst(), p.getSecond()));

    inAllianceZoneSupplier = () -> RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation());

    deployCommand = deploy.deployCommand().asProxy();

    sotmSmallHopperCommand =
        new ConditionalCommand(
            ShootOnTheMoveCommands.shootOnTheMoveAutoCommand(
                    drive,
                    shooter,
                    indexer,
                    deploy,
                    booster,
                    hood,
                    leds,
                    () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d(),
                    inAllianceZoneSupplier,
                    () -> false,
                    () -> false,
                    () -> true,
                    () -> false)
                .alongWith(intake.intakeUntilInterruptedCommand(1).asProxy())
                .withDeadline(Commands.waitTime(SMALL_HOPPER_SHOOT_DURATION)),
            Commands.none(),
            inAllianceZoneSupplier);

    spoolShooterCommand =
        ShooterCommands.shootAtDistanceCommand(
                shooter,
                () ->
                    Meters.of(
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(RebuiltUtils.getCurrentHubLocation().toTranslation2d())),
                () -> ShooterCommands.HUB_SETPOINTS)
            .asProxy();

    startIntakeCommand =
        intake
            .intakeUntilInterruptedCommand(1)
            .asProxy()
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    stopIntakeCommand =
        intake
            .intakeUntilInterruptedCommand(0)
            .asProxy()
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    shootUntilDoneCommand =
        new ConditionalCommand(
            new SafeAimAndShootCommand(
                drive,
                shooter,
                indexer,
                deploy,
                booster,
                hood,
                leds,
                () -> 0,
                () -> 0,
                () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d(),
                inAllianceZoneSupplier,
                () -> false,
                () -> false,
                () -> true,
                () -> false),
            Commands.none(),
            inAllianceZoneSupplier);

    autoWaitTime.set(0);
    FollowPath.registerEventTrigger("deployIntake", deployCommand);
    FollowPath.registerEventTrigger("startIntake", startIntakeCommand);
    FollowPath.registerEventTrigger("stopIntake", stopIntakeCommand);
    FollowPath.registerEventTrigger("coast", Commands.runOnce(() -> drive.setCoastMode()));

    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> drive.setBrakeMode()));

    // --- CONFIGURE AUTO CHOOSER ---

    // 1. Set the default routine (your original code)
    autoChooser.setDefaultOption("Citrus Right Sweep", getCitrusRight());
    autoChooser.addOption("Citrus Left Sweep", getCitrusLeft());
    autoChooser.addOption("Test", getTest());

    // 2. Add custom routines here
    autoChooser.addOption("Do Nothing", Commands.none());

    // Example of another custom path routine:
    // autoChooser.addOption("Citrus Left", Commands.sequence(pathBuilder.build(new
    // Path("CitrusLeft")), shootUntilDoneCommand));

    // 3. Push the chooser to SmartDashboard
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private Command buildShootSmallHopperCommand() {
    return new ConditionalCommand(
            new SafeAimAndShootCommand(
                drive,
                shooter,
                indexer,
                deploy,
                booster,
                hood,
                leds,
                () -> 0,
                () -> 0,
                () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d(),
                inAllianceZoneSupplier,
                () -> false,
                () -> false,
                () -> true,
                () -> false),
            Commands.none(),
            inAllianceZoneSupplier)
        .alongWith(deploy.crunchCommand().asProxy())
        .withDeadline(Commands.waitTime(SMALL_HOPPER_SHOOT_DURATION));
  }
  // Helper method to break out your original routine
  private Command getCitrusRight() {
    Path firstSweep = new Path("CitrusRightFirstSweep");
    Path secondSweep = new Path("CitrusRightSecondSweep");
    Path thirdSweep = new Path("CitrusRightThirdSweep");
    return new SequentialCommandGroup(
        buildPath(firstSweep, true),
        buildShootSmallHopperCommand(),
        buildPath(secondSweep, false),
        buildShootSmallHopperCommand(),
        buildPath(thirdSweep, false));
  }

  private Command getCitrusLeft() {
    Path firstSweep = new Path("CitrusRightFirstSweep");
    Path secondSweep = new Path("CitrusRightSecondSweep");
    Path thirdSweep = new Path("CitrusRightThirdSweep");
    firstSweep.mirror();
    secondSweep.mirror();
    thirdSweep.mirror();
    return new SequentialCommandGroup(
        buildPath(firstSweep, true),
        buildShootSmallHopperCommand(),
        buildPath(secondSweep, false),
        buildShootSmallHopperCommand(),
        buildPath(thirdSweep, false));
  }

  private Command getTest() {
    return Commands.sequence(
        buildPath(new Path("Test"), true),
        buildShootSmallHopperCommand(),
        shootUntilDoneCommand.asProxy());
  }

  private Command buildPath(Path path, boolean resetPose) {
    FollowPath.Builder pathBuilder =
        new FollowPath.Builder(
                drive, // Subsystem requirement
                drive::getPose, // Supplier<Pose2d>
                drive::getChassisSpeeds, // Supplier<ChassisSpeeds> (robot-relative)
                drive::runVelocity, // Consumer<ChassisSpeeds>  (robot-relative)
                new PIDController(4.5, 0.0, 0.0), // translation — minimizes remaining distance
                new PIDController(3.0, 0.0, 0.0), // rotation    — minimizes heading error
                new PIDController(2.0, 0.0, 0.0)) // cross-track — minimizes perpendicular deviation
            .withDefaultShouldFlip(); // auto-flip when on the red alliance
    if (resetPose) {
      pathBuilder = pathBuilder.withPoseReset(drive::setPose);
    }
    return pathBuilder.build(path);
  }

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
}
