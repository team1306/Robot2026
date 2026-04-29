package frc.robot.controls;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import badgerutils.commands.CommandUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FaceforwardCommand;
import frc.robot.commands.ShootOnTheMoveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.booster.Booster;
import frc.robot.subsystems.deploy.Deploy;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LocationUtils;
import frc.robot.util.RebuiltUtils;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class CompetitionControllerMapping extends ControllerMapping {

  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Booster booster;
  private final Hood hood;
  private final FuelDetection fuelDetection;
  private final Leds leds;
  private final Deploy deploy;

  public CompetitionControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Booster booster,
      Hood hood,
      FuelDetection fuelDetection,
      Leds leds,
      Deploy deploy) {
    super(driverController, operatorController);
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    this.booster = booster;
    this.hood = hood;
    this.fuelDetection = fuelDetection;
    this.leds = leds;
    this.deploy = deploy;

    shooter.resetVelocityOverride();
  }

  @Override
  public void bind() {
    Command logWithinRangeCommand =
        Commands.run(
            () ->
                SmartDashboard.putBoolean(
                    "Controls/In Range",
                    LocationUtils.getDistanceToLocation(
                            drive.getPose().getTranslation(),
                            RebuiltUtils.getCurrentHubLocation().toTranslation2d())
                        .gt(Feet.of(7.5))));

    Command loggedTargetCommand =
        Commands.run(
            () ->
                Logger.recordOutput(
                    "Controls/Target",
                    RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation())
                        ? RebuiltUtils.getCurrentHubLocation().toTranslation2d()
                        : RebuiltUtils.getNearestAllianceCorner(drive.getPose().getTranslation())));

    BooleanSupplier inAllianceZoneSupplier =
        () -> RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation());

    /* ---Default Commands--- */

    // Drive with stick
    drive.setDefaultCommand(
        DriveCommands.joystickDriveCommand(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX())
            .alongWith(logWithinRangeCommand));

    // Hood down
    hood.setDefaultCommand(hood.moveToAngle(HoodConstants.ZERO_POSITION));

    /* ---P1--- */

    // Reset Odometry
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Intake
    driverController
        .leftTrigger(0.5)
        .whileTrue(
            intake
                .intakeUntilInterruptedCommand(1)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Hub Sweep
    driverController
        .b()
        .whileTrue(
            Commands.deferredProxy(
                () -> getHubCollectingPath(drive, drive.getPose().getTranslation())));

    // Robot Relative Drive
    driverController
        .x()
        .whileTrue(
            DriveCommands.robotRelativeAngularVelocityCommand(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> -driverController.getRightX())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Reverse Intake
    driverController
        .y()
        .whileTrue(
            new StartEndCommand(() -> intake.setDutyCycle(-1), () -> intake.setDutyCycle(0), intake)
                .alongWith(
                    Commands.startEnd(
                        () -> indexer.setDutyCycle(-1), () -> indexer.setDutyCycle(0), indexer)));

    // Snake Mode
    driverController
        .a()
        .whileTrue(
            new FaceforwardCommand(
                    drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Shoot to Hub or Corner Depending on Location
    driverController
        .rightBumper()
        .whileTrue(
            ShootOnTheMoveCommands.aimAndShootOnTheMoveCommand(
                    drive,
                    shooter,
                    indexer,
                    deploy,
                    booster,
                    hood,
                    leds,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () ->
                        inAllianceZoneSupplier.getAsBoolean()
                            ? RebuiltUtils.getCurrentHubLocation().toTranslation2d()
                            : RebuiltUtils.getNearestAllianceCorner(
                                drive.getPose().getTranslation()),
                    inAllianceZoneSupplier,
                    operatorController.rightBumper(),
                    () ->
                        operatorController.rightBumper().getAsBoolean()
                            || !inAllianceZoneSupplier.getAsBoolean(),
                    () ->
                        operatorController.rightBumper().getAsBoolean()
                            || operatorController.leftBumper().getAsBoolean()
                            || !inAllianceZoneSupplier.getAsBoolean(),
                    () ->
                        operatorController.rightBumper().getAsBoolean()
                            || !inAllianceZoneSupplier.getAsBoolean())
                .alongWith(loggedTargetCommand)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    /* ---P2--- */

    // Force through defence?
    operatorController
        .leftStick()
        .whileTrue(
            Commands.startEnd(
                () -> drive.setHighCurrentLimits(), () -> drive.setLowCurrentLimits()));

    // Spool Shooter
    operatorController
        .rightTrigger()
        .whileTrue(
            ShooterCommands.shootAtDistanceCommand(
                    shooter,
                    () ->
                        LocationUtils.getDistanceToLocation(
                            inAllianceZoneSupplier.getAsBoolean()
                                ? RebuiltUtils.getCurrentHubLocation().toTranslation2d()
                                : RebuiltUtils.getNearestAllianceCorner(
                                    drive.getPose().getTranslation()),
                            drive.getPose().getTranslation()),
                    () -> ShooterCommands.HUB_SETPOINTS)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                .alongWith(
                    new InstantCommand(() -> operatorController.setRumble(RumbleType.kBothRumble, 0.25)))
                .alongWith(
                    (booster.boostCommand(-0.5).alongWith(indexer.indexUntilCancelledCommand(-0.5)))
                        .withDeadline(Commands.waitSeconds(0.25))
                        .andThen(booster.boostCommand(.8))))
        .onFalse(
            new InstantCommand(() -> operatorController.setRumble(RumbleType.kBothRumble, 0))
                .ignoringDisable(true));

    // DEPLOY
    operatorController.x().onTrue(deploy.deployCommand());
    operatorController.leftTrigger(0.5).whileTrue(deploy.crunchCommand());
    operatorController.povLeft().whileTrue(deploy.deployManuallyCommand(0.2));
    operatorController.povRight().whileTrue(deploy.deployManuallyCommand(-0.2));

    // OVERRIDES

    // Force Indexer
    operatorController.rightBumper().whileTrue(indexer.indexUntilCancelledCommand(1));

    // Override Shooter by +0.5 RPS
    operatorController
        .povUp()
        .onTrue(
            new InstantCommand(() -> shooter.changeVelocityOverride(RotationsPerSecond.of(0.5))));

    // Override Shooter by -0.5 RPS
    operatorController
        .povDown()
        .onTrue(
            new InstantCommand(() -> shooter.changeVelocityOverride(RotationsPerSecond.of(-0.5))));

    // Reset overrides
    operatorController.start().onTrue(new InstantCommand(() -> shooter.resetVelocityOverride()));

    // Reverse Shooter
    operatorController
        .y()
        .whileTrue(ShooterCommands.shootAtSpeedCommand(shooter, () -> RotationsPerSecond.of(-20)));

    // Reverse Indexer
    operatorController
        .b()
        .onTrue(new InstantCommand(() -> indexer.setDutyCycle(-0.25)))
        .onFalse(new InstantCommand(() -> indexer.setDutyCycle(0)));

    // Reverse Intake
    operatorController
        .a()
        .onTrue(new InstantCommand(() -> intake.setDutyCycle(-0.5)))
        .onFalse(new InstantCommand(() -> intake.setDutyCycle(0)));

    Command rumblePulse =
        Commands.sequence(
                Commands.runOnce(() -> operatorController.setRumble(RumbleType.kBothRumble, 0.1)),
                Commands.waitTime(Seconds.of(0.1)),
                Commands.runOnce(() -> operatorController.setRumble(RumbleType.kBothRumble, 0)))
            .finallyDo(() -> operatorController.setRumble(RumbleType.kBothRumble, 0));

    Trigger warningTrigger = new Trigger(() -> RebuiltUtils.getShiftTime() <= 5.0);

    warningTrigger.whileTrue(rumblePulse.ignoringDisable(true));
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
    CommandUtils.removeAndCancelDefaultCommand(intake);
  }

  private Command getHubCollectingPath(Drive drive, Translation2d position) {
    Command collectionPath;
    if (RebuiltUtils.isInAllianceZone(position)) {
      collectionPath =
          RebuiltUtils.isLeftSide(position)
              ? drive.leftToRightAllianceCloseHubSweep
              : drive.rightToLeftAllianceCloseHubSweep;
    } else if (RebuiltUtils.isInOpponentAllianceZone(position)) {
      collectionPath =
          RebuiltUtils.isLeftSide(position)
              ? drive.leftToRightAllianceFarHubSweep
              : drive.rightToLeftAllianceFarHubSweep;
    } else if (RebuiltUtils.isOurHalf(position)) {
      collectionPath =
          RebuiltUtils.isLeftSide(position)
              ? drive.leftToRightMidCloseHubSweep
              : drive.rightToLeftMidCloseHubSweep;
    } else if (!RebuiltUtils.isOurHalf(position)) {
      collectionPath =
          RebuiltUtils.isLeftSide(position)
              ? drive.leftToRightMidFarHubSweep
              : drive.rightToLeftMidFarHubSweep;
    } else {

      System.out.println("Could not find suitable path");
      return Commands.none();
    }

    return collectionPath;
  }
  /*
  private Command pathOnTheFlyToHubCollectingPath(Drive drive, Command hubCollectingPath) {
    HashMap<Command, Pose2d> map = new HashMap<>();
    map.put(
        drive.leftToRightAllianceCloseHubSweep, new Pose2d(3.45, 5, Rotation2d.fromDegrees(-90)));
    map.put(
        drive.rightToLeftAllianceCloseHubSweep, new Pose2d(3.45, 3.1, Rotation2d.fromDegrees(90)));
    map.put(drive.leftToRightMidCloseHubSweep, new Pose2d(5.8, 5, Rotation2d.fromDegrees(-90)));
    map.put(drive.rightToLeftMidCloseHubSweep, new Pose2d(5.8, 3.1, Rotation2d.fromDegrees(90)));
    map.put(drive.leftToRightMidFarHubSweep, new Pose2d(10.75, 5, Rotation2d.fromDegrees(-90)));
    map.put(drive.rightToLeftMidFarHubSweep, new Pose2d(10.75, 3.1, Rotation2d.fromDegrees(90)));
    map.put(drive.leftToRightAllianceFarHubSweep, new Pose2d(13.1, 5, Rotation2d.fromDegrees(-90)));
    map.put(
        drive.rightToLeftAllianceFarHubSweep, new Pose2d(13.1, 3.1, Rotation2d.fromDegrees(90)));

    return AutoBuilder.followPath(
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(drive.getPose(), map.get(hubCollectingPath)),
            new PathConstraints(2.0, 3.0, Degrees.of(540).in(Radians), Degrees.of(720).in(Radians)),
            new IdealStartingState(
                Math.sqrt(
                    Math.pow(drive.getChassisSpeeds().vxMetersPerSecond, 2)
                        + Math.pow(drive.getChassisSpeeds().vyMetersPerSecond, 2)),
                drive.getPose().getRotation()),
            new GoalEndState(2, map.get(hubCollectingPath).getRotation())));
  }*/
}
