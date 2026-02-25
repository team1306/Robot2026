package frc.robot.controls;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import badgerutils.commands.CommandUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FaceforwardCommand;
import frc.robot.commands.FuelCollectionCommand;
import frc.robot.commands.SafeShootCommand;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LocationUtils;
import frc.robot.util.RebuiltUtils;
import org.littletonrobotics.junction.Logger;

public class CompetitionControllerMapping extends ControllerMapping {

  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final FuelDetection fuelDetection;

  public CompetitionControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      FuelDetection fuelDetection) {
    super(driverController, operatorController);
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    this.fuelDetection = fuelDetection;
  }

  @Override
  public void bind() {
    Command loggedTargetCommand =
        Commands.run(
            () ->
                Logger.recordOutput(
                    "Controls/Target",
                    RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation())
                        ? RebuiltUtils.getCurrentHubLocation().toTranslation2d()
                        : RebuiltUtils.getNearestAllianceCorner(drive.getPose().getTranslation())));

    /* ---Default Commands--- */

    // Drive with stick
    drive.setDefaultCommand(
        DriveCommands.joystickDriveCommand(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    driverController.a().whileTrue(indexer.indexUntilCancelledCommand(0.5));

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
                .intakeUntilInterruptedCommand(
                    () -> operatorController.rightStick().getAsBoolean() ? 0.5 : 0.75)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Fuel Collection
    driverController.b().whileTrue(new FuelCollectionCommand(drive, fuelDetection));

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

    // Snake Mode
    driverController
        .y()
        .whileTrue(
            new FaceforwardCommand(
                    drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Shoot to Hub
    driverController
        .rightBumper()
        .whileTrue(
            new SafeShootCommand(
                    drive,
                    shooter,
                    indexer,
                    intake,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d(),
                    operatorController.rightBumper())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Shoot to Corner
    driverController
        .leftBumper()
        .whileTrue(
            new SafeShootCommand(
                    drive,
                    shooter,
                    indexer,
                    intake,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> RebuiltUtils.getNearestAllianceCorner(drive.getPose().getTranslation()),
                    operatorController.rightBumper())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Shoot to Hub or Corner Depending on Location
    driverController
        .rightTrigger()
        .whileTrue(
            new SafeShootCommand(
                    drive,
                    shooter,
                    indexer,
                    intake,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () ->
                        RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation())
                            ? RebuiltUtils.getCurrentHubLocation().toTranslation2d()
                            : RebuiltUtils.getNearestAllianceCorner(
                                drive.getPose().getTranslation()),
                    operatorController.rightBumper())
                .alongWith(loggedTargetCommand));

    /* ---P2--- */

    // Jumble Indexer
    operatorController
        .leftTrigger(0.1)
        .whileTrue(
            indexer
                .jumbleIndexer(() -> operatorController.getLeftTriggerAxis())
                .alongWith(
                    new InstantCommand(
                        () ->
                            operatorController.setRumble(
                                RumbleType.kBothRumble, operatorController.getLeftTriggerAxis()))))
        .onFalse(new InstantCommand(() -> operatorController.setRumble(RumbleType.kBothRumble, 0)));
    /*
       operatorController
           .leftTrigger(0.1)
           .whileTrue(
               intake
                   .jumbleIntake()
                   .alongWith(
                       new InstantCommand(
                           () ->
                               operatorController.setRumble(
                                   RumbleType.kBothRumble, operatorController.getLeftTriggerAxis()))))
           .onFalse(new InstantCommand(() -> operatorController.setRumble(RumbleType.kBothRumble, 0)));
    */
    // Spool Shooter
    operatorController
        .rightTrigger()
        .whileTrue(
            ShooterCommands.shootAtDistanceCommand(
                    shooter,
                    () ->
                        LocationUtils.getDistanceToLocation(
                            RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation())
                                ? RebuiltUtils.getCurrentHubLocation().toTranslation2d()
                                : RebuiltUtils.getNearestAllianceCorner(
                                    drive.getPose().getTranslation()),
                            drive.getPose().getTranslation()))
                .alongWith(
                    new InstantCommand(
                        () -> operatorController.setRumble(RumbleType.kBothRumble, 0.25))))
        .onFalse(new InstantCommand(() -> operatorController.setRumble(RumbleType.kBothRumble, 0)));

    // Deploy Intake
    operatorController.x().onTrue(intake.deployCommand());

    // Overides

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
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
    CommandUtils.removeAndCancelDefaultCommand(intake);
  }
}
