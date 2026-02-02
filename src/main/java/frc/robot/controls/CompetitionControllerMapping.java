package frc.robot.controls;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import badgerutils.commands.CommandUtils;
import badgerutils.triggers.AllianceTriggers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class CompetitionControllerMapping extends ControllerMapping {

  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;

  public CompetitionControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Shooter shooter) {
    super(driverController, operatorController);
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
  }

  @Override
  public void bind() {
    Trigger aimedAtHubTrigger =
        new Trigger(
            () ->
                DriveCommands.isAimedAtHub(
                    drive,
                    AllianceTriggers.isRedAlliance()
                        ? Constants.Locations.redHub.toTranslation2d()
                        : Constants.Locations.blueHub.toTranslation2d(),
                    0.1));

    aimedAtHubTrigger
        .and(driverController.rightBumper())
        .onTrue(
            Commands.startEnd(
                    () -> driverController.setRumble(RumbleType.kBothRumble, 1),
                    () -> driverController.setRumble(RumbleType.kBothRumble, 0))
                .alongWith(
                    Commands.startEnd(
                        () -> operatorController.setRumble(RumbleType.kBothRumble, 1),
                        () -> operatorController.setRumble(RumbleType.kBothRumble, 0)))
                .withTimeout(0.05));
    drive.setDefaultCommand(
        DriveCommands.joystickDriveCommand(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    driverController
        .rightBumper()
        .whileTrue(
            DriveCommands.driveAimLockedCommand(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                AllianceTriggers.isRedAlliance()
                    ? Constants.Locations.redHub.toTranslation2d()
                    : Constants.Locations.blueHub.toTranslation2d()));

    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    driverController
        .leftTrigger(0.5)
        .onTrue(Commands.runOnce(() -> intake.setDutyCycle(1)))
        .onFalse(Commands.runOnce(() -> intake.setDutyCycle(0)));
    operatorController
        .leftTrigger()
        .whileTrue(new InstantCommand(() -> shooter.setVelocity(RotationsPerSecond.of(5))));
    operatorController
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () ->
                    ShooterCommands.shootAtDistanceCommand(
                        shooter,
                        AllianceTriggers.isRedAlliance()
                            ? () ->
                                Meters.of(
                                    Constants.Locations.redHub.getDistance(
                                        new Pose3d(drive.getPose()).getTranslation()))
                            : () ->
                                Meters.of(
                                    Constants.Locations.blueHub.getDistance(
                                        new Pose3d(drive.getPose()).getTranslation())))));
    operatorController
        .leftTrigger(0.5)
        .onTrue(new InstantCommand(() -> operatorController.setRumble(RumbleType.kLeftRumble, 1)))
        .onFalse(Commands.runOnce(() -> operatorController.setRumble(RumbleType.kBothRumble, 0.0)));
    operatorController
        .rightTrigger()
        .onTrue(
            new InstantCommand(() -> operatorController.setRumble(RumbleType.kRightRumble, 0.75)))
        .onFalse(
            Commands.runOnce(() -> operatorController.setRumble(RumbleType.kRightRumble, 0.0)));
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
  }
}
