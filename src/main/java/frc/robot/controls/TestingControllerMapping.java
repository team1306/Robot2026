package frc.robot.controls;

import badgerutils.triggers.AllianceTriggers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class TestingControllerMapping extends ControllerMapping {

  private final Drive drive;

  public TestingControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive) {
    super(driverController, operatorController);
    this.drive = drive;
  }

  @Override
  public void bind() {
    drive.setDefaultCommand(
        DriveCommands.joystickDriveCommand(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

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
        .whileTrue(
            DriveCommands.driveAimLockedCommand(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () ->
                    AllianceTriggers.isBlueAlliance()
                        ? Constants.Locations.blueHub.toTranslation2d()
                        : Constants.Locations.redHub.toTranslation2d()));
  }
}
