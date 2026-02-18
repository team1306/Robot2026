package frc.robot.controls;

import badgerutils.commands.CommandUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

public class CompetitionControllerMapping extends ControllerMapping {

  private final Drive drive;
  private final Intake intake;

  public CompetitionControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake) {
    super(driverController, operatorController);
    this.drive = drive;
    this.intake = intake;
  }

  @Override
  public void bind() {
    drive.setDefaultCommand(
        DriveCommands.faceForwardCommand(
            drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX()));

    driverController
        .leftTrigger(0.5)
        .whileTrue(
            DriveCommands.driveAimLockedCommand(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Constants.Locations.blueHub.toTranslation2d()));

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
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
  }
}
