package frc.robot.controls;

import badgerutils.commands.CommandUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class CompetitionControllerMapping extends ControllerMapping {

  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;

  public CompetitionControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Indexer indexer) {
    super(driverController, operatorController);
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
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
        .onTrue(Commands.runOnce(() -> intake.setDutyCycle(1)))
        .onFalse(Commands.runOnce(() -> intake.setDutyCycle(0)));

    // A button: while held, indexer duty cycle is 0.5; when released, indexer duty cycle is 0
    driverController.a().whileTrue(indexer.indexUntilCancelledCommand(0.5));

    // B button: while held, indexer duty cycle is set by the left trigger; when released, indexer
    // duty cycle is 0
    driverController
        .b()
        .whileTrue(indexer.indexUntilCancelledCommand(() -> driverController.getLeftTriggerAxis()));
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
  }
}
