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
import frc.robot.subsystems.shooter.Shooter;

public class SimTestingControllerMapping extends ControllerMapping {

  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;

  public SimTestingControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Indexer indexer,
      Shooter shooter) {
    super(driverController, operatorController);
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
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

    indexerTesting();
  }

  private void indexerTesting() {
    // A button: while held, indexer duty cycle is 0.5; when released, indexer duty cycle is 0
    driverController.a().whileTrue(indexer.indexUntilCancelledCommand(0.5));

    // B button: while held, indexer duty cycle is set by the left trigger; when released, indexer
    // duty cycle is 0
    driverController
        .b()
        .whileTrue(indexer.indexUntilCancelledCommand(() -> driverController.getLeftTriggerAxis()));
  }

  private void intakeTesting() {
    // A button: while held, intake duty cycle is 1 (running); when released, intake duty cycle is 0
    // (off).
    driverController.a().whileTrue(intake.intakeUntilInterruptedCommand());

    // B button: while held, intake duty cycle is 0.5 (running slower); when released, intake duty
    // cycle is 0 (off).
    driverController.b().whileTrue(intake.intakeUntilInterruptedCommand(0.5));

    // X button: intake at 0.5 power when pressed
    driverController.x().onTrue(intake.intakeAtDutyCycleCommand(0.5));

    // Y button: when pressed, stop intake
    driverController.y().onTrue(intake.intakeAtDutyCycleCommand(0));
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
  }
}
