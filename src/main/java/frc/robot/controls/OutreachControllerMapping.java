package frc.robot.controls;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import badgerutils.commands.CommandUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class OutreachControllerMapping extends ControllerMapping {
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;

  private final LoggedNetworkNumber driveSpeedFactor =
      new LoggedNetworkNumber("Outreach/Drive Speed", 0.5);
  private final LoggedNetworkNumber rotationSpeedFactor =
      new LoggedNetworkNumber("Outreach/Rotation Speed", 0.5);

  public OutreachControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Shooter shooter,
      Indexer indexer) {
    super(driverController, operatorController);
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
  }

  @Override
  public void bind() {
    drive.setDefaultCommand(
        DriveCommands.joystickDriveCommand(
            drive,
            () -> -driverController.getLeftY() * driveSpeedFactor.getAsDouble(),
            () -> -driverController.getLeftX() * driveSpeedFactor.getAsDouble(),
            () -> -driverController.getRightX() * rotationSpeedFactor.getAsDouble()));

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
            intake
                .intakeUntilInterruptedCommand(0.5)
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

    driverController
        .rightTrigger(0.5)
        .whileTrue(
            ShooterCommands.shootAtSpeedCommand(shooter, RotationsPerSecond.of(40))
                .alongWith(
                    Commands.waitSeconds(0.5).andThen(indexer.indexUntilCancelledCommand(0.5))));

    driverController.x().whileTrue(intake.deployCommand());

    // Override Shooter by 0.5 RPS
    driverController
        .povUp()
        .onTrue(
            new InstantCommand(() -> shooter.changeVelocityOverride(RotationsPerSecond.of(0.5))));

    // Override Shooter by -0.5 RPS
    driverController
        .povDown()
        .onTrue(
            new InstantCommand(() -> shooter.changeVelocityOverride(RotationsPerSecond.of(-0.5))));
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
  }
}
