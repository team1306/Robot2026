package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import badgerutils.commands.CommandUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
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

    driverController.leftTrigger(0.5).whileTrue(intake.intakeUntilInterruptedCommand(1));

    driverController.b().whileTrue(new FuelCollectionCommand(drive, fuelDetection));

    driverController
        .x()
        .whileTrue(
            DriveCommands.robotRelativeAngularVelocityCommand(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

    driverController
        .y()
        .whileTrue(
            new FaceforwardCommand(
                drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX()));

    driverController
        .rightBumper()
        .whileTrue(
            new SafeShootCommand(
                drive,
                shooter,
                indexer,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d()));

    driverController
        .leftBumper()
        .whileTrue(
            new SafeShootCommand(
                drive,
                shooter,
                indexer,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> RebuiltUtils.getNearestAllianceCorner(drive.getPose().getTranslation())));

    driverController
        .rightTrigger()
        .whileTrue(
            new SafeShootCommand(
                    drive,
                    shooter,
                    indexer,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () ->
                        RebuiltUtils.isInAllianceZone(drive.getPose().getTranslation())
                            ? RebuiltUtils.getCurrentHubLocation().toTranslation2d()
                            : RebuiltUtils.getNearestAllianceCorner(
                                drive.getPose().getTranslation()))
                .alongWith(loggedTargetCommand));
    // P2 -- ME!!!

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
    operatorController
        .rightTrigger()
        .whileTrue(
            ShooterCommands.shootAtDistanceCommand(
                    shooter,
                    () ->
                        LocationUtils.getDistanceToLocation(
                            drive.getPose().getTranslation(),
                            RebuiltUtils.getCurrentHubLocation().toTranslation2d()))
                .alongWith(
                    new InstantCommand(
                        () ->
                            operatorController.setRumble(
                                RumbleType.kBothRumble, operatorController.getLeftTriggerAxis()))))
        .onFalse(new InstantCommand(() -> operatorController.setRumble(RumbleType.kBothRumble, 0)));

    // Overides

    operatorController
        .leftBumper()
        .onTrue(new InstantCommand(() -> shooter.setVelocity(RotationsPerSecond.of(0)))); // brake

    operatorController
        .povUp()
        .onTrue(new InstantCommand(() -> shooter.speedOveride.plus(RotationsPerSecond.of(10))));
    operatorController
        .povDown()
        .onTrue(new InstantCommand(() -> shooter.speedOveride.minus(RotationsPerSecond.of(10))));

    operatorController
        .povLeft()
        .onTrue(new InstantCommand(() -> intake.deployOveride.plus(Degrees.of(10))));
    operatorController
        .povRight()
        .onTrue(new InstantCommand(() -> intake.deployOveride.minus(Degrees.of(10))));

    operatorController
        .y()
        .onTrue(new InstantCommand(() -> shooter.setVelocity(RotationsPerSecond.of(-20))))
        .onFalse(new InstantCommand(() -> shooter.setIdle()));
    operatorController
        .b()
        .onTrue(new InstantCommand(() -> indexer.setDutyCycle(-1)))
        .onFalse(new InstantCommand(() -> indexer.setDutyCycle(0)));
    operatorController
        .a()
        .onTrue(new InstantCommand(() -> intake.setDutyCycle(-1)))
        .onFalse(new InstantCommand(() -> intake.setDutyCycle(0)));
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
  }
}
