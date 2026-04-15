package frc.robot.controls;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import badgerutils.commands.CommandUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.DriveAimLockedCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GuardedCommand;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.booster.Booster;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LocationUtils;
import frc.robot.util.LoggedNetworkNumberPlus;
import frc.robot.util.RebuiltUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterTestingControllerMapping extends ControllerMapping {

  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Booster booster;

  @AutoLogOutput
  private final LoggedNetworkNumberPlus targetSpeed =
      new LoggedNetworkNumberPlus("/Tuning/Shooter RPS", 0.75);

  @AutoLogOutput
  private final LoggedNetworkNumberPlus boosterPower =
      new LoggedNetworkNumberPlus("/Tuning/Booster Duty Cycle", 1);

  public ShooterTestingControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Booster booster) {
    super(driverController, operatorController);
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    this.booster = booster;
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

    /* ---Default Commands--- */

    // Drive with stick
    drive.setDefaultCommand(
        DriveCommands.joystickDriveCommand(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX())
            .alongWith(logWithinRangeCommand)
            .alongWith(
                new RunCommand(
                    () ->
                        Logger.recordOutput(
                            "Shooter/Distance to Hub",
                            LocationUtils.getDistanceToLocation(
                                drive.getPose().getTranslation(),
                                RebuiltUtils.getCurrentHubLocation().toTranslation2d())))));

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

    driverController
        .leftBumper()
        .whileTrue(
            ShooterCommands.shootAtSpeedCommand(
                    shooter, () -> RotationsPerSecond.of(targetSpeed.get()))
                .alongWith(
                    new GuardedCommand(
                        indexer.indexUntilCancelledCommand(1),
                        shooter.isAtRequestedSpeed(Constants.Tolerances.NORMAL_SPEED_TOLERANCE))));
    driverController.a().whileTrue(booster.boostCommand(1));

    driverController
        .rightBumper()
        .whileTrue(
            ShooterCommands.shootAtSpeedCommand(
                    shooter, () -> RotationsPerSecond.of(targetSpeed.get()))
                .alongWith(
                    new GuardedCommand(
                            indexer.indexUntilCancelledCommand(1),
                            shooter.isAtRequestedSpeed(Constants.Tolerances.NORMAL_SPEED_TOLERANCE))
                        .alongWith(intake.shakeIntake())
                        .alongWith(
                            new DriveAimLockedCommand(
                                drive,
                                () -> -driverController.getLeftY(),
                                () -> -driverController.getLeftX(),
                                () -> RebuiltUtils.getCurrentHubLocation().toTranslation2d(),
                                true))));
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
  }
}
