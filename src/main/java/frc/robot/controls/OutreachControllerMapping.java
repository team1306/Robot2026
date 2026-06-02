package frc.robot.controls;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import badgerutils.commands.CommandUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LocationUtils;
import frc.robot.util.LoggedNetworkNumberPlus;
import frc.robot.util.RebuiltUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class OutreachControllerMapping extends ControllerMapping {

  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Booster booster;
  private final Hood hood;

  @AutoLogOutput
  private final LoggedNetworkNumberPlus shooterRPS =
      new LoggedNetworkNumberPlus("/Tuning/Outreach/Shooter RPS", 12);

  @AutoLogOutput
  private final LoggedNetworkNumberPlus hoodAngle =
      new LoggedNetworkNumberPlus("/Tuning/Outreach/Hood Angle (0-0.9)", 0);

  @AutoLogOutput
  private final LoggedNetworkNumberPlus boosterDutyCycle =
      new LoggedNetworkNumberPlus("/Tuning/Outreach/Booster Duty Cycle", 0.5);

  @AutoLogOutput
  private final LoggedNetworkNumberPlus driveSpeedMultiplier =
      new LoggedNetworkNumberPlus("/Tuning/Outreach/Drive Speed Multiplier", 0.3);

  @AutoLogOutput
  private final LoggedNetworkNumberPlus turnSpeedMultiplier =
      new LoggedNetworkNumberPlus("/Tuning/Outreach/Turning Speed Multiplier", 0.5);

  public OutreachControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Booster booster,
      Hood hood) {
    super(driverController, operatorController);
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    this.booster = booster;

    this.hood = hood;
  }

  @Override
  public void bind() {
    /* ---Default Commands--- */

    // Drive with stick
    drive.setDefaultCommand(
        DriveCommands.joystickDriveCommand(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                driveSpeedMultiplier,
                turnSpeedMultiplier)
            .alongWith(
                new RunCommand(
                    () ->
                        Logger.recordOutput(
                            "Shooter/Distance to Hub",
                            LocationUtils.getDistanceToLocation(
                                drive.getPose().getTranslation(),
                                RebuiltUtils.getCurrentHubLocation().toTranslation2d())))));

    hood.setDefaultCommand(
        Commands.run(() -> hood.setAngle(Rotations.of(hoodAngle.getAsDouble())), hood));

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
        .rightBumper()
        .whileTrue(
            ShooterCommands.shootAtSpeedCommand(
                    shooter, () -> RotationsPerSecond.of(shooterRPS.get()))
                .alongWith(
                    new GuardedCommand(
                        indexer.indexUntilCancelledCommand(1),
                        shooter.isAtRequestedSpeed(Constants.Tolerances.NORMAL_SPEED_TOLERANCE)))
                .alongWith(booster.boostCommand(boosterDutyCycle)));

    driverController.rightTrigger().whileTrue(indexer.indexUntilCancelledCommand(1));

    driverController
        .a()
        .whileTrue(
            new DriveAimLockedCommand(
                drive,
                () -> 0,
                () -> 0,
                () -> Constants.Locations.blueHub.toTranslation2d(),
                true));
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
    CommandUtils.removeAndCancelDefaultCommand(hood);
  }
}
