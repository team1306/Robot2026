package frc.robot.controls;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import badgerutils.commands.CommandUtils;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    // A button: while held, shooter rpm goes from 10-20 rps based on left trigger
    driverController
        .a()
        .whileTrue(
            ShooterCommands.shootAtDistanceCommand(
                shooter, () -> Meters.of(driverController.getLeftTriggerAxis() + 1))); // 1-2 meters

    // B button: while held shooter rpm goes to 20 rps for 2 seconds
    driverController
        .b()
        .onTrue(ShooterCommands.shootForTimeCommand(shooter, () -> Meters.of(2), Seconds.of(2)));

    // X button: while held, shooter rps goes to 10 rps;
    driverController
        .x()
        .whileTrue(ShooterCommands.shootAtSpeedCommand(shooter, RotationsPerSecond.of(10)));

    // Y button: while held, shooter rps goes from 0 to 10 rps based on left trigger
    driverController
        .y()
        .whileTrue(
            ShooterCommands.shootAtSpeedCommand(
                shooter, () -> RotationsPerSecond.of(10 * driverController.getLeftTriggerAxis())));
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
  }
}
