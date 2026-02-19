package frc.robot.controls;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import badgerutils.commands.CommandUtils;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedNetworkNumberPlus;
import org.littletonrobotics.junction.AutoLogOutput;

public class CompetitionControllerMapping extends ControllerMapping {

  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;

  @AutoLogOutput
  private final LoggedNetworkNumberPlus targetSpeed =
      new LoggedNetworkNumberPlus("/Tuning/Shooter RPS", 0);

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
    driverController
        .a()
        .whileTrue(
            ShooterCommands.shootAtSpeedCommand(
                shooter, () -> RotationsPerSecond.of(targetSpeed.get())));
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
  }
}
