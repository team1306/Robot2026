package frc.robot.controls;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedNetworkNumberPlus;
import org.littletonrobotics.junction.AutoLogOutput;

public class CleaningControllerMapping extends ControllerMapping {
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;

  @AutoLogOutput
  private final LoggedNetworkNumberPlus targetSpeed =
      new LoggedNetworkNumberPlus("/Tuning/Shooter RPS", 0);

  public CleaningControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Intake intake,
      Indexer indexer,
      Shooter shooter) {
    super(driverController, operatorController);
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
  }

  @Override
  public void bind() {
    driverController.a().whileTrue(intake.intakeUntilInterruptedCommand(0.05));

    driverController.b().whileTrue(indexer.indexUntilCancelledCommand(0.05));

    driverController
        .x()
        .whileTrue(ShooterCommands.shootAtSpeedCommand(shooter, RotationsPerSecond.of(0.5)));

    driverController.y().whileTrue(intake.deployAtDutyCycleCommand(-0.25));
  }

  @Override
  public void clear() {
    super.clear();
  }
}
