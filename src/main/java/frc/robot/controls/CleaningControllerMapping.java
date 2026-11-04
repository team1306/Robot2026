package frc.robot.controls;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.booster.Booster;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedNetworkNumberPlus;
import org.littletonrobotics.junction.AutoLogOutput;

public class CleaningControllerMapping extends ControllerMapping {
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Booster booster;

  @AutoLogOutput
  private final LoggedNetworkNumberPlus targetSpeed =
      new LoggedNetworkNumberPlus("/Tuning/Shooter RPS", 0.75);

  public CleaningControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Intake intake,
      Indexer indexer,
      Shooter shooter,
      Booster booster) {
    super(driverController, operatorController);
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    this.booster = booster;
  }

  @Override
  public void bind() {
    driverController.a().whileTrue(intake.intakeUntilInterruptedCommand(0.08));

    driverController.b().whileTrue(indexer.indexUntilCancelledCommand(0.08));

    driverController.y().whileTrue(booster.boostCommand(0.05));

    driverController
        .x()
        .whileTrue(
            ShooterCommands.shootAtSpeedCommand(
                shooter, () -> RotationsPerSecond.of(targetSpeed.get())));
  }

  @Override
  public void clear() {
    super.clear();
  }
}
