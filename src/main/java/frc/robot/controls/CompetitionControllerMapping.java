package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CompetitionControllerMapping extends ControllerMapping {

  public CompetitionControllerMapping(
      CommandXboxController driverController, CommandXboxController operatorController) {
    super(driverController, operatorController);
  }

  @Override
  public void bind() {}

  @Override
  public void clear() {
    super.clear();
  }
}
