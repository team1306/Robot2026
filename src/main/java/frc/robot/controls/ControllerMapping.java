package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public abstract class ControllerMapping {

  protected CommandXboxController driverController;
  protected CommandXboxController operatorController;

  public ControllerMapping(
      CommandXboxController driverController, CommandXboxController operatorController) {
    this.driverController = driverController;
    this.operatorController = operatorController;
  }

  public abstract void bind();

  public void clear() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
  }
}
