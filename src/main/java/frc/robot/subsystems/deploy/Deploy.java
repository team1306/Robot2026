package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Deploy extends SubsystemBase {
  private final DeployIOInputsAutoLogged inputs = new DeployIOInputsAutoLogged();
  private final DeployIO deployIO;

  public Deploy(DeployIO deployIO) {
    this.deployIO = deployIO;
  }

  @Override
  public void periodic() {
    deployIO.updateInputs(inputs);
    Logger.processInputs("Deploy", inputs);
  }

  public void setDeployerPosition(DeployerPosition position) {
    deployIO.setPosition(position.getAngle());
    Logger.recordOutput("Deploy/Angle Setpoint", position.getAngle().in(Rotations));
  }

  public void setDutyCycle(double dutyCycle) {
    deployIO.setDutyCycle(dutyCycle);
    Logger.recordOutput("Deploy/Duty Cycle", dutyCycle);
  }

  public Command deployManuallyCommand(double dutyCycle) {
    return Commands.startEnd(() -> setDutyCycle(0.5), () -> setDutyCycle(0), this);
  }

  public Command retractManuallyCommand(double dutyCycle) {
    return Commands.startEnd(() -> setDutyCycle(-.5), () -> setDutyCycle(0), this);
  }

  public Command deployCommand() {
    return Commands.runOnce(() -> setDeployerPosition(DeployerPosition.EXTENDED), this);
  }

  public Command retractCommand() {
    return Commands.runOnce(() -> setDeployerPosition(DeployerPosition.RETRACTED), this);
  }

  public Command crunchCommand() {
    return Commands.startEnd(
        () -> setDeployerPosition(DeployerPosition.RETRACTED),
        () -> setDeployerPosition(DeployerPosition.EXTENDED),
        this);
  }
}
