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
    deployIO.setPosition(position);
    Logger.recordOutput("Deploy/Angle Setpoint", position.getAngle().in(Rotations));
    Logger.recordOutput("Deploy/Duty Cycle", 0); // seems to not work atm
  }

  public void setDutyCycle(double dutyCycle) {
    deployIO.setDutyCycle(dutyCycle);
    Logger.recordOutput("Deploy/Duty Cycle", dutyCycle);
  }

  public Command deployManuallyCommand(double dutyCycle) {
    return Commands.startEnd(() -> setDutyCycle(dutyCycle), () -> setDutyCycle(0), this);
  }

  public Command deployCommand() {
    return Commands.runOnce(() -> setDeployerPosition(DeployerPosition.EXTENDED), this);
  }

  public Command deployToPositionCommand(DeployerPosition position) {
    return Commands.startEnd(
        () -> {
          setDeployerPosition(position);
        },
        () -> setDeployerPosition(DeployerPosition.EXTENDED),
        this);
  }

  public Command crunchCommand() {
    return deployToPositionCommand(DeployerPosition.DUMP);
  }
}
