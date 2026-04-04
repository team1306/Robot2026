package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public void setAngle(Angle angle) {
    io.setAngle(angle);
    Logger.recordOutput("Hood/Angle Setpoint", angle.in(Rotations));
  }

  public Command moveToAngle(Angle angle) {
    return Commands.runEnd(
        () -> setAngle(angle), () -> setAngle(HoodConstants.ZERO_POSITION), this);
  }
}
