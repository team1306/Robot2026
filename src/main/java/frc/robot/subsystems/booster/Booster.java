package frc.robot.subsystems.booster;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Booster extends SubsystemBase {

  public final BoosterIOInputsAutoLogged inputs = new BoosterIOInputsAutoLogged();
  public final BoosterIO io;

  public Booster(BoosterIO boosterIO) {
    this.io = boosterIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Booster", inputs);
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
    Logger.recordOutput("Booster/Requested Duty Cycle", dutyCycle);
  }

  public Command boostCommand(DoubleSupplier dutyCycle) {
    return (Commands.runEnd(
        () -> setDutyCycle(dutyCycle.getAsDouble()), () -> setDutyCycle(0), this));
  }

  public Command boostCommand(double dutyCycle) {
    return (Commands.startEnd(() -> setDutyCycle(dutyCycle), () -> setDutyCycle(0), this));
  }
}
