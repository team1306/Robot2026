package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  public final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  public final IndexerIO indexerIO;

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public void setDutyCycle(double dutyCycle) {
    indexerIO.setDutyCycle(dutyCycle);
    Logger.recordOutput("Indexer/Requested Duty Cycle", dutyCycle);
  }

  public Command indexUntilCancelledCommand(DoubleSupplier speed) {
    return (Commands.runEnd(() -> setDutyCycle(speed.getAsDouble()), () -> setDutyCycle(0), this));
  }

  public Command indexUntilCancelledCommand(double speed) {
    return (Commands.startEnd(() -> setDutyCycle(speed), () -> setDutyCycle(0), this));
  }

  public Command jumbleIndexer(DoubleSupplier speed) {
    return (Commands.runEnd(
        () ->
            setDutyCycle(
                Math.sin(RobotController.getFPGATime() * 0.001) * speed.getAsDouble() * 0.25),
        () -> setDutyCycle(0),
        this));
  }

  public Command indexForTime(Time time, double speed) {
    return indexUntilCancelledCommand(speed).withDeadline(Commands.waitTime(time));
  }
}
