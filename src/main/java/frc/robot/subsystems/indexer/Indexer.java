package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  public final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  public final IndexerIO indexerIO;
  private double targetSpeed = 0;

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    indexerIO.changeVelocity(targetSpeed);
  }

  public void changeVelocity(double velocity) {
    targetSpeed = velocity;
  }

  @AutoLogOutput
  public Command runSpeed(DoubleSupplier speed) {
    return runEnd(
        () -> indexerIO.changeVelocity(speed.getAsDouble()), () -> indexerIO.changeVelocity(0.0));
  }
}
