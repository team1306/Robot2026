package frc.robot.subsystems.latch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Latch extends SubsystemBase {
  public final LatchIOInputsAutoLogged inputs = new LatchIOInputsAutoLogged();
  public final LatchIO latchIO;

  public Latch(LatchIO io) {
    this.latchIO = io;
  }
}
