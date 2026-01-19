package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    
    public final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
    public final HopperIO hopperIO;

    public Hopper(HopperIO hopperIO) {
        this.hopperIO = hopperIO;
    }
}
