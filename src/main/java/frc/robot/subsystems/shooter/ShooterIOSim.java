package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOSim implements ShooterIO {

    public ShooterIOSim() {
        
    }
    
    @Override
    public void updateInputs(ShooterIOInputs inputs) {}

    @Override
    public void setVoltage(Voltage voltage) {}

    @Override
    public void setVelocity(AngularVelocity velocity) {}
}
