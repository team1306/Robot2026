package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    public final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
    public final ShooterIO shooterIO;
    public final ShooterConstants shooterConstants;

    public Shooter(ShooterIO shooterIO, ShooterConstants shooterConstants) {
        this.shooterIO = shooterIO;
        this.shooterConstants = shooterConstants;
    }
}
