package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    public final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    public final IntakeIO intakeIO;
    
    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    @Override
    public void periodic(){
        intakeIO.updateInputs(inputs);
    }
}
