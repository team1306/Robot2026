package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.Shooter;

// test class, not meant to stay around for competition.
public class TestingControllerMapping extends ControllerMapping {
    private final Shooter shooter;

    public TestingControllerMapping(
        CommandXboxController driverController, CommandXboxController operatorController, Shooter shooter) {
        super(driverController, operatorController);
        this.shooter = shooter;
    }

    @Override
    public void bind() {
        
        shooter.setDefaultCommand(new RunCommand(() -> shooter.setDutyCycle(driverController.getLeftY()), shooter));
    }
}
