package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    private IntakeCommands() {}

    public static Command extendLatch(Intake intake) {
        return new InstantCommand(() -> intake.intakeIO.extend());
    }

    public static Command setIntakePower(Intake intake, double power) {
        return new InstantCommand(() -> intake.intakeIO.set(power));
    }
}
