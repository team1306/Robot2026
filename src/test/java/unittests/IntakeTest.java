package unittests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class IntakeTest {
  private static final double DELTA = 1e-9;

  @Test
  void commandsDelegateCurrentSupplierValueAndStopOnEnd() {
    UnitTestIntakeIO io = new UnitTestIntakeIO();
    Intake intake = new Intake(io);
    AtomicReference<Double> requested = new AtomicReference<>(0.25);
    Command command = intake.intakeUntilInterruptedCommand(requested::get);

    command.execute();
    assertEquals(0.25, io.dutyCycle, DELTA);
    requested.set(0.8);
    command.execute();
    assertEquals(0.8, io.dutyCycle, DELTA);
    command.end(false);
    assertEquals(0.0, io.dutyCycle, DELTA);

    intake.periodic();
    assertEquals(1, io.updateCount);
  }

  private static class UnitTestIntakeIO implements IntakeIO {
    double dutyCycle;
    int updateCount;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
      updateCount++;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
      this.dutyCycle = dutyCycle;
    }
  }
}
