package unittests;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShooterCommands.ShooterSetpoint;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodIO;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class HoodTest {
  private static final double DELTA = 1e-9;

  @Test
  void commandsApplyAnglesAndResetToZeroPosition() {
    UnitTestHoodIO io = new UnitTestHoodIO();
    Hood hood = new Hood(io);
    AtomicReference<Angle> requested = new AtomicReference<>(Rotations.of(0.3));
    Command command = hood.moveToAngle(requested::get);

    command.execute();
    assertEquals(0.3, io.angle.in(Rotations), DELTA);
    requested.set(Rotations.of(0.6));
    command.execute();
    assertEquals(0.6, io.angle.in(Rotations), DELTA);
    command.end(true);
    assertEquals(HoodConstants.ZERO_POSITION.in(Rotations), io.angle.in(Rotations), DELTA);

    hood.periodic();
    assertEquals(1, io.updateCount);
  }

  @Test
  void angleFromDistanceUsesInterpolatedSetpoint() {
    UnitTestHoodIO io = new UnitTestHoodIO();
    Hood hood = new Hood(io);
    ShooterSetpoint[] setpoints = {
      new ShooterSetpoint(Meters.of(0), Rotations.of(0.1), RotationsPerSecond.of(1), Seconds.of(1)),
      new ShooterSetpoint(Meters.of(10), Rotations.of(0.5), RotationsPerSecond.of(2), Seconds.of(2))
    };

    Command command = hood.angleFromDistance(() -> Meters.of(5), () -> setpoints);
    command.execute();

    assertEquals(0.3, io.angle.in(Rotations), DELTA);
  }

  private static class UnitTestHoodIO implements HoodIO {
    Angle angle = Rotations.of(0);
    int updateCount;

    @Override
    public void updateInputs(HoodIOInputs inputs) {
      updateCount++;
    }

    @Override
    public void setAngle(Angle angle) {
      this.angle = angle;
    }
  }
}
