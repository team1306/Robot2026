package unittests;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import org.junit.jupiter.api.Test;

class ShooterTest {
  private static final double DELTA = 1e-9;

  @Test
  void velocityOverrideAddsMagnitudeInEitherDirection() {
    UnitTestShooterIO io = new UnitTestShooterIO();
    Shooter shooter = new Shooter(io);

    shooter.changeVelocityOverride(RotationsPerSecond.of(2.0));
    shooter.setVelocity(RotationsPerSecond.of(10.0));
    assertEquals(12.0, io.requestedVelocity.in(RotationsPerSecond), DELTA);

    shooter.setVelocity(RotationsPerSecond.of(-10.0));
    assertEquals(-12.0, io.requestedVelocity.in(RotationsPerSecond), DELTA);

    shooter.setVelocity(RotationsPerSecond.of(0.0));
    assertEquals(0.0, io.requestedVelocity.in(RotationsPerSecond), DELTA);
  }

  @Test
  void resetVelocityOverrideRestoresUnmodifiedSetpoint() {
    UnitTestShooterIO io = new UnitTestShooterIO();
    Shooter shooter = new Shooter(io);

    shooter.changeVelocityOverride(RotationsPerSecond.of(1.0));
    shooter.changeVelocityOverride(RotationsPerSecond.of(2.0));
    shooter.resetVelocityOverride();
    shooter.setVelocity(RotationsPerSecond.of(10.0));

    assertEquals(10.0, io.requestedVelocity.in(RotationsPerSecond), DELTA);
  }

  @Test
  void averageClosedLoopErrorUsesOnlyConnectedMotors() {
    Shooter shooter = new Shooter(new UnitTestShooterIO());
    shooter.inputs.isShooterLeftTopMotorConnected = true;
    shooter.inputs.isShooterLeftBottomMotorConnected = false;
    shooter.inputs.isShooterRightTopMotorConnected = true;
    shooter.inputs.isShooterRightBottomMotorConnected = true;
    shooter.inputs.shooterLeftTopClosedLoopError = 1.0;
    shooter.inputs.shooterLeftBottomClosedLoopError = 100.0;
    shooter.inputs.shooterRightTopClosedLoopError = 2.0;
    shooter.inputs.shooterRightBottomClosedLoopError = 3.0;

    assertEquals(2.0, shooter.getAvgClosedLoopError(), DELTA);
  }

  @Test
  void requestedSpeedSupplierComparesAverageErrorToTolerance() {
    Shooter shooter = new Shooter(new UnitTestShooterIO());
    shooter.inputs.isShooterLeftTopMotorConnected = true;
    shooter.inputs.shooterLeftTopClosedLoopError = 0.2;

    assertTrue(shooter.isAtRequestedSpeed(RotationsPerSecond.of(0.25)).getAsBoolean());
    assertFalse(shooter.isAtRequestedSpeed(RotationsPerSecond.of(0.2)).getAsBoolean());
  }

  @Test
  void idleAndCharacterizationDelegateToIo() {
    UnitTestShooterIO io = new UnitTestShooterIO();
    Shooter shooter = new Shooter(io);

    shooter.setIdle();
    shooter.runCharacterization(Volts.of(4.5));

    assertEquals(1, io.idleCount);
    assertEquals(4.5, io.characterizationVoltage.in(Volts), DELTA);
  }

  private static class UnitTestShooterIO implements ShooterIO {
    AngularVelocity requestedVelocity = RotationsPerSecond.of(0.0);
    Voltage characterizationVoltage = Volts.of(0.0);
    int idleCount;

    @Override
    public void setVelocity(AngularVelocity velocity) {
      requestedVelocity = velocity;
    }

    @Override
    public void setIdle() {
      idleCount++;
    }

    @Override
    public void runCharacterization(Voltage voltage) {
      characterizationVoltage = voltage;
    }
  }
}
