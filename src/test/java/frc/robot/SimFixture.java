package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * Observes the four shooter motors through Phoenix's simulation layer, and feeds them the inputs a
 * physical robot would supply.
 *
 * <p>These are second handles on the same CAN IDs that {@link
 * frc.robot.subsystems.shooter.ShooterIOReal} drives. Phoenix keys a device's simulation state
 * purely by device ID, so a second {@code TalonFX} observes the identical simulated device. That
 * lets tests read what the real IO layer commanded without adding test-only accessors to shipping
 * code.
 *
 * <p>Note that the shooter's left and right pairs are configured with opposite inversion, so a
 * command that spins the wheels the same direction physically produces opposite-signed outputs.
 * Assertions must use magnitude, not sign.
 */
public class SimFixture {

  public static final double SUPPLY_VOLTAGE = 12.0;

  private final Map<TalonFX, TalonFXSimState> motors;
  private final Map<CANcoder, CANcoderSimState> encoders;

  public SimFixture(RobotSimHarness harness, TalonFX[] motors, CANcoder[] encoders) {
    harness.addPreLoopHook(this::feedSimInputs);

    this.motors = new HashMap<>();
    for (TalonFX motor : motors) {
      this.motors.put(motor, motor.getSimState());
    }

    this.encoders = new HashMap<>();
    for (CANcoder encoder : encoders) {
      this.encoders.put(encoder, encoder.getSimState());
    }

    // Deliberately no getConfigurator().apply(...) anywhere in this class: these handles observe
    // the devices, they must not reconfigure what ShooterIOReal set up.
  }

  /** Runs before every robot loop, standing in for the physical power and sensor wiring. */
  private void feedSimInputs() {
    encoders.forEach(
        (encoder, simState) -> {
          simState.setSupplyVoltage(SUPPLY_VOLTAGE);
        });
    motors.forEach(
        (motor, simState) -> {
          simState.setSupplyVoltage(SUPPLY_VOLTAGE);
        });
  }

  public Set<TalonFX> motors() {
    return motors.keySet();
  }

  public Map<CANcoder, CANcoderSimState> encoders() {
    return encoders;
  }

  /** Commanded torque current, the output of the shooter's {@code VelocityTorqueCurrentFOC}. */
  public double torqueCurrentAmps(TalonFX motor) {
    return motors.get(motor).getTorqueCurrent();
  }

  public double motorVoltageVolts(TalonFX motor) {
    return motors.get(motor).getMotorVoltage();
  }

  /** The velocity setpoint the motor is closing on, in rotations per second. */
  public double closedLoopReferenceRps(TalonFX motor) {
    return motor.getClosedLoopReference().getValueAsDouble();
  }

  public boolean remoteSensorInvalid(TalonFX motor) {
    return motor.getFault_RemoteSensorDataInvalid().getValue();
  }

  /** One-line dump of everything a failing assertion might want to explain itself. */
  public String describe(TalonFX motor) {
    return String.format(
        "%-12s torqueCurrent=%8.3fA motorVoltage=%7.3fV closedLoopRef=%7.3frps remoteSensorBad=%s",
        motor.getDeviceID(),
        torqueCurrentAmps(motor),
        motorVoltageVolts(motor),
        closedLoopReferenceRps(motor),
        remoteSensorInvalid(motor));
  }
}
