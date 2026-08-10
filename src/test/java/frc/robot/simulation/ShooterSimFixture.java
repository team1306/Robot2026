package frc.robot.simulation;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.robot.Constants;

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
public final class ShooterSimFixture {

  public static final double SUPPLY_VOLTAGE = 12.0;

  private static ShooterSimFixture instance;

  private final String[] names = {"leftTop", "leftBottom", "rightTop", "rightBottom"};
  private final TalonFX[] motors;
  private final TalonFXSimState[] motorSims;
  private final CANcoder encoder;
  private final CANcoderSimState encoderSim;

  /** Attaches the fixture to the harness, registering its per-loop input feed exactly once. */
  public static synchronized ShooterSimFixture attach(RobotSimHarness harness) {
    if (instance == null) {
      instance = new ShooterSimFixture();
      harness.addPreLoopHook(instance::feedSimInputs);
    }
    return instance;
  }

  private ShooterSimFixture() {
    motors =
        new TalonFX[] {
          new TalonFX(Constants.CanIds.SHOOTER_LEFT_TOP_MOTOR_ID),
          new TalonFX(Constants.CanIds.SHOOTER_LEFT_BOTTOM_MOTOR_ID),
          new TalonFX(Constants.CanIds.SHOOTER_RIGHT_TOP_MOTOR_ID),
          new TalonFX(Constants.CanIds.SHOOTER_RIGHT_BOTTOM_MOTOR_ID)
        };
    motorSims = new TalonFXSimState[motors.length];
    for (int i = 0; i < motors.length; i++) {
      motorSims[i] = motors[i].getSimState();
    }

    // Deliberately no getConfigurator().apply(...) anywhere in this class: these handles observe
    // the devices, they must not reconfigure what ShooterIOReal set up.
    encoder = new CANcoder(Constants.CanIds.SHOOTER_ENCODER_ID);
    encoderSim = encoder.getSimState();
  }

  /** Runs before every robot loop, standing in for the physical power and sensor wiring. */
  private void feedSimInputs() {
    // The shooter closes its velocity loop on the remote CANcoder, so that device must be powered
    // too or the closed loop has no feedback source.
    encoderSim.setSupplyVoltage(SUPPLY_VOLTAGE);
    for (TalonFXSimState motorSim : motorSims) {
      motorSim.setSupplyVoltage(SUPPLY_VOLTAGE);
    }
  }

  public int count() {
    return motors.length;
  }

  public String name(int index) {
    return names[index];
  }

  /** Commanded torque current, the output of the shooter's {@code VelocityTorqueCurrentFOC}. */
  public double torqueCurrentAmps(int index) {
    return motorSims[index].getTorqueCurrent();
  }

  public double motorVoltageVolts(int index) {
    return motorSims[index].getMotorVoltage();
  }

  /** The velocity setpoint the motor is closing on, in rotations per second. */
  public double closedLoopReferenceRps(int index) {
    return motors[index].getClosedLoopReference().refresh().getValueAsDouble();
  }

  public boolean remoteSensorInvalid(int index) {
    return motors[index].getFault_RemoteSensorDataInvalid().refresh().getValue();
  }

  /** One-line dump of everything a failing assertion might want to explain itself. */
  public String describe(int index) {
    return String.format(
        "%-12s torqueCurrent=%8.3fA motorVoltage=%7.3fV closedLoopRef=%7.3frps remoteSensorBad=%s",
        names[index],
        torqueCurrentAmps(index),
        motorVoltageVolts(index),
        closedLoopReferenceRps(index),
        remoteSensorInvalid(index));
  }
}
