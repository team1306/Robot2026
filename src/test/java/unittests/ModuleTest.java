package unittests;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIO;
import org.junit.jupiter.api.Test;

class ModuleTest {
  private static final double WHEEL_RADIUS_METERS = 0.05;
  private static final double DELTA = 1e-9;

  @Test
  void periodicConvertsSensorInputsToModuleMeasurements() {
    RecordingModuleIO io = new RecordingModuleIO();
    io.drivePositionRad = 4.0;
    io.driveVelocityRadPerSec = 10.0;
    io.turnPosition = Rotation2d.fromDegrees(30.0);
    io.odometryTimestamps = new double[] {1.0, 1.02};
    io.odometryDrivePositionsRad = new double[] {2.0, 3.0};
    io.odometryTurnPositions =
        new Rotation2d[] {Rotation2d.fromDegrees(10.0), Rotation2d.fromDegrees(20.0)};
    Module module = new Module(io, 0, constants());

    module.periodic();

    assertEquals(0.2, module.getPositionMeters(), DELTA);
    assertEquals(0.5, module.getVelocityMetersPerSec(), DELTA);
    assertEquals(30.0, module.getAngle().getDegrees(), DELTA);
    assertEquals(4.0, module.getWheelRadiusCharacterizationPosition(), DELTA);
    assertEquals(10.0 / (2.0 * Math.PI), module.getFFCharacterizationVelocity(), DELTA);
    assertArrayEquals(io.odometryTimestamps, module.getOdometryTimestamps(), DELTA);

    SwerveModulePosition[] odometry = module.getOdometryPositions();
    assertEquals(2, odometry.length);
    assertEquals(0.1, odometry[0].distanceMeters, DELTA);
    assertEquals(10.0, odometry[0].angle.getDegrees(), DELTA);
    assertEquals(0.15, odometry[1].distanceMeters, DELTA);
    assertEquals(20.0, odometry[1].angle.getDegrees(), DELTA);
  }

  @Test
  void runSetpointConvertsLinearVelocityToWheelAngularVelocity() {
    RecordingModuleIO io = new RecordingModuleIO();
    Module module = new Module(io, 0, constants());
    module.periodic();

    module.runSetpoint(new SwerveModuleState(2.0, Rotation2d.kZero));

    assertEquals(40.0, io.driveVelocityRadPerSec, DELTA);
    assertEquals(1, io.turnPositionSetCount);
    assertEquals(0.0, io.requestedTurnPosition.getRadians(), DELTA);
  }

  @Test
  void characterizationAndStopDelegateExpectedOutputs() {
    RecordingModuleIO io = new RecordingModuleIO();
    Module module = new Module(io, 0, constants());

    module.runCharacterization(3.5);
    assertEquals(3.5, io.driveOpenLoop, DELTA);
    assertEquals(1, io.turnPositionSetCount);
    assertEquals(0.0, io.requestedTurnPosition.getRadians(), DELTA);

    module.stop();
    assertEquals(0.0, io.driveOpenLoop, DELTA);
    assertEquals(1, io.turnOpenLoopSetCount);
    assertEquals(0.0, io.turnOpenLoop, DELTA);
  }

  @Test
  void driveModeAndCurrentLimitMethodsDelegateToIo() {
    RecordingModuleIO io = new RecordingModuleIO();
    Module module = new Module(io, 0, constants());

    module.setBrakeMode();
    module.setCoastMode();
    module.setHighCurrentLimits();
    module.setLowCurrentLimits();

    assertEquals(1, io.brakeCount);
    assertEquals(1, io.coastCount);
    assertEquals(1, io.highCurrentCount);
    assertEquals(1, io.lowCurrentCount);
  }

  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants() {
    return new SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
        .withWheelRadius(WHEEL_RADIUS_METERS);
  }

  private static class RecordingModuleIO implements ModuleIO {
    double drivePositionRad;
    double driveVelocityRadPerSec;
    Rotation2d turnPosition = Rotation2d.kZero;
    double[] odometryTimestamps = new double[0];
    double[] odometryDrivePositionsRad = new double[0];
    Rotation2d[] odometryTurnPositions = new Rotation2d[0];

    double driveOpenLoop;
    double turnOpenLoop;
    Rotation2d requestedTurnPosition = Rotation2d.kZero;
    int turnOpenLoopSetCount;
    int turnPositionSetCount;
    int brakeCount;
    int coastCount;
    int highCurrentCount;
    int lowCurrentCount;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.driveConnected = true;
      inputs.turnConnected = true;
      inputs.turnEncoderConnected = true;
      inputs.drivePositionRad = drivePositionRad;
      inputs.driveVelocityRadPerSec = driveVelocityRadPerSec;
      inputs.turnPosition = turnPosition;
      inputs.odometryTimestamps = odometryTimestamps;
      inputs.odometryDrivePositionsRad = odometryDrivePositionsRad;
      inputs.odometryTurnPositions = odometryTurnPositions;
    }

    @Override
    public void setDriveOpenLoop(double output) {
      driveOpenLoop = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
      turnOpenLoop = output;
      turnOpenLoopSetCount++;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
      driveVelocityRadPerSec = velocityRadPerSec;
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
      requestedTurnPosition = rotation;
      turnPositionSetCount++;
    }

    @Override
    public void setBrakeMode() {
      brakeCount++;
    }

    @Override
    public void setCoastMode() {
      coastCount++;
    }

    @Override
    public void setHighCurrentLimits() {
      highCurrentCount++;
    }

    @Override
    public void setLowCurrentLimits() {
      lowCurrentCount++;
    }
  }
}
