package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DutyCycleOut dutyCycle;

  private final TalonFX latchMotor;
  private final StatusSignal<Current> latchCurrentSignal;
  private final PositionTorqueCurrentFOC latchPositionRequest;

  private final StatusSignal<Current> leftCurrentSignal;
  private final StatusSignal<Current> rightCurrentSignal;

  public IntakeIOReal() {
    leftMotor = new TalonFX(IntakeConstants.INTAKE_LEFT_MOTOR_ID);
    rightMotor = new TalonFX(IntakeConstants.INTAKE_RIGHT_MOTOR_ID);

    leftCurrentSignal = leftMotor.getSupplyCurrent();
    rightCurrentSignal = rightMotor.getSupplyCurrent();

    latchMotor = new TalonFX(IntakeConstants.LATCH_MOTOR_ID);
    latchCurrentSignal = latchMotor.getSupplyCurrent();
    latchPositionRequest = new PositionTorqueCurrentFOC(Degrees.of(0));

    dutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isLeftMotorConnected = leftMotor.isConnected();
    inputs.isRightMotorConnected = rightMotor.isConnected();
    inputs.isLatchMotorConnected = latchMotor.isConnected();
    inputs.leftVelocity = leftMotor.getVelocity().getValue();
    inputs.rightVelocity = rightMotor.getVelocity().getValue();
    inputs.latchPosition = latchMotor.getPosition().getValue();
    inputs.leftTemp = leftMotor.getDeviceTemp().getValue();
    inputs.rightTemp = rightMotor.getDeviceTemp().getValue();
    inputs.latchTemp = latchMotor.getDeviceTemp().getValue();

    BaseStatusSignal.refreshAll(leftCurrentSignal, rightCurrentSignal, latchCurrentSignal);
    inputs.leftSupplyCurrent = leftCurrentSignal.getValue();
    inputs.rightSupplyCurrent = rightCurrentSignal.getValue();
    inputs.latchCurrent = latchCurrentSignal.getValue();
  }

  @Override
  public void set(double dutyCycle) {
    leftMotor.setControl(this.dutyCycle.withOutput(dutyCycle));
    rightMotor.setControl(this.dutyCycle.withOutput(dutyCycle));
  }

  @Override
  public void setDeployerPosition(Angle angle) {
    latchMotor.setControl(latchPositionRequest.withPosition(angle));
  }
}
