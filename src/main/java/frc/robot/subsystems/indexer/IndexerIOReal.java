package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<AngularVelocity> rightVelocity;

  private final StatusSignal<Temperature> leftTemp;
  private final StatusSignal<Temperature> rightTemp;

  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> rightSupplyCurrent;

  private final TalonFX leftIndexerMotor;
  private final TalonFX rightIndexerMotor;

  public IndexerIOReal() {
    leftIndexerMotor = new TalonFX(Constants.CanIds.INDEXER_LEFT_MOTOR_ID);
    rightIndexerMotor = new TalonFX(Constants.CanIds.INDEXER_RIGHT_MOTOR_ID);

    leftIndexerMotor.getConfigurator().apply(IndexerConstants.CW_INDEXER_MOTOR_CONFIGS);
    rightIndexerMotor.getConfigurator().apply(IndexerConstants.CCW_INDEXER_MOTOR_CONFIGS);

    leftVelocity = leftIndexerMotor.getVelocity();
    rightVelocity = rightIndexerMotor.getVelocity();

    leftTemp = leftIndexerMotor.getDeviceTemp();
    rightTemp = rightIndexerMotor.getDeviceTemp();

    leftSupplyCurrent = leftIndexerMotor.getSupplyCurrent();
    rightSupplyCurrent = rightIndexerMotor.getSupplyCurrent();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    StatusCode leftStatus = BaseStatusSignal.refreshAll(leftVelocity, leftTemp, leftSupplyCurrent);
    StatusCode rightStatus =
        BaseStatusSignal.refreshAll(rightVelocity, rightTemp, rightSupplyCurrent);

    inputs.isLeftMotorConnected = leftStatus.isOK();
    inputs.isRightMotorConnected = rightStatus.isOK();

    inputs.leftVelocity = leftVelocity.getValue();
    inputs.rightVelocity = rightVelocity.getValue();

    inputs.isLeftMotorConnected = leftIndexerMotor.isConnected();
    inputs.isRightMotorConnected = rightIndexerMotor.isConnected();

    inputs.leftTemp = leftTemp.getValue();
    inputs.rightTemp = rightTemp.getValue();

    inputs.leftSupplyCurrent = leftSupplyCurrent.getValue();
    inputs.rightSupplyCurrent = rightSupplyCurrent.getValue();
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    DutyCycleOut request = new DutyCycleOut(dutyCycle).withEnableFOC(true);
    leftIndexerMotor.setControl(request);
    rightIndexerMotor.setControl(request);
  }
}
