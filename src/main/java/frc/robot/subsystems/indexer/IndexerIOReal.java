package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {

  private final TalonFX leftIndexerMotor;
  private final TalonFX rightIndexerMotor;

  public IndexerIOReal() {
    leftIndexerMotor = new TalonFX(Constants.CanIds.INDEXER_LEFT_MOTOR_ID);
    rightIndexerMotor = new TalonFX(Constants.CanIds.INDEXER_RIGHT_MOTOR_ID);

    leftIndexerMotor.getConfigurator().apply(IndexerConstants.CW_INDEXER_MOTOR_CONFIGS);
    rightIndexerMotor.getConfigurator().apply(IndexerConstants.CCW_INDEXER_MOTOR_CONFIGS);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.leftVelocity = leftIndexerMotor.getVelocity().getValue();
    inputs.rightVelocity = rightIndexerMotor.getVelocity().getValue();

    inputs.isLeftMotorConnected = leftIndexerMotor.isConnected();
    inputs.isRightMotorConnected = rightIndexerMotor.isConnected();

    inputs.leftTemp = leftIndexerMotor.getDeviceTemp().getValue();
    inputs.rightTemp = rightIndexerMotor.getDeviceTemp().getValue();

    inputs.leftSupplyCurrent = leftIndexerMotor.getSupplyCurrent().getValue();
    inputs.rightSupplyCurrent = rightIndexerMotor.getSupplyCurrent().getValue();
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    DutyCycleOut request = new DutyCycleOut(dutyCycle).withEnableFOC(true);
    leftIndexerMotor.setControl(request);
    rightIndexerMotor.setControl(request);
  }
}
