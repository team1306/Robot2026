package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IndexerIOReal implements IndexerIO {

  private final TalonFX indexerMotor;

  public IndexerIOReal() {
    indexerMotor = new TalonFX(IndexerConstants.indexerMotorID);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.velocity = indexerMotor.getVelocity().getValue();
    inputs.isConnected = indexerMotor.isConnected();
    inputs.temp = indexerMotor.getDeviceTemp().getValue();
  }

  @Override
  public void setPower(double dutyCycle) {
    indexerMotor.setControl((new DutyCycleOut(dutyCycle)).withEnableFOC(true));
  }
}
