package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IndexerIOReal implements IndexerIO {

  private final TalonFX indexerMotor;
  private double targetVelocity;

  public IndexerIOReal() {
    indexerMotor = new TalonFX(IndexerConstants.indexerMotorID);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.currentVelocity = indexerMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void changeVelocity(double dutyCycle) {
    indexerMotor.setControl((new DutyCycleOut(0)).withOutput(targetVelocity).withEnableFOC(true));
    targetVelocity = dutyCycle;
  }
}
