package frc.robot.subsystems.booster;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;

public class BoosterIOReal implements BoosterIO {
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Current> supplyCurrent;

  private final TalonFX motor;

  private final DutyCycleOut dutyCycleRequest;
  private final NeutralOut neutralRequest;

  public BoosterIOReal() {
    motor = new TalonFX(Constants.CanIds.BOOSTER_MOTOR_ID);
    motor.getConfigurator().apply(BoosterConstants.BOOSTER_MOTOR_CONFIGS);

    dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
    neutralRequest = new NeutralOut();

    velocity = motor.getVelocity();
    temp = motor.getDeviceTemp();
    supplyCurrent = motor.getSupplyCurrent();
  }

  @Override
  public void updateInputs(BoosterIOInputs inputs) {
    StatusCode status = BaseStatusSignal.refreshAll(velocity, temp, supplyCurrent);
    inputs.isMotorConnected = status.isOK();

    inputs.velocity = velocity.getValue();
    inputs.temp = temp.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    motor.setControl(dutyCycle == 0 ? neutralRequest : dutyCycleRequest.withOutput(0));
  }
}
