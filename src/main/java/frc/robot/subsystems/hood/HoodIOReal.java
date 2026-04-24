package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Rotations;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;
import frc.robot.util.LoggedNetworkNumberPlus;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.AutoLogOutput;

public class HoodIOReal implements HoodIO {
  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KP_SUPPLIER =
      new LoggedNetworkNumberPlus("Tuning/Hood KP", HoodConstants.KP);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KD_SUPPLIER =
      new LoggedNetworkNumberPlus("Tuning/Hood KD", HoodConstants.KD);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KS_SUPPLIER =
      new LoggedNetworkNumberPlus("Tuning/Hood KS", HoodConstants.KS);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KG_SUPPLIER =
      new LoggedNetworkNumberPlus("Tuning/Hood KG", HoodConstants.KG);

  private final TalonFX motor;

  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<Current> motorSupplyCurrent;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<Temperature> motorTemperature;

  private final PositionTorqueCurrentFOC positionRequest;

  public HoodIOReal() {
    motor = new TalonFX(Constants.CanIds.HOOD_MOTOR_ID);
    motor.getConfigurator().apply(HoodConstants.HOOD_MOTOR_CONFIGS);

    motorVelocity = motor.getVelocity();
    motorPosition = motor.getPosition();
    motorSupplyCurrent = motor.getSupplyCurrent();
    motorStatorCurrent = motor.getStatorCurrent();
    motorTemperature = motor.getDeviceTemp();

    positionRequest = new PositionTorqueCurrentFOC(0);

    DoubleConsumer applySlot0 =
        value ->
            motor
                .getConfigurator()
                .apply(
                    MotorConfigUtils.createPidConfig(
                        KP_SUPPLIER.get(),
                        0,
                        KD_SUPPLIER.get(),
                        KS_SUPPLIER.get(),
                        0,
                        KG_SUPPLIER.get(),
                        0,
                        GravityTypeValue.Arm_Cosine));

    KP_SUPPLIER.addSubscriber(applySlot0);
    KD_SUPPLIER.addSubscriber(applySlot0);
    KS_SUPPLIER.addSubscriber(applySlot0);
    KG_SUPPLIER.addSubscriber(applySlot0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    StatusCode motorStatus =
        BaseStatusSignal.refreshAll(
            motorVelocity, motorPosition, motorSupplyCurrent, motorStatorCurrent, motorTemperature);

    inputs.isMotorConnected = motorStatus.isOK();
    inputs.motorVelocity = motorVelocity.getValue();
    inputs.motorPosition = motorPosition.getValue();
    inputs.motorSupplyCurrent = motorSupplyCurrent.getValue();
    inputs.motorStatorCurrent = motorStatorCurrent.getValue();
    inputs.motorTemperature = motorTemperature.getValue();
  }

  @Override
  public void setAngle(Angle angle) {
    angle =
        Rotations.of(
            MathUtil.clamp(
                angle.in(Rotations),
                HoodConstants.ZERO_POSITION.in(Rotations),
                HoodConstants.MAX_ANGLE.in(Rotations)));
    motor.setControl(positionRequest.withPosition(angle));
  }
}
