package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean isMotorConnected;
    public AngularVelocity motorVelocity;
    public Angle motorPosition;
    public Current motorSupplyCurrent;
    public Current motorStatorCurrent;
    public Temperature motorTemperature;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setAngle(Angle angle) {}
}
