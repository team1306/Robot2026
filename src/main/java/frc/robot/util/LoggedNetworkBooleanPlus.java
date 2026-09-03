package frc.robot.util;

import edu.wpi.first.util.function.BooleanConsumer;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class LoggedNetworkBooleanPlus extends LoggedNetworkBoolean {
  private boolean lastValue = get();
  private List<BooleanConsumer> subscribers = new ArrayList<BooleanConsumer>();

  public LoggedNetworkBooleanPlus(String key, boolean defaultValue) {
    super(key, defaultValue);
  }

  public LoggedNetworkBooleanPlus(String key) {
    super(key);
  }

  public void addSubscriber(BooleanConsumer subscriber) {
    subscribers.add(subscriber);
  }

  public void removeSubscriber(BooleanConsumer subscriber) {
    subscribers.remove(subscriber);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (subscribers.size() == 0)
      return; // If nobody is listening, we don't care about checking the value

    boolean currentValue = get();
    if (lastValue != currentValue) {
      for (BooleanConsumer subscriber : subscribers) {
        subscriber.accept(currentValue);
      }
    }
    lastValue = currentValue;
  }
}
