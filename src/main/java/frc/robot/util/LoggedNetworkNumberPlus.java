package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedNetworkNumberPlus extends LoggedNetworkNumber {
  private double lastValue = get();
  private List<DoubleConsumer> subscribers = new ArrayList<DoubleConsumer>();

  public LoggedNetworkNumberPlus(String key, double defaultValue) {
    super(key, defaultValue);
  }

  public LoggedNetworkNumberPlus(String key) {
    super(key);
  }

  public void addSubscriber(DoubleConsumer subscriber) {
    subscribers.add(subscriber);
  }

  public void removeSubscriber(DoubleConsumer subscriber) {
    subscribers.remove(subscriber);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (subscribers.size() == 0)
      return; // If nobody is listening, we don't care about checking the value

    double currentValue = get();
    if (lastValue != currentValue) {
      for (DoubleConsumer subscriber : subscribers) {
        subscriber.accept(currentValue);
      }
    }
    lastValue = currentValue;
  }
}
