package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedNetworkNumberPlus extends LoggedNetworkNumber {
  private double lastValue = get();
  private List<Runnable> listeners = new ArrayList<Runnable>();

  public LoggedNetworkNumberPlus(String key, double defaultValue) {
    super(key, defaultValue);
  }

  public LoggedNetworkNumberPlus(String key) {
    super(key);
  }

  public void addListener(Runnable listener) {
    listeners.add(listener);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (listeners.size() == 0)
      return; // If nobody is listening, we don't care about checking the value

    if (lastValue != get()) {
      for (Runnable listener : listeners) {
        listener.run();
      }
    }
    lastValue = get();
  }
}
