package unittests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.StatusCode;
import frc.robot.util.PhoenixUtil;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class PhoenixUtilTest {
  @Test
  void stopsRetryingAfterSuccess() {
    AtomicInteger attempts = new AtomicInteger();

    PhoenixUtil.tryUntilOk(
        5, () -> attempts.incrementAndGet() == 3 ? StatusCode.OK : StatusCode.SimDeviceNotFound);

    assertEquals(3, attempts.get());
  }

  @Test
  void stopsAtMaximumAttemptsWhenCommandKeepsFailing() {
    AtomicInteger attempts = new AtomicInteger();

    PhoenixUtil.tryUntilOk(
        4,
        () -> {
          attempts.incrementAndGet();
          return StatusCode.SimDeviceNotFound;
        });

    assertEquals(4, attempts.get());
  }

  @Test
  void doesNotRunCommandWhenMaximumAttemptsIsNonpositive() {
    AtomicInteger attempts = new AtomicInteger();

    PhoenixUtil.tryUntilOk(0, () -> successfulAttempt(attempts));
    PhoenixUtil.tryUntilOk(-1, () -> successfulAttempt(attempts));

    assertEquals(0, attempts.get());
  }

  private static StatusCode successfulAttempt(AtomicInteger attempts) {
    attempts.incrementAndGet();
    return StatusCode.OK;
  }
}
