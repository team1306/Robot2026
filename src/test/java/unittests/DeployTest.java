package unittests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.deploy.Deploy;
import frc.robot.subsystems.deploy.DeployIO;
import frc.robot.subsystems.deploy.DeployerPosition;
import org.junit.jupiter.api.Test;

class DeployTest {
  @Test
  void commandsReturnToExtendedPositionAndExposeSetpointState() {
    UnitTestDeployIO io = new UnitTestDeployIO();
    Deploy deploy = new Deploy(io);
    Command command = deploy.crunchCommand();

    command.initialize();
    assertSame(DeployerPosition.DUMP, io.position);
    command.end(false);
    assertSame(DeployerPosition.EXTENDED, io.position);

    assertFalse(deploy.isPastOrAtSetpoint());
    io.atSetpoint = true;
    assertTrue(deploy.isPastOrAtSetpoint());
    deploy.periodic();
    assertEquals(1, io.updateCount);
  }

  private static class UnitTestDeployIO implements DeployIO {
    DeployerPosition position;
    boolean atSetpoint;
    int updateCount;

    @Override
    public void updateInputs(DeployIOInputs inputs) {
      updateCount++;
    }

    @Override
    public void setPosition(DeployerPosition position) {
      this.position = position;
    }

    @Override
    public boolean isAtSetpoint() {
      return atSetpoint;
    }
  }
}
