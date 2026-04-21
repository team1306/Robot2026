package frc.robot.subsystems.leds;

import badgerutils.statemachine.Edges;
import badgerutils.statemachine.StateMachine;
import badgerutils.triggers.AllianceTriggers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {

  public final LedsIO LedsIO;
  public boolean isShooting = false;
  public boolean isInShootingTolerance = false;
  private StateMachine<LedState> stateMachine;

  public enum LedState {
    DISABLED,
    AUTO,
    AIMING,
    SHOOTING,
    OTHER,
    NONE,
  }

  public Leds(LedsIO LedsIO) {
    this.LedsIO = LedsIO;
  }

  @Override
  public void periodic() {
    Edges<LedState> edges =
        new Edges<LedState>()
            .anyToState(LedState.DISABLED, state -> LedsIO.setRainbow(100))
            .anyToState(LedState.AUTO, state -> LedsIO.setSolid(255, 0, 255))
            .anyToState(LedState.AIMING, state -> LedsIO.setSolid(0, 255, 0))
            .anyToState(LedState.SHOOTING, state -> LedsIO.setBounce(0, 255, 0, 80))
            .anyToState(
                LedState.OTHER,
                AllianceTriggers.isRedAlliance()
                    ? state -> LedsIO.setSolid(255, 0, 0)
                    : state -> LedsIO.setSolid(0, 0, 255));

    stateMachine = new StateMachine<>(LedState.NONE, edges);
    if (DriverStation.isDisabled()) {
      stateMachine.tryChangeState(LedState.DISABLED);
    } else if (DriverStation.isAutonomous()) {
      stateMachine.tryChangeState(LedState.AUTO);
    } else if (isShooting && !isInShootingTolerance) {
      stateMachine.tryChangeState(LedState.AIMING);
    } else if (isShooting) {
      stateMachine.tryChangeState(LedState.SHOOTING);
    } else {
      stateMachine.tryChangeState(LedState.OTHER);
    }
    Logger.recordOutput("Leds/state", stateMachine.getCurrentState());
    Logger.recordOutput("Leds/isShooting", isShooting);
    Logger.recordOutput("Leds/isInShootingTolerance", isInShootingTolerance);
  }
}
