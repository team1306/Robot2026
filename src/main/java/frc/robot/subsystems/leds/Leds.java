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
    OTHER
  }

  public Leds(LedsIO LedsIO) {
    this.LedsIO = LedsIO;
  }

  @Override
  public void periodic() {
    Edges<LedState> edges =
        new Edges<LedState>()
            .anyToState(LedState.DISABLED, state -> LedsIO.setRainbow(4))
            .anyToState(LedState.AUTO, state -> LedsIO.setBlink(255, 0, 255, 4))
            .anyToState(LedState.AIMING, state -> LedsIO.setSolid(0, 150, 0))
            .anyToState(LedState.SHOOTING, state -> LedsIO.setBlink(0, 255, 0, 4))
            .anyToState(
                LedState.OTHER,
                state ->
                    LedsIO.setSolid(
                        AllianceTriggers.isRedAlliance() ? 150 : 0,
                        0,
                        AllianceTriggers.isRedAlliance() ? 0 : 150));

    stateMachine = new StateMachine<>(LedState.DISABLED, edges);
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

    Logger.recordOutput("Leds/isShooting", isShooting);
    Logger.recordOutput("Leds/isInShootingTolerance", isInShootingTolerance);
  }
}
