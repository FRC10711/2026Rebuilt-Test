package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

/**
 * SmashBump state machine command.
 *
 * <p>States:
 *
 * <ul>
 *   <li>ALIGN: align/setup before bump interaction
 *   <li>RUN: drive/run into/through bump
 *   <li>GOINGUP: climbing phase
 *   <li>LAND: landing/settling phase
 * </ul>
 */
public class SmashBumpCommand extends Command {
  public enum State {
    ALIGN,
    RUN,
    GOINGUP,
    LAND
  }

  @SuppressWarnings("unused")
  private final RobotContainer robot;

  private State state = State.ALIGN;

  public SmashBumpCommand(RobotContainer robot) {
    this.robot = robot;
    // TODO: addRequirements(robot.drive, ...);
  }

  @Override
  public void initialize() {
    setState(State.ALIGN);
  }

  @Override
  public void execute() {
    Logger.recordOutput("SmashBump/State", state.toString());
    switch (state) {
      case ALIGN -> align();
      case RUN -> runState();
      case GOINGUP -> goingUp();
      case LAND -> land();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("SmashBump/Interrupted", interrupted);
    // TODO: stop any subsystems (drive, etc.) if needed
  }

  private void setState(State next) {
    if (next != state) {
      state = next;
      Logger.recordOutput("SmashBump/StateEnter", state.toString());
    }
  }

  // --- State handlers (fill in later) ---
  private void align() {
    // TODO: alignment logic
    // setState(State.RUN);
  }

  private void runState() {
    // TODO: run logic
    // setState(State.GOINGUP);
  }

  private void goingUp() {
    // TODO: going up logic
    // setState(State.LAND);
  }

  private void land() {
    // TODO: landing logic
    // optionally setState(State.ALIGN) or finish
  }
}
