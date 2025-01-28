package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * A GamePieceManipulatorCommands uses the GamepieceMainpulatorSubsytems and hold the coral in postion or fixes position
 */
public class ActiveHoldCoralCommand extends Command {
  // The Subsystem the command runs on.
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;

  /**
   * Hold the coral in the Game Pieace Maniupltor.
   * 
   * @param manipulator
   */
  public ActiveHoldCoralCommand(GamePieceManipulatorSubsystem manipulator) {
    this.gamePieceManipulatorSubsystem = manipulator;
    addRequirements(gamePieceManipulatorSubsystem);
  }

  @Override
  public void execute() {
    gamePieceManipulatorSubsystem.activeHoldGamePiece();

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    gamePieceManipulatorSubsystem.stop();
  }
}
