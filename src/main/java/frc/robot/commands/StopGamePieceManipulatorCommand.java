package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * A GamePieceManipulatorCommands uses the GamepieceMainpulatorSubsytems stops the Game Pieace Manipulator.
 */
public class StopGamePieceManipulatorCommand extends Command {
  // The Subsystem the command runs on.
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;

  /**
   * Command to stop the Game Pieace Manipulator.
   * 
   * @param manipulator
   */
  public StopGamePieceManipulatorCommand(GamePieceManipulatorSubsystem manipulator) {
    this.gamePieceManipulatorSubsystem = manipulator;
    addRequirements(gamePieceManipulatorSubsystem);
  }

  @Override
  public void execute() {
    gamePieceManipulatorSubsystem.stop();

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
