package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * A GamePieceManipulatorCommands uses the GamepieceMainpulatorSubsytems to score the coral.
 */
public class ScoreCoralCommand extends Command {
  // The Subsystem the command runs on.
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;

  public ScoreCoralCommand(GamePieceManipulatorSubsystem manipulator) {
    this.gamePieceManipulatorSubsystem = manipulator;
    addRequirements(gamePieceManipulatorSubsystem);
  }

  @Override
  public void execute() {
    gamePieceManipulatorSubsystem.scoreCoral();

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
