package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * A GamePieceManipulatorCommands uses the GamepieceMainpulatorSubsytems
 */
public class GamePieaceManipulator_Eject extends Command {
  // The Subsystem the command runs on.
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;

  public GamePieaceManipulator_Eject(GamePieceManipulatorSubsystem manipulator) {
    this.gamePieceManipulatorSubsystem = manipulator;
    addRequirements(gamePieceManipulatorSubsystem);
  }

  @Override
  public void initialize() {
    gamePieceManipulatorSubsystem.ejectCoral();

  }

  @Override
  public boolean isFinished() {
    return true;
  }

}
