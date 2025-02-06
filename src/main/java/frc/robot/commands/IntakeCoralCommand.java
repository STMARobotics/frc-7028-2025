package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Runs the indexer belt and the manipulator wheels to transport the coral to the manipulator
 */
public class IntakeCoralCommand extends Command {

  private final IndexerSubsystem indexerSubsystem;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;

  /**
   * Constructor for the IntakeCoralCommand
   *
   * @param indexerSubsystem indexer subsystem
   * @param gamePieceManipulatorSubsystem manipulator subsystem
   */
  public IntakeCoralCommand(
      IndexerSubsystem indexerSubsystem,
      GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem) {
    this.indexerSubsystem = indexerSubsystem;
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;

    addRequirements(indexerSubsystem, gamePieceManipulatorSubsystem);
  }

  @Override
  public void execute() {
    indexerSubsystem.intake();
    gamePieceManipulatorSubsystem.intakeCoral();
  }

  @Override
  public boolean isFinished() {
    return gamePieceManipulatorSubsystem.isCoralInPickupPosition();
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    gamePieceManipulatorSubsystem.stop();
  }
}
