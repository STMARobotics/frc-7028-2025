package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Command to intake coral from the human player at the Coral Station.
 */
public class IntakeCoralCommand extends Command {

  private final IndexerSubsystem indexerSubsystem;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final Debouncer coralDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private boolean hasIndexerStartedMoving = false;

  /**
   * Constructors a IntakeCoralCommand
   *
   * @param indexerSubsystem indexer subsystem
   * @param gamePieceManipulatorSubsystem manipulator subsystem
   * @param armSubsystem arm subsystem
   */
  public IntakeCoralCommand(
      IndexerSubsystem indexerSubsystem,
      GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem,
      ArmSubsystem armSubsystem) {
    this.indexerSubsystem = indexerSubsystem;
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
    this.armSubsystem = armSubsystem;

    addRequirements(indexerSubsystem, gamePieceManipulatorSubsystem, armSubsystem);
  }

  @Override
  public void initialize() {
    coralDebouncer.calculate(false);
    hasIndexerStartedMoving = false;
  }

  @Override
  public void execute() {
    armSubsystem.moveToCoralIntakePosition();
    if (armSubsystem.isAtPosition()) {
      // Run the belt if the arm and game piece manipulator are in position
      indexerSubsystem.intake();
      gamePieceManipulatorSubsystem.intakeCoral();
    } else {
      indexerSubsystem.stop();
      gamePieceManipulatorSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    final var isIndexerMoving = coralDebouncer.calculate(indexerSubsystem.isIndexerMoving());
    if (hasIndexerStartedMoving && false == isIndexerMoving) {
      return true;
    }
    hasIndexerStartedMoving = isIndexerMoving;
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    gamePieceManipulatorSubsystem.stop();
    armSubsystem.stop();
  }
}