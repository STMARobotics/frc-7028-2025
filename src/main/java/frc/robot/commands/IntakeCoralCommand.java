package frc.robot.commands;

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
  public void execute() {
    armSubsystem.moveToCoralIntakePosition();
    gamePieceManipulatorSubsystem.intakeCoral();
    if (armSubsystem.isElevatorAtPosition() && armSubsystem.isArmAtAngle()) {
      // Run the belt if the arm is in position
      indexerSubsystem.intake();
    }
  }

  @Override
  public boolean isFinished() {
    return indexerSubsystem.isCoralInPickupPosition();
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    gamePieceManipulatorSubsystem.stop();
    armSubsystem.stop();
  }
}