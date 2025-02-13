package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Command to eject the coral from the robot.
 */
public class EjectCoralCommand extends Command {
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  /**
   * Constructs a new EjectCoralCommand
   * 
   * @param manipulatorSubsytem manipulator subsystem
   * @param armSubsystem arm subsystem
   * @param indexerSubsystem indexer subsystem
   */
  public EjectCoralCommand(
      GamePieceManipulatorSubsystem manipulatorSubsytem,
      ArmSubsystem armSubsystem,
      IndexerSubsystem indexerSubsystem) {
    this.gamePieceManipulatorSubsystem = manipulatorSubsytem;
    this.armSubsystem = armSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(manipulatorSubsytem, armSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.moveToCoralIntakePosition();
  }

  @Override
  public void execute() {
    armSubsystem.moveToCoralIntakePosition();
    indexerSubsystem.eject();
    if (armSubsystem.isAtPosition()) {
      gamePieceManipulatorSubsystem.ejectCoral();
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
    gamePieceManipulatorSubsystem.stop();
    indexerSubsystem.stop();
  }
}