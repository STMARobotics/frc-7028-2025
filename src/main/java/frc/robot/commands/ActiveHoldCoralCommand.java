package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * A GamePieceManipulatorCommands uses the GamepieceMainpulatorSubsytems and hold the coral in postion or fixes position
 */
public class ActiveHoldCoralCommand extends Command {
  // The Subsystem the command runs on.
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;

  /**
   * The construcnter of the command and tell the perimeteres.
   * 
   * @param manipulator
   */
  public ActiveHoldCoralCommand(GamePieceManipulatorSubsystem manipulator, ArmSubsystem arm) {
    this.gamePieceManipulatorSubsystem = manipulator;
    addRequirements(gamePieceManipulatorSubsystem);

    this.armSubsystem = arm;
    addRequirements(armSubsystem);
  }

  /**
   * If the elevator and teh arm are in the right postion it allows it to hold the coral if not then stops.
   */
  @Override
  public void execute() {
    var isElevatorAtPosition = armSubsystem.isElevatorAtPosition();
    var isArmAtPosition = armSubsystem.isArmAtPosition();

    if (isElevatorAtPosition && isArmAtPosition) {
      gamePieceManipulatorSubsystem.activeHoldGamePiece();
    } else {
      gamePieceManipulatorSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Stops and resets the command.
   */
  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveElevatorToDefault();
    armSubsystem.moveArmToIntake();
    gamePieceManipulatorSubsystem.stop();
  }
}
