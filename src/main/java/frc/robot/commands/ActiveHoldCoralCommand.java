package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * Command to actively hold the coral in the game piece manipulator.
 */
public class ActiveHoldCoralCommand extends Command {
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;

  /**
   * Constructs a new ActiveHoldCommand
   * 
   * @param manipulatorSubsystem manipulator subsystem
   * @param armSubsystem arm subsystem
   */
  public ActiveHoldCoralCommand(GamePieceManipulatorSubsystem manipulatorSubsystem, ArmSubsystem armSubsystem) {
    this.gamePieceManipulatorSubsystem = manipulatorSubsystem;
    this.armSubsystem = armSubsystem;
    addRequirements(gamePieceManipulatorSubsystem, armSubsystem);
  }

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
    return gamePieceManipulatorSubsystem.isCoralInPickupPosition();

  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveElevatorToDefault();
    armSubsystem.moveArmToIntake();
    gamePieceManipulatorSubsystem.stop();
  }
}
