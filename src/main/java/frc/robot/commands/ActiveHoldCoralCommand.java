package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * Runs the motor enough to secure the coral.
 */
public class ActiveHoldCoralCommand extends Command {
  // The Subsystem the command runs on.
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;

  /**
   * The construcnter of the command and tell the perimeteres.
   * 
   * @param manipulator manipulator subsystem
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

    gamePieceManipulatorSubsystem.activeHoldGamePiece();
    armSubsystem.moveElevatorToDefault();
    ;

  }

  @Override
  public boolean isFinished() {
    return gamePieceManipulatorSubsystem.isCoralInPickupPosition();

  }

  /**
   * Stops and resets the command.
   */
  @Override
  public void end(boolean interrupted) {
    gamePieceManipulatorSubsystem.stop();
  }
}
