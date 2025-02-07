package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * Intakes the coral.
 */
public class IntakeCoralCommand extends Command {
  // The Subsystem the command runs on.
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;

  /**
   * The construcnter of the command and tell the perimeteres.
   * 
   * @param manipulator
   */
  public IntakeCoralCommand(GamePieceManipulatorSubsystem manipulator, ArmSubsystem arm) {
    this.gamePieceManipulatorSubsystem = manipulator;
    addRequirements(gamePieceManipulatorSubsystem);
    this.armSubsystem = arm;
    addRequirements(armSubsystem);
  }

  /**
   * Move the arm to intake postition, once.
   */
  @Override
  public void initialize() {
    armSubsystem.moveArmToIntake();

  }

  /**
   * Allow the Game Pieace Manipulator to grab the coral.
   */
  @Override
  public void execute() {
    gamePieceManipulatorSubsystem.intakeCoral();

  }

  @Override
  public boolean isFinished() {
    return gamePieceManipulatorSubsystem.isCoralInPickupPosition();
  }

  /**
   * Stops and resets the Inject command.
   */
  @Override
  public void end(boolean interupted) {
    armSubsystem.moveElevatorToDefault();
    gamePieceManipulatorSubsystem.stop();
  }

}
