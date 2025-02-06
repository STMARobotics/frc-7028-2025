package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * A GamePieceManipulatorCommands uses the GamepieceMainpulatorSubsytems and ejcets the coral.
 */
public class EjectCoralCommand extends Command {
  // The Subsystem the command runs on.
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;

  /**
   * The construcnter of the command and tell the perimeteres.
   * 
   * @param manipulator
   */
  public EjectCoralCommand(GamePieceManipulatorSubsystem manipulator, ArmSubsystem arm) {
    this.gamePieceManipulatorSubsystem = manipulator;
    addRequirements(gamePieceManipulatorSubsystem);

    this.armSubsystem = arm;
    addRequirements(armSubsystem);
  }

  /**
   * Allows the Game Piece Manipulator to let go of the coral.
   */
  @Override
  public void execute() {
    gamePieceManipulatorSubsystem.ejectCoral();

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Stops an dresets the commands.
   */
  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveElevatorToDefault();
    armSubsystem.moveArmToIntake();
    gamePieceManipulatorSubsystem.stop();
  }
}
