package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * Command to eject the coral from the robot.
 */
public class EjectCoralCommand extends Command {
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;

  /**
   * Constructs a new EjectCoralCommand
   * 
   * @param manipulatorSubsytem manipulator subsystem
   * @param armSubsystem arm subsystem
   */
  public EjectCoralCommand(GamePieceManipulatorSubsystem manipulatorSubsytem, ArmSubsystem armSubsystem) {
    this.gamePieceManipulatorSubsystem = manipulatorSubsytem;
    this.armSubsystem = armSubsystem;
    addRequirements(gamePieceManipulatorSubsystem, armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.moveElevatorToDefault();
    armSubsystem.moveArmToIntake();
  }

  @Override
  public void execute() {
    gamePieceManipulatorSubsystem.ejectCoral();

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveElevatorToDefault();
    armSubsystem.moveArmToIntake();
    gamePieceManipulatorSubsystem.stop();
  }
}
