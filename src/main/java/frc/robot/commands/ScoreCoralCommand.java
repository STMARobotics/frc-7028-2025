package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * A GamePieceManipulatorCommands uses the GamepieceMainpulatorSubsytems to score the coral.
 */
public class ScoreCoralCommand extends Command {
  // The Subsystem the command runs on.
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;

  public ScoreCoralCommand(GamePieceManipulatorSubsystem manipulator, ArmSubsystem arm) {
    this.gamePieceManipulatorSubsystem = manipulator;
    addRequirements(gamePieceManipulatorSubsystem);

    this.armSubsystem = arm;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.moveElevatorLevel2();
  }

  @Override
  public void execute() {
    gamePieceManipulatorSubsystem.scoreCoral();
    armSubsystem.moveArmToLevel2();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    gamePieceManipulatorSubsystem.stop();
  }
}
