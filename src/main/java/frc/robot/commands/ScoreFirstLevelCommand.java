package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreFirstLevelCommand extends Command {

  private final ArmSubsystem armSubsystem;

  /**
   * Command to score the coral on the first level of the reef
   */
  public ScoreFirstLevelCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;

    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.moveArmToIntake();
    armSubsystem.moveElevatorLevel1();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveArmToIntake();
    armSubsystem.moveElevatorToDefault();
  }
}