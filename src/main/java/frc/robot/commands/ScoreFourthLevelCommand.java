package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreFourthLevelCommand extends Command {

  private final ArmSubsystem armSubsystem;

  /**
   * Command to score the coral on the fourth level of the reef
   */
  public ScoreFourthLevelCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;

    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.moveArmToLevel2();
    armSubsystem.moveElevatorLevel2();
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