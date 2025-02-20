package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Command to move the arm to a reef scoring position
 */
public class MoveArmToReefLevel4Command extends Command {

  private final ArmSubsystem armSubsystem;

  /**
   * Constructs a new MoveArmToReefLevel4Command
   * 
   * @param armSubsystem arm subsystem
   */
  public MoveArmToReefLevel4Command(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;

    addRequirements(armSubsystem);
  }

  @Override
  public void execute() {
    armSubsystem.moveToLevel4();
  }

  @Override
  public boolean isFinished() {
    return armSubsystem.isAtPosition();
  }

}
