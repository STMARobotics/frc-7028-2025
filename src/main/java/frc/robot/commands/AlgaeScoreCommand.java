package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * Ejects algae into processer
 */
public class AlgaeScoreCommand extends Command {
  private final AlgaeSubsystem algaeSubsystem;

  public AlgaeScoreCommand(AlgaeSubsystem algaeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    addRequirements(algaeSubsystem);
  }

  /**
   * make sure to add that it will only score if intake is in position
   */
  @Override
  public void execute() {
    algaeSubsystem.dropAlgaeToProcess();
  }

  @Override
  public void end(boolean interrupted) {
    algaeSubsystem.stop();
  }

}