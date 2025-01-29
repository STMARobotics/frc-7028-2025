package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * Ejects algae into processer
 */
public class AlgaeScoreCommand extends Command {
  private final AlgaeSubsystem _algaeSubsystem;

  public AlgaeScoreCommand(AlgaeSubsystem algaeSubsystem) {
    this._algaeSubsystem = algaeSubsystem;
    addRequirements(_algaeSubsystem);
  }

  public void initialize() {

  }

  /**
   * make sure to add that it will only score if intake is in position
   */
  @Override
  public void execute() {
    _algaeSubsystem.dropAlgaeToProcess();
  }

}
