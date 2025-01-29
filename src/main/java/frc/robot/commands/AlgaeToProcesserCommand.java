package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * Moves the intake to the position to score in the processer
 */
public class AlgaeToProcesserCommand extends Command {
  private final AlgaeSubsystem _algaeSubsystem;

  public AlgaeToProcesserCommand(AlgaeSubsystem algaeSubsystem) {
    this._algaeSubsystem = algaeSubsystem;
    addRequirements(_algaeSubsystem);
  }

  public void initialize() {
    _algaeSubsystem.moveToProcessorPosition();
  }

  @Override
  public void execute() {

  }

}
