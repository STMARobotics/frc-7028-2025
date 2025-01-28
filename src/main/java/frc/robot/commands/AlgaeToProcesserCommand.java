package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * Moves the intake to the position to score in the processer
 */
public class AlgaeToProcesserCommand extends Command {
  private final AlgaeSubsystem algaeSubsystem;

  public AlgaeToProcesserCommand(AlgaeSubsystem algaeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    addRequirements(algaeSubsystem);
  }

  public void initialize() {
    algaeSubsystem.moveToProcessorPosition();
  }

  @Override
  public void execute() {

  }

}