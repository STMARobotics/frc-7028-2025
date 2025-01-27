package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * Brings algae intake down and runs rollers
 */
public class AlgaeIntakeCommand extends Command {
  private final AlgaeSubsystem _algaeSubsystem;

  public AlgaeIntakeCommand(AlgaeSubsystem algaeSubsystem) {
    this._algaeSubsystem = algaeSubsystem;
    addRequirements(_algaeSubsystem);
  }

  public void initialize() {
    _algaeSubsystem.moveIntakeDown();
  }

  @Override
  public void execute() {
    _algaeSubsystem.intake();
  }

}
