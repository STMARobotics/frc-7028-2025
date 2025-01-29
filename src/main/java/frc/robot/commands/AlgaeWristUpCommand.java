package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * Moves the wrist up to hold algae
 */
public class AlgaeWristUpCommand extends Command {
  private final AlgaeSubsystem _algaeSubsystem;

  public AlgaeWristUpCommand(AlgaeSubsystem algaeSubsystem) {
    this._algaeSubsystem = algaeSubsystem;
    addRequirements(_algaeSubsystem);
  }

  public void initialize() {
    _algaeSubsystem.moveIntakeUp();
  }

  @Override
  public void execute() {

  }

}
