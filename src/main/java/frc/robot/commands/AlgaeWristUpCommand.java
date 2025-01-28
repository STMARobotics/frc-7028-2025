package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * Moves the wrist up to hold algae
 */
public class AlgaeWristUpCommand extends Command {
  private final AlgaeSubsystem algaeSubsystem;

  public AlgaeWristUpCommand(AlgaeSubsystem algaeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    addRequirements(algaeSubsystem);
  }

  @Override
  public void execute() {
    algaeSubsystem.moveIntakeUp();
  }

}