package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * Brings algae intake down and runs rollers
 */
public class AlgaeIntakeCommand extends Command {
  private final AlgaeSubsystem algaeSubsystem;

  public AlgaeIntakeCommand(AlgaeSubsystem algaeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    addRequirements(algaeSubsystem);
  }

  public void initialize() {
    algaeSubsystem.moveIntakeDown();
  }

  @Override
  public void execute() {
    algaeSubsystem.intake();
  }

}