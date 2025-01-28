package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * Ejects currently held algae
 */
public class AlgaeEjectCommand extends Command {
  private final AlgaeSubsystem algaeSubsystem;

  public AlgaeEjectCommand(AlgaeSubsystem algaeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    addRequirements(algaeSubsystem);
  }

  public void initialize() {

  }

  @Override
  public void execute() {
    algaeSubsystem.eject();
  }

}