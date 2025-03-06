package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Arrays;

/**
 * Command that runs in the background to schedule a default command for a set of commands. This will run a provided
 * command when no other commands are running on all of the specified subsystems. The intent is that you have a default
 * command for the set of subsystems. The provided command must require all of the subsystems.
 */
public class DefaultSchedulerCommand extends Command {

  private final Subsystem[] subsystems;
  private final Command defaultCommand;

  /**
   * Constructs a new DefaultSchedulerCommand
   * 
   * @param defaultCommand command to schedule when all of the subsystems have no command running. This command must
   *          require all of the subsystems.
   * @param subsystems when no command is running on all these subsystems, defaultCommand will be scheduled
   */
  public DefaultSchedulerCommand(Command defaultCommand, Subsystem... subsystems) {
    this.subsystems = subsystems;
    this.defaultCommand = defaultCommand;

    if (!defaultCommand.getRequirements().containsAll(Arrays.asList(subsystems))) {
      throw new IllegalArgumentException("Default commands must require their subsystem!");
    }
  }

  @Override
  public void execute() {
    if (Arrays.stream(subsystems).allMatch(subsystem -> subsystem.getCurrentCommand() == null)) {
      // None of the subsystems are running a command, so schedule the default command
      defaultCommand.schedule();
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    // Runs when disabled so it doesn't have to be restarted
    return true;
  }
}
