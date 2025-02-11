package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Default command for the arm
 */
public class DefaultArmCommand extends Command {

  private final ArmSubsystem armSubsystem;

  /**
   * Constructor
   * 
   * @param armSubsystem arm subsystem
   */
  public DefaultArmCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;

    addRequirements(armSubsystem);
  }

  @Override
  public void execute() {
    if (armSubsystem.isElevatorParked()) {
      // Turn the elevator off if it's parked
      armSubsystem.stopElevator();
    } else {
      armSubsystem.parkElevator();
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopElevator();
  }

}