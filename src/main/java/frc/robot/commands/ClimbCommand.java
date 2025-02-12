package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Command to run the climb
 */
public class ClimbCommand extends Command {

  private final DoubleSupplier frontClimbSpeed;
  private final DoubleSupplier backClimbSpeed;

  private final ClimbSubsystem climbSubsystem;

  /**
   * This command sets the local variables climbSubsystem, frontClimbSpeed, and backClimbSpeed equal to their instance
   * variables
   * 
   * @param climbSubsystem climb subsystem
   * @param frontClimbSpeed Double supplier that determines the speed the front climb will move at; in the range of
   *          [-1, 1]
   * @param backClimbSpeed Double supplier that determines the speed the back climb will move at; in the range of
   *          [-1, 1]
   */
  public ClimbCommand(ClimbSubsystem climbSubsystem, DoubleSupplier frontClimbSpeed, DoubleSupplier backClimbSpeed) {

    this.frontClimbSpeed = frontClimbSpeed;
    this.backClimbSpeed = backClimbSpeed;

    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void execute() {
    climbSubsystem.runClimb(frontClimbSpeed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stop();
  }

}
