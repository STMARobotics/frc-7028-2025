package frc.robot.commands;

import frc.robot.subsystems.ClimbSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {

  private final DoubleSupplier frontClimbSpeed;
  private final DoubleSupplier backClimbSpeed;

  private final ClimbSubsystem climbSubsystem;

  public ClimbCommand(ClimbSubsystem climbSubsystem, DoubleSupplier frontClimbSpeed, DoubleSupplier backClimbSpeed) {

    this.frontClimbSpeed = frontClimbSpeed;
    this.backClimbSpeed = backClimbSpeed;

    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }
  
  //command to running climb motors
  @Override
  public void execute() {
    climbSubsystem.runFrontClimb(frontClimbSpeed.getAsDouble());
    climbSubsystem.runBackClimb(backClimbSpeed.getAsDouble());
  }

//command to stop running climb motors
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopMotors();
  }

}
