package frc.robot.commands.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.function.BooleanSupplier;

/** Command to run the boot animation on the LED strips */
public class LEDProgressBarCommand extends Command {

  private final BooleanSupplier[] booleanArraySupplier;
  private final LEDSubsystem ledSubsystem;

  /**
   * Creates a new LEDProgressBarCommand
   * 
   * @param ledSubsystem LED Subsystem
   */
  public LEDProgressBarCommand(LEDSubsystem ledSubsystem, BooleanSupplier[] booleanArraySupplier) {
    this.ledSubsystem = ledSubsystem;
    this.booleanArraySupplier = booleanArraySupplier;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ledSubsystem.setLEDSegments(Color.kBlue, booleanArraySupplier);
  }

  @Override
  public boolean isFinished() {
    return !RobotState.isTest();
  }

  @Override
  public void end(boolean interrupted) {
    ledSubsystem.runPattern(LEDPattern.kOff);
  }
}