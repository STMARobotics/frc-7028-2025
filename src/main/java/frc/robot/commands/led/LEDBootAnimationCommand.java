package frc.robot.commands.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command that runs the boot animation on the LEDs
 */
public class LEDBootAnimationCommand extends Command {

  private final LEDSubsystem ledSubsystem;
  private final Timer timer = new Timer();

  private static final double RUN_TIME = 2.0;

  private LEDPattern pattern;

  /**
   * Creates a new LEDBootAnimationCommand
   * 
   * @param ledSubsystem LED Subsystem
   */
  public LEDBootAnimationCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    pattern = LEDPattern.progressMaskLayer(() -> timer.get() / RUN_TIME)
        .mask(LEDPattern.progressMaskLayer(() -> 1.2 - (timer.get() / RUN_TIME)).reversed())
        .mask(LEDPattern.solid(Color.kYellow))
        .overlayOn(LEDPattern.solid(Color.kBlue));
  }

  @Override
  public void execute() {
    ledSubsystem.runPattern(pattern);
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= RUN_TIME;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    ledSubsystem.runPattern(LEDPattern.kOff);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
