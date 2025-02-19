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

    pattern = overlayOn(
        LEDPattern.progressMaskLayer(() -> timer.get() / RUN_TIME * 1.2)
            .mask(LEDPattern.progressMaskLayer(() -> 1.2 - (timer.get() / RUN_TIME * 1.2)).reversed())
            .mask(LEDPattern.solid(Color.kYellow)),
          LEDPattern.solid(Color.kBlue));
  }

  /**
   * A version of {@link LEDPattern#overlayOn(LEDPattern)} that does what it says, only replacing BLACK LEDs.
   * 
   * @param base base pattern
   * @param overlay overlay pattern
   * @return new pattern with the base overlayed on the overlay
   */
  private LEDPattern overlayOn(LEDPattern base, LEDPattern overlay) {
    return (reader, writer) -> {
      // write the base pattern down first...
      base.applyTo(reader, writer);

      // ... then, overwrite with the illuminated LEDs from the overlay
      overlay.applyTo(reader, (i, r, g, b) -> {
        var led = reader.getLED(i);
        if ((led.blue == 0 && led.green == 0 && led.red == 0) && (r != 0 || g != 0 || b != 0)) {
          writer.setRGB(i, r, g, b);
        }
      });
    };
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
