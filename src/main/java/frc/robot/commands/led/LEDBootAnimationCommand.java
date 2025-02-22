package frc.robot.commands.led;

import static edu.wpi.first.wpilibj.util.Color.kBlack;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/** Command to run the boot animation on the LED strips */
public class LEDBootAnimationCommand extends Command {

  private static final int BLIP_SIZE = 5;
  private final LEDSubsystem ledSubsystem;
  private final Timer timer = new Timer();

  private int blipIndex = -1;
  private boolean done = false;
  private boolean initialized = false;

  public LEDBootAnimationCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    ledSubsystem.setCandyCane(kBlack, kBlack);
    blipIndex = -1;
    timer.start();
    done = false;
    initialized = false;
  }

  @Override
  public void execute() {
    if (timer.advanceIfElapsed(0.05) || !initialized) {
      if (!initialized) {
        ledSubsystem.setColor(kBlack);
        initialized = true;
      }
      for (int index = 0; index < ledSubsystem.getStripLength(); index++) {
        if (index <= blipIndex && index >= blipIndex - (BLIP_SIZE - 1)) {
          ledSubsystem.setLED(index, Color.kOrange);
        } else {
          ledSubsystem.setLED(index, Color.kBlue);
        }
      }
      blipIndex++;
      done = blipIndex - (BLIP_SIZE + 1) >= ledSubsystem.getStripLength();
    }
  }

  @Override
  public boolean isFinished() {
    return done;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    ledSubsystem.setColor(kBlack);
  }
}
