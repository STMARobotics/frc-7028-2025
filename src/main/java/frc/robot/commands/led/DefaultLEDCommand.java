package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/**
 * The default command for controlling the LEDs
 */
public class DefaultLEDCommand extends Command {

  private enum LEDMode {
    DISCONNECTED,
    DISABLED,
    DEFAULT
  }

  private static final LEDPattern PATTERN_OFF = LEDPattern.solid(Color.kBlack);

  private LEDSubsystem ledSubsystem;

  private Timer timer = new Timer();
  private boolean candyCaneState = false;
  private static final double CANDY_CANE_SPEED = 1.0;

  public DefaultLEDCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    switch (getMode()) {
      case DISCONNECTED:
        if (timer.advanceIfElapsed(CANDY_CANE_SPEED)) {
          candyCaneState = !candyCaneState;
        }
        ledSubsystem
            .setCandyCane(candyCaneState ? Color.kWhite : Color.kRed, candyCaneState ? Color.kRed : Color.kWhite);
        break;
      case DISABLED:
        if (timer.advanceIfElapsed(CANDY_CANE_SPEED)) {
          candyCaneState = !candyCaneState;
        }
        ledSubsystem
            .setCandyCane(candyCaneState ? Color.kBlue : Color.kYellow, candyCaneState ? Color.kYellow : Color.kBlue);
        break;
      default:
        ledSubsystem.runPattern(PATTERN_OFF);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  /**
   * Gets the mode that the LEDs should be running in.
   * 
   * @return Current mode that the LEDs should be running in
   */
  private LEDMode getMode() {
    if (!DriverStation.isDSAttached()) {
      return LEDMode.DISCONNECTED;
    }
    if (RobotState.isDisabled()) {
      return LEDMode.DISABLED;
    }
    return LEDMode.DEFAULT;
  }

}
