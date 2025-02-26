package frc.robot.commands.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
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
    TEST,
    DEFAULT
  }

  private final LEDSubsystem ledSubsystem;

  private final Timer timer = new Timer();
  private boolean candyCaneState = false;
  private static final double RED_AND_WHITE_CANDY_CANE_SPEED = 0.5;
  private static final double BLUE_AND_YELLOW_CANDY_CANE_SPEED = 1.0;
  private static final double ORANGE_AND_BLACK_CANDY_CANE_SPEED = 0.5;

  /**
   * Creates a new DefaultLEDCommand
   * 
   * @param ledSubsystem LED subsystem
   */
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
        if (timer.advanceIfElapsed(RED_AND_WHITE_CANDY_CANE_SPEED)) {
          candyCaneState = !candyCaneState;
        }
        ledSubsystem.setCandyCane(
            candyCaneState ? Color.kDarkRed : Color.kIndianRed,
              candyCaneState ? Color.kIndianRed : Color.kDarkRed);
        break;
      case DISABLED:
        if (timer.advanceIfElapsed(BLUE_AND_YELLOW_CANDY_CANE_SPEED)) {
          candyCaneState = !candyCaneState;
        }
        ledSubsystem
            .setCandyCane(candyCaneState ? Color.kBlue : Color.kOrange, candyCaneState ? Color.kOrange : Color.kBlue);
        break;
      case TEST:
        if (timer.advanceIfElapsed(ORANGE_AND_BLACK_CANDY_CANE_SPEED)) {
          candyCaneState = !candyCaneState;
        }
        ledSubsystem
            .setCandyCane(candyCaneState ? Color.kOrange : Color.kBlack, candyCaneState ? Color.kBlack : Color.kOrange);
        break;
      default:
        ledSubsystem.runPattern(
            LEDPattern.gradient(GradientType.kContinuous, Color.kBlue, Color.kOrange)
                .scrollAtRelativeSpeed(Percent.per(Second).of(25)));
        break;
    }
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
    if (RobotState.isTest()) {
      return LEDMode.TEST;
    }
    return LEDMode.DEFAULT;
  }

}
