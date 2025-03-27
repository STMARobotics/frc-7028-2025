package frc.robot.commands.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.wpilibj.LEDPattern.GradientType.kContinuous;
import static edu.wpi.first.wpilibj.LEDPattern.gradient;
import static edu.wpi.first.wpilibj.LEDPattern.kOff;
import static edu.wpi.first.wpilibj.util.Color.kBlack;
import static edu.wpi.first.wpilibj.util.Color.kBlue;
import static edu.wpi.first.wpilibj.util.Color.kDarkRed;
import static edu.wpi.first.wpilibj.util.Color.kIndianRed;
import static edu.wpi.first.wpilibj.util.Color.kOrange;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
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
  private static final double CANDY_CANE_SPEED = 0.5;

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
    timer.restart();
  }

  @Override
  public void execute() {
    boolean advanceCandyCane = timer.advanceIfElapsed(CANDY_CANE_SPEED);
    switch (getMode()) {
      case DISCONNECTED:
        if (advanceCandyCane) {
          candyCaneState = !candyCaneState;
        }
        ledSubsystem.setCandyCane(candyCaneState ? kDarkRed : kIndianRed, candyCaneState ? kIndianRed : kDarkRed);
        break;
      case DISABLED:
        ledSubsystem
            .runPattern(gradient(kContinuous, kBlue, kOrange).scrollAtRelativeSpeed(Percent.per(Second).of(75)));
        break;
      case TEST:
        if (advanceCandyCane) {
          candyCaneState = !candyCaneState;
        }
        ledSubsystem.setCandyCane(candyCaneState ? kOrange : kBlack, candyCaneState ? kBlack : kOrange);
        break;
      default:
        ledSubsystem.runPattern(kOff);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    ledSubsystem.runPattern(kOff);
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
