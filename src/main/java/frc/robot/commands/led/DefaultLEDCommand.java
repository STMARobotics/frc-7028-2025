package frc.robot.commands.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.LEDPattern.GradientType.kContinuous;
import static edu.wpi.first.wpilibj.LEDPattern.gradient;
import static edu.wpi.first.wpilibj.LEDPattern.kOff;
import static edu.wpi.first.wpilibj.util.Color.kBlack;
import static edu.wpi.first.wpilibj.util.Color.kBlue;
import static edu.wpi.first.wpilibj.util.Color.kDarkRed;
import static edu.wpi.first.wpilibj.util.Color.kIndianRed;
import static edu.wpi.first.wpilibj.util.Color.kOrange;
import static frc.robot.subsystems.LEDSubsystem.candyCane;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/**
 * The default command for controlling the LEDs
 */
public class DefaultLEDCommand extends Command {

  private static final Time CANDY_CANE_SPEED = Seconds.of(0.5);

  private final LEDSubsystem ledSubsystem;

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
  public void execute() {
    LEDPattern pattern;
    if (!DriverStation.isDSAttached()) {
      pattern = candyCane(kDarkRed, kIndianRed, CANDY_CANE_SPEED);
    } else if (RobotState.isDisabled()) {
      pattern = gradient(kContinuous, kBlue, kOrange).scrollAtRelativeSpeed(Percent.per(Second).of(75));
    } else if (RobotState.isTest()) {
      pattern = candyCane(kOrange, kBlack, CANDY_CANE_SPEED);
    } else {
      pattern = kOff;
    }
    ledSubsystem.runPattern(pattern);
  }

  @Override
  public void end(boolean interrupted) {
    ledSubsystem.off();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
