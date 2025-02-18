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
import java.util.function.BooleanSupplier;

public class DefaultLEDCommand extends Command {

  private enum LEDMode {
    DISCONNECTED,
    DISABLED,
    CORAL_IN_END_EFFECTOR,
    ALGAE_IN_END_EFFECTOR,
    DEFAULT
  }

  private static final LEDPattern PATTERN_WHITE_SCROLL = LEDPattern
      .gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kWhite)
      .scrollAtRelativeSpeed(Percent.per(Second).of(50));
  private static final LEDPattern PATTERN_GREEN_SCROLL = LEDPattern
      .gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kLightSeaGreen)
      .scrollAtRelativeSpeed(Percent.per(Second).of(50));
  private static final LEDPattern PATTERN_OFF = LEDPattern.solid(Color.kBlack);

  private LEDSubsystem ledSubsystem;
  private BooleanSupplier coralInEndEffector;
  private BooleanSupplier algaeInEndEffector;

  private Timer timer = new Timer();
  private boolean candyCaneState = false;
  private static final double CANDY_CANE_SPEED = 1.0;

  public DefaultLEDCommand(
      LEDSubsystem ledSubsystem,
      BooleanSupplier coralInEndEffector,
      BooleanSupplier algaeInEndEffector) {
    this.ledSubsystem = ledSubsystem;
    this.coralInEndEffector = coralInEndEffector;
    this.algaeInEndEffector = algaeInEndEffector;

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
      case CORAL_IN_END_EFFECTOR:
        ledSubsystem.runPattern(PATTERN_WHITE_SCROLL);
        break;
      case ALGAE_IN_END_EFFECTOR:
        ledSubsystem.runPattern(PATTERN_GREEN_SCROLL);
        break;
      default:
        ledSubsystem.runPattern(PATTERN_OFF);
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  private LEDMode getMode() {
    if (!DriverStation.isDSAttached()) {
      return LEDMode.DISCONNECTED;
    }
    if (RobotState.isDisabled()) {
      return LEDMode.DISABLED;
    }
    if (coralInEndEffector.getAsBoolean()) {
      return LEDMode.CORAL_IN_END_EFFECTOR;
    }
    if (algaeInEndEffector.getAsBoolean()) {
      return LEDMode.ALGAE_IN_END_EFFECTOR;
    }
    return LEDMode.DEFAULT;
  }

}
