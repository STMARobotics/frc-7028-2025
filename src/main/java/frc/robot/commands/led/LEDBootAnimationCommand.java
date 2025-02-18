package frc.robot.commands.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.Map;

public class LEDBootAnimationCommand extends Command {

  private LEDSubsystem ledSubsystem;
  private Timer timer = new Timer();

  private static final double RUN_TIME = 2.0;

  private LEDPattern pattern;

  public LEDBootAnimationCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;

    // LEDPattern blueBase = LEDPattern.solid(Color.kBlue);
    // LEDPattern yellowBase = LEDPattern.solid(Color.kYellow);
    // Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
    // LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(1 / RUN_TIME));
    // LEDPattern yellowAnimation = yellowBase.mask(mask);

    // pattern = yellowAnimation.overlayOn(blueBase);

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    // Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.2, Color.kBlack);
    // pattern = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(80 / RUN_TIME));

    LEDPattern blueBase = LEDPattern.solid(Color.kBlue);
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kYellow, 0.2, Color.kBlack);
    pattern = LEDPattern.steps(maskSteps)
        .scrollAtRelativeSpeed(Percent.per(Second).of(100 / RUN_TIME))
        .overlayOn(blueBase);
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
  }

}
