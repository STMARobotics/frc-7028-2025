package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;

/** Control bindings for driving with joysticks */
public class JoystickControlBindings extends ControlBindings {

  private final CommandJoystick leftJoystick = new CommandJoystick(0);
  private final CommandJoystick rightJoystick = new CommandJoystick(1);

  private final MutLinearVelocity translationX = MetersPerSecond.mutable(0);
  private final MutLinearVelocity translationY = MetersPerSecond.mutable(0);
  private final MutAngularVelocity omega = RadiansPerSecond.mutable(0);

  @Override
  public Supplier<LinearVelocity> translationX() {
    return () -> translationX
        .mut_replace(MAX_TELEOP_VELOCITY.in(MetersPerSecond) * -squareAxis(leftJoystick.getY()), MetersPerSecond);
  }

  @Override
  public Supplier<LinearVelocity> translationY() {
    return () -> translationY
        .mut_replace(MAX_TELEOP_VELOCITY.in(MetersPerSecond) * -squareAxis(leftJoystick.getX()), MetersPerSecond);
  }

  @Override
  public Supplier<AngularVelocity> omega() {
    return () -> omega.mut_replace(
        MAX_TELEOP_ANGULAR_VELOCITY.in(RadiansPerSecond) * -squareAxis(rightJoystick.getX()),
          RadiansPerSecond);
  }

  private static double squareAxis(double value) {
    return Math.copySign(value * value, value);
  }

  @Override
  public Optional<Trigger> moveArmToReefLevel2() {
    return Optional.of(rightJoystick.povRight());
  }

  @Override
  public Optional<Trigger> moveArmToReefLevel3() {
    return Optional.of(rightJoystick.povLeft());
  }

  @Override
  public Optional<Trigger> moveArmToReefLevel4() {
    return Optional.of(rightJoystick.povUp());
  }

  @Override
  public Optional<Trigger> climb() {
    return Optional.of(rightJoystick.button(9));
  }

  @Override
  public Optional<Trigger> ejectCoral() {
    return Optional.of(leftJoystick.povRight());
  }

  @Override
  public Optional<Trigger> releaseCoral() {
    return Optional.of(rightJoystick.trigger());
  }

  @Override
  public Optional<Trigger> tuneArm() {
    return Optional.of(leftJoystick.button(8));
  }

  @Override
  public Optional<Trigger> moveArmToReefLowerAlgae() {
    return Optional.of(leftJoystick.povDown());
  }

  @Override
  public Optional<Trigger> moveArmToReefUpperAlgae() {
    return Optional.of(leftJoystick.povUp());
  }

  @Override
  public Optional<Trigger> selectCoralLevel1() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> selectCoralLevel2() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> selectCoralLevel3() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> selectCoralLevel4() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> scoreCoralLeft() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> scoreCoralRight() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> slowMode() {
    return Optional.of(leftJoystick.button(3));
  }

  @Override
  public Optional<Trigger> holdAlgae() {
    return Optional.of(leftJoystick.button(10));
  }

  @Override
  public Optional<Trigger> shootAlgae() {
    return Optional.of(leftJoystick.button(9));
  }
}