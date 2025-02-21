package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;

/** Control bindings for an Xbox controller */
public class XBoxControlBindings extends ControlBindings {

  private final CommandXboxController driverController = new CommandXboxController(0);

  // Mutable measure objects to reduce memory pressure by not creating new instances on each iteration
  private final MutLinearVelocity translationX = MetersPerSecond.mutable(0);
  private final MutLinearVelocity translationY = MetersPerSecond.mutable(0);
  private final MutAngularVelocity omega = RadiansPerSecond.mutable(0);

  @Override
  public Supplier<LinearVelocity> translationX() {
    return () -> translationX.mut_replace(
        MAX_TELEOP_VELOCITY.in(MetersPerSecond) * -squareAxis(driverController.getLeftY()),
          MetersPerSecond);
  }

  @Override
  public Supplier<LinearVelocity> translationY() {
    return () -> translationY.mut_replace(
        MAX_TELEOP_VELOCITY.in(MetersPerSecond) * -squareAxis(driverController.getLeftX()),
          MetersPerSecond);
  }

  @Override
  public Supplier<AngularVelocity> omega() {
    return () -> omega.mut_replace(
        MAX_TELEOP_ANGULAR_VELOCITY.in(RadiansPerSecond) * -squareAxis(driverController.getRightX()),
          RadiansPerSecond);
  }

  private static double squareAxis(double value) {
    return Math.copySign(value * value, value);
  }

  @Override
  public Optional<Trigger> wheelsToX() {
    return Optional.of(driverController.back());
  }

  @Override
  public Optional<Trigger> intakeCoral() {
    return Optional.of(driverController.rightBumper());
  }

  @Override
  public Optional<Trigger> ejectCoral() {
    return Optional.of(driverController.leftBumper());
  }

  @Override
  public Optional<Trigger> tuneArm() {
    return Optional.of(driverController.start());
  }

  @Override
  public Optional<Trigger> scoreCoralLevel4() {
    return Optional.of(driverController.a());
  }
}