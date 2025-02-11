package frc.robot.controls;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Abstract class that defines the available bindings for driver controls. It can be extended to provide a "control
 * scheme"
 */
public abstract class ControlBindings {

  /**
   * Supplier for the driver desired X speed
   *
   * @return velocity supplier
   */
  public abstract Supplier<LinearVelocity> translationX();

  /**
   * Supplier for the driver desired Y speed
   *
   * @return velocity supplier
   */
  public abstract Supplier<LinearVelocity> translationY();

  /**
   * Supplier for the drive desired omega rotation
   *
   * @return velocity supplier
   */
  public abstract Supplier<AngularVelocity> omega();

  /**
   * Triggers putting the wheels into an X configuration
   *
   * @return optional trigger
   */
  public Optional<Trigger> wheelsToX() {
    return Optional.empty();
  }

  /**
   * Trigger to reset reset the heading to zero.
   * <p>
   * Because of global pose estimation, <b>there is no reason this should be used in competition bindings.<b>
   * </p>
   *
   * @return optional trigger
   */
  public Optional<Trigger> seedFieldCentric() {
    return Optional.empty();
  }

  /**
   * Supplier for the climb duty cycle
   *
   * @return duty cycle supplier
   */
  public Optional<DoubleSupplier> climb() {
    return Optional.empty();
  }

  /**
   * Intakes coral from the Coral Station.
   * 
   * @return optional trigger
   */
  public Optional<Trigger> intakeCoral() {
    return Optional.empty();
  }

  /**
   * Eject the Coral and put it on the
   * 
   * @return Optional Trigger
   */
  public Optional<Trigger> ejectCoral() {
    return Optional.empty();
  }

  /**
   * Make sure the the Coral is in the right postion and allowes the arm to hold it.
   * 
   * @return Optional Trigger
   */
  public Optional<Trigger> holdCoral() {
    return Optional.empty();
  }

  /**
   * Allows the Game Piece Manipultor to score accurately.
   * 
   * @return Optional Trigger
   */
  public Optional<Trigger> scoreCoral() {
    return Optional.empty();
  }
}
