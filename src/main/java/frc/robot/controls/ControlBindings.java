package frc.robot.controls;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
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

  public interface InnerControlBindings {

    /**
     * Intakes the Coral from the reef.
     * 
     * @return
     */
    public static Optional<Trigger> intakeCoralCommand() {
      return Optional.empty();
    }

    /**
     * Eject the Coral from the arm and Game Pieace Manipulator.
     * 
     * @return
     */
    public static Optional<Trigger> ejectCoralCommand() {
      return Optional.empty();
    }

    /**
     * Make sure the the Coral is in the right postion and allowes the arm to hold it.
     * 
     * @return
     */
    public static Optional<Trigger> activeHoldCoralComammand() {
      return Optional.empty();
    }

    /**
     * Allows the Game Piece Manipultor to score accurately.
     * 
     * @return
     */
    public static Optional<Trigger> scoreCoralCommand() {
      return Optional.empty();
    }
  }
}