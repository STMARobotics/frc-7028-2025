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

  /**
   * Trigger to move the climb in the direction that makes the robot go up
   *
   * @return duty cycle supplier
   */
  public Optional<Trigger> climb() {
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
   * @return optional Trigger
   */
  public Optional<Trigger> ejectCoral() {
    return Optional.empty();
  }

  /**
   * Moves the arm to reef level 2
   * 
   * @return optional trigger
   */
  public Optional<Trigger> moveArmToReefLevel2() {
    return Optional.empty();
  }

  /**
   * Moves the arm to reef level 3
   * 
   * @return optional trigger
   */
  public Optional<Trigger> moveArmToReefLevel3() {
    return Optional.empty();
  }

  /**
   * Moves the arm to reef level 4
   * 
   * @return optional trigger
   */
  public Optional<Trigger> moveArmToReefLevel4() {
    return Optional.empty();
  }

  /**
   * Runs the game piece manipulator to release the coral
   * 
   * @return optional trigger
   */
  public Optional<Trigger> releaseCoral() {
    return Optional.empty();
  }

  /**
   * Moves the arm to the park position
   * 
   * @return optional trigger
   */
  public Optional<Trigger> parkArm() {
    return Optional.empty();
  }

  /**
   * Runs the command to move the arm to positions from NT. Should be used in competition.
   * 
   * @return optional trigger
   */
  public Optional<Trigger> tuneArm() {
    return Optional.empty();
  }

  /**
   * Moves the arm to the position to extract algae from the reef at the lower level
   * 
   * @return optional trigger
   */
  public Optional<Trigger> moveArmToReefAlgaeLevel1() {
    return Optional.empty();
  }

  /**
   * Moves the arm to the position to extract algae from the reef at the higher level
   * 
   * @return optional trigger
   */
  public Optional<Trigger> moveArmToReefAlgaeLevel2() {
    return Optional.empty();
  }

  /**
   * Runs the manipulator wheels to eject algae
   * 
   * @return optional trigger
   */
  public Optional<Trigger> ejectAlgae() {
    return Optional.empty();
  }

  /**
   * Drives to the nearest reef branch and scores on level 4
   * 
   * @return optional trigger
   */
  public Optional<Trigger> scoreCoralLevel4() {
    return Optional.empty();
  }

  /**
   * Drives to the nearest reef branch and scores on level 3
   * 
   * @return optional trigger
   */
  public Optional<Trigger> scoreCoralLevel3() {
    return Optional.empty();
  }

  /**
   * Changes the drivetrain into slow mode
   * 
   * @return optional trigger
   */
  public Optional<Trigger> slowMode() {
    return Optional.empty();
  }
}