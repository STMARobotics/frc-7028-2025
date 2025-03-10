package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.AlignmentConstants.BACK_CANRANGE_DISTANCE_FROM_CENTER;
import static frc.robot.Constants.AlignmentConstants.DEVICE_ID_BACK_CANRANGE;
import static frc.robot.Constants.AlignmentConstants.DEVICE_ID_FRONT_CANRANGE;
import static frc.robot.Constants.AlignmentConstants.FRONT_CANRANGE_DISTANCE_FROM_CENTER;
import static frc.robot.Constants.CANIVORE_BUS_NAME;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem with sensors for aligning to a wall
 */
@Logged
public class AlignmentSubsystem extends SubsystemBase {

  private final CANrange frontCanRange = new CANrange(DEVICE_ID_FRONT_CANRANGE, CANIVORE_BUS_NAME);
  private final CANrange backCanRange = new CANrange(DEVICE_ID_BACK_CANRANGE, CANIVORE_BUS_NAME);

  private final StatusSignal<Distance> frontDistanceSignal = frontCanRange.getDistance();
  private final StatusSignal<Distance> backDistanceSignal = backCanRange.getDistance();

  /**
   * Constructs a new AlignmentSubsystem
   */
  public AlignmentSubsystem() {
    var canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ToFParams.withUpdateMode(UpdateModeValue.LongRangeUserFreq);

    frontCanRange.getConfigurator().apply(canRangeConfig);
    backCanRange.getConfigurator().apply(canRangeConfig);
  }

  /**
   * Gets the distance detected by the front sensor
   * 
   * @return distance detected by the sensor
   */
  public Distance getFrontDistance() {
    return frontDistanceSignal.refresh().getValue();
  }

  /**
   * Gets the distance detected by the back sensor
   * 
   * @return distance detected by the sensor
   */
  public Distance getBackDistance() {
    return backDistanceSignal.refresh().getValue();
  }

  /**
   * Gets the relative angle of the drivetrain to the reef. Positive angle means the robot needs to turn clockwise.
   * 
   * @return relative angle of the drivetrain to the reef
   */
  public Angle getRelativeAngle() {
    Distance frontDistance = getFrontDistance();
    Distance backDistance = getBackDistance();

    return Radians.of(
        Math.atan2(
            backDistance.minus(frontDistance).in(Inches),
              Math.abs(FRONT_CANRANGE_DISTANCE_FROM_CENTER.minus(BACK_CANRANGE_DISTANCE_FROM_CENTER).in(Inches))));
  }

  /**
   * Gets the distance from the center of the front robot on the plane of the CANRanges to the reef perpendicular to the
   * reef.
   * 
   * @return Distance to the reef perpendicular to the reef
   */
  public Distance getDistance() {
    Distance backDistance = getBackDistance();
    Angle relativeAngle = getRelativeAngle();
    Distance normalizedBackDistance = backDistance.times(Math.cos(relativeAngle.in(Radians)));

    return normalizedBackDistance.plus(BACK_CANRANGE_DISTANCE_FROM_CENTER.times(Math.sin(relativeAngle.in(Radians))));
  }

}
