package frc.robot.subsystems;

import static frc.robot.Constants.AlignmentConstants.DEVICE_ID_LEFT_CANRANGE;
import static frc.robot.Constants.AlignmentConstants.DEVICE_ID_RIGHT_CANRANGE;
import static frc.robot.Constants.CANIVORE_BUS_NAME;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem with sensors for aligning to a wall
 */
public class AlignmentSubsystem extends SubsystemBase {

  private final CANrange leftCanRange = new CANrange(DEVICE_ID_RIGHT_CANRANGE, CANIVORE_BUS_NAME);
  private final CANrange rightCanRange = new CANrange(DEVICE_ID_LEFT_CANRANGE, CANIVORE_BUS_NAME);

  private final StatusSignal<Distance> leftDistanceSignal = leftCanRange.getDistance();
  private final StatusSignal<Distance> rightDistanceSignal = rightCanRange.getDistance();

  /**
   * Constructs a new AlignmentSubsystem
   */
  public AlignmentSubsystem() {
    var canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ToFParams.withUpdateMode(UpdateModeValue.LongRangeUserFreq);

    leftCanRange.getConfigurator().apply(canRangeConfig);
    rightCanRange.getConfigurator().apply(canRangeConfig);
  }

  /**
   * Gets the distance detected by the left sensor
   * 
   * @return distance detected by the sensor
   */
  public Distance getLeftDistance() {
    return leftDistanceSignal.getValue();
  }

  /**
   * Gets the distance detected by the right sensor
   * 
   * @return distance detected by the sensor
   */
  public Distance getRightDistance() {
    return rightDistanceSignal.refresh().getValue();
  }

}
