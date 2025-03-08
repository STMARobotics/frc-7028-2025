package frc.robot.subsystems;

import static frc.robot.Constants.AlignmentConstants.DEVICE_ID_BACK_CANRANGE;
import static frc.robot.Constants.AlignmentConstants.DEVICE_ID_FRONT_CANRANGE;
import static frc.robot.Constants.CANIVORE_BUS_NAME;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.epilogue.Logged;
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

}
