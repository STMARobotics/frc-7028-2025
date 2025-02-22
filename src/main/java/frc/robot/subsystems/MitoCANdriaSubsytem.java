package frc.robot.subsystems;

import static au.grapplerobotics.interfaces.MitoCANdriaInterface.MITOCANDRIA_CHANNEL_5VA;
import static au.grapplerobotics.interfaces.MitoCANdriaInterface.MITOCANDRIA_CHANNEL_5VB;
import static au.grapplerobotics.interfaces.MitoCANdriaInterface.MITOCANDRIA_CHANNEL_ADJ;
import static au.grapplerobotics.interfaces.MitoCANdriaInterface.MITOCANDRIA_CHANNEL_USB1;
import static au.grapplerobotics.interfaces.MitoCANdriaInterface.MITOCANDRIA_CHANNEL_USB2;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.MitoCANDriaConstants.DEVICE_ID_MITOCANDRIA;

import au.grapplerobotics.MitoCANdria;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * MitoCANdria subsystem. This subsystem configures the channels used by the robot, and provides methods to get the
 * status of the channels so they can be logged with Epilogue.
 */
@Logged(strategy = Strategy.OPT_IN)
public class MitoCANdriaSubsytem {

  private final MitoCANdria mitoCANdria = new MitoCANdria(DEVICE_ID_MITOCANDRIA);

  // Reusable measures to reduce memory pressure
  private final MutCurrent usb1Current = Amps.mutable(0);
  private final MutCurrent usb2Current = Amps.mutable(0);
  private final MutCurrent fiveVoltACurrent = Amps.mutable(0);
  private final MutVoltage usb1Voltage = Volts.mutable(0);
  private final MutVoltage usb2Voltage = Volts.mutable(0);
  private final MutVoltage fiveVoltAVoltage = Volts.mutable(0);

  /**
   * Constructs a new MitoCANdria subsystem, configuring the ports used by the robot.
   */
  public MitoCANdriaSubsytem() {
    try {
      // Enable 5VA channel for LEDs
      mitoCANdria.setChannelEnabled(MITOCANDRIA_CHANNEL_5VA, true);
      // Disable channels not used
      mitoCANdria.setChannelEnabled(MITOCANDRIA_CHANNEL_5VB, false);
      mitoCANdria.setChannelEnabled(MITOCANDRIA_CHANNEL_ADJ, false);
    } catch (Exception e) {
      DriverStation.reportError("MitoCANDria configuration failure: " + e.toString(), true);
    }
  }

  /**
   * Gets the current being drawn by Orange Pi 1
   * 
   * @return current draw, or zero if the value cannot be read
   */
  @Logged(name = "Pi One Current")
  public Current getPiOneCurrent() {
    getChannelCurrent(MITOCANDRIA_CHANNEL_USB1, usb1Current);
    return usb1Current;
  }

  /**
   * Gets the current being drawn by Orange Pi 2
   * 
   * @return current draw, or zero if the value cannot be read
   */
  @Logged(name = "Pi Two Current")
  public Current getPiTwoCurrent() {
    getChannelCurrent(MITOCANDRIA_CHANNEL_USB2, usb2Current);
    return usb2Current;
  }

  /**
   * Gets the current being drawn by the LEDs
   * 
   * @return current draw, or zero if the value cannot be read
   */
  @Logged(name = "LED Current")
  public Current getLedCurrent() {
    getChannelCurrent(MITOCANDRIA_CHANNEL_5VA, fiveVoltACurrent);
    return fiveVoltACurrent;
  }

  /**
   * Gets the current being drawn by Orange Pi 1
   * 
   * @return current draw, or zero if the value cannot be read
   */
  @Logged(name = "Pi One Volts")
  public Voltage getPiOneVolts() {
    getChannelVoltage(MITOCANDRIA_CHANNEL_USB1, usb1Voltage);
    return usb1Voltage;
  }

  /**
   * Gets the current being drawn by Orange Pi 2
   * 
   * @return current draw, or zero if the value cannot be read
   */
  @Logged(name = "Pi Two Volts")
  public Voltage getPiTwoVolts() {
    getChannelVoltage(MITOCANDRIA_CHANNEL_USB2, usb2Voltage);
    return usb2Voltage;
  }

  /**
   * Gets the current being drawn by the LEDs
   * 
   * @return current draw, or zero if the value cannot be read
   */
  @Logged(name = "LED Volts")
  public Voltage getLedVolts() {
    getChannelVoltage(MITOCANDRIA_CHANNEL_5VA, fiveVoltAVoltage);
    return fiveVoltAVoltage;
  }

  /**
   * Gets the current from a channel. If the value cannot be read, zero is returned.
   * 
   * @param channel channel to read
   * @param current mutable current measure whose value will be replaced with the current reading
   */
  private void getChannelCurrent(int channel, MutCurrent current) {
    try {
      mitoCANdria.getChannelCurrent(channel)
          .ifPresentOrElse(amps -> current.mut_replace(amps, Amps), () -> current.mut_replace(0, Amps));
    } catch (Exception e) {
      current.mut_replace(0, Amps);
    }
  }

  /**
   * Gets the voltage from a channel. If the value cannot be read, zero is returned.
   * 
   * @param channel channel to read
   * @param current mutable voltage measure whose value will be replaced with the voltage reading
   */
  private void getChannelVoltage(int channel, MutVoltage voltage) {
    try {
      mitoCANdria.getChannelVoltage(channel)
          .ifPresentOrElse(volts -> voltage.mut_replace(volts, Volts), () -> voltage.mut_replace(0, Volts));
    } catch (Exception e) {
      voltage.mut_replace(0, Volts);
    }
  }
}
