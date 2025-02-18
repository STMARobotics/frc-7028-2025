package frc.robot.subsystems;

import static frc.robot.Constants.LEDConstants.BACK_LEFT_STRIP_LENGTH;
import static frc.robot.Constants.LEDConstants.DEVICE_ID_LEDS;
import static frc.robot.Constants.LEDConstants.FRONT_LEFT_STRIP_LENGTH;
import static frc.robot.Constants.LEDConstants.TOTAL_LEDS;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the LEDs
 */
public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED leds = new AddressableLED(DEVICE_ID_LEDS);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(TOTAL_LEDS);
  private final AddressableLEDBufferView frontLeftStripBuffer;
  private final AddressableLEDBufferView backLeftStripBuffer;
  private final AddressableLEDBufferView backRightStripBuffer;

  public LEDSubsystem() {
    leds.setLength(TOTAL_LEDS);
    leds.setData(ledBuffer);

    frontLeftStripBuffer = new AddressableLEDBufferView(ledBuffer, 0, FRONT_LEFT_STRIP_LENGTH - 1);
    backLeftStripBuffer = new AddressableLEDBufferView(
        ledBuffer,
        FRONT_LEFT_STRIP_LENGTH,
        FRONT_LEFT_STRIP_LENGTH + BACK_LEFT_STRIP_LENGTH - 1).reversed();
    backRightStripBuffer = new AddressableLEDBufferView(
        ledBuffer,
        FRONT_LEFT_STRIP_LENGTH + BACK_LEFT_STRIP_LENGTH,
        TOTAL_LEDS - 1);

    leds.start();
  }

  @Override
  public void periodic() {
    leds.setData(ledBuffer);
  }

  /**
   * This will create a command that runs a pattern on each LED strip individually. This pattern will automatically
   * update as long as the the command is running, so this method only needs to be called one time for an animation.
   * 
   * @param pattern Pattern to set on each LED strip from bottom to top
   * @return A command that will run the pattern on each LED strip continuously
   */
  public Command runPatternAsCommand(LEDPattern pattern) {
    return run(() -> {
      pattern.applyTo(frontLeftStripBuffer);
      pattern.applyTo(backLeftStripBuffer);
      pattern.applyTo(backRightStripBuffer);
    });
  }

  /**
   * Applies the pattern to each LED strip individually. This will only happen once. If running an animation, this
   * method must be called continuously to update the led states.
   * 
   * @param pattern Pattern to set on each LED strip from bottom to top
   */
  public void runPattern(LEDPattern pattern) {
    pattern.applyTo(frontLeftStripBuffer);
    pattern.applyTo(backLeftStripBuffer);
    pattern.applyTo(backRightStripBuffer);
  }

  /**
   * Applies a candy cane pattern (alternating colors) to each LED strip.
   * 
   * @param color1 The first color
   * @param color2 The second color
   */
  public void setCandyCane(Color color1, Color color2) {
    for (int i = 0; i < TOTAL_LEDS; i++) {
      ledBuffer.setLED(i, i % 2 == 0 ? color1 : color2);
    }
  }

}
