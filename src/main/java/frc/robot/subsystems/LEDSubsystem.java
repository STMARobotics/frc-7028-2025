package frc.robot.subsystems;

import static frc.robot.Constants.LEDConstants.DEVICE_ID_LEDS;
import static frc.robot.Constants.LEDConstants.LED_STRIP_LENGTH;
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
  private final AddressableLEDBufferView frontStripBuffer;
  private final AddressableLEDBufferView backStripBuffer;

  /**
   * Creates a new LEDSubsystem
   */
  public LEDSubsystem() {
    leds.setLength(TOTAL_LEDS);
    leds.setData(ledBuffer);

    frontStripBuffer = new AddressableLEDBufferView(ledBuffer, 0, LED_STRIP_LENGTH - 1);
    backStripBuffer = new AddressableLEDBufferView(ledBuffer, LED_STRIP_LENGTH, 2 * LED_STRIP_LENGTH - 1).reversed();

    leds.start();
  }

  @Override
  public void periodic() {
    leds.setData(ledBuffer);
  }

  /**
   * This will create a command that runs a pattern on each LED strip individually. This pattern will automatically
   * update as long as the the command is running, so this method only needs to be called one time for an animation. The
   * command will turn the LEDs off when it is completed.
   * 
   * @param pattern Pattern to set on each LED strip from bottom to top
   * @return A command that will run the pattern on each LED strip continuously
   */
  public Command runPatternAsCommand(LEDPattern pattern) {
    return run(() -> {
      pattern.applyTo(frontStripBuffer);
      pattern.applyTo(backStripBuffer);
    }).finallyDo(() -> LEDPattern.kOff.applyTo(ledBuffer));
  }

  /**
   * Applies the pattern to each LED strip individually. This will only happen once. If running an animation, this
   * method must be called continuously to update the led states.
   * 
   * @param pattern Pattern to set on each LED strip from bottom to top
   */
  public void runPattern(LEDPattern pattern) {
    pattern.applyTo(frontStripBuffer);
    pattern.applyTo(backStripBuffer);
  }

  /**
   * Applies a candy cane pattern (alternating colors) to each LED strip.
   * 
   * @param color1 The first color
   * @param color2 The second color
   */
  public void setCandyCane(Color color1, Color color2) {
    for (int i = 0; i < LED_STRIP_LENGTH; i++) {
      frontStripBuffer.setLED(i, i % 2 == 0 ? color1 : color2);
      backStripBuffer.setLED(i, i % 2 == 0 ? color1 : color2);
    }
  }

  /**
   * Sets the RGB value for an LED at a specific index.
   *
   * @param index the index of the LED to write to
   * @param color the color to set
   */
  public void setLED(int index, Color color) {
    frontStripBuffer.setLED(index, color);
    backStripBuffer.setLED(index, color);
  }

  /**
   * Sets the RGB value for all LEDs.
   * 
   * @param color the color to set
   */
  public void setColor(Color color) {
    LEDPattern solidColor = LEDPattern.solid(color);
    solidColor.applyTo(frontStripBuffer);
    solidColor.applyTo(backStripBuffer);
  }

  /**
   * Gets the length of the LED strips.
   * 
   * @return length of the strips
   */
  public int getStripLength() {
    return LED_STRIP_LENGTH;
  }

}
