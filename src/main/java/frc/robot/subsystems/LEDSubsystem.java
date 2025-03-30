package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microsecond;
import static frc.robot.Constants.LEDConstants.DEVICE_ID_LEDS;
import static frc.robot.Constants.LEDConstants.LED_STRIP_LENGTH;
import static frc.robot.Constants.LEDConstants.TOTAL_LEDS;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

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
    }).finallyDo(() -> LEDPattern.kOff.applyTo(ledBuffer)).ignoringDisable(true);
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
   * Creates an LEDPattern that makes an animated candy cane effect (alternating colors)
   * 
   * @param color1 The first color
   * @param color2 The second color
   * @param period The length of time before swapping the colors
   */
  public static LEDPattern candyCane(Color color1, Color color2, Time period) {
    var periodMicros = (long) period.in(Microsecond);
    return (reader, writer) -> {
      var isOdd = (RobotController.getTime() / periodMicros) % 2 == 1;
      for (int led = 0; led < reader.getLength(); led++) {
        if (isOdd) {
          writer.setLED(led, led % 2 == 0 ? color1 : color2);
        } else {
          writer.setLED(led, led % 2 == 0 ? color2 : color1);
        }
      }
    };
  }

  /**
   * Lights up the LEDs in segments. Useful for indicating ready state, for example.
   *
   * @param color the color of the segments when lit
   * @param segmentValues array of boolean suppliers. The strip will be split into segments one segment for each element
   *          of the array.
   */
  public static LEDPattern ledSegments(Color color, BooleanSupplier... segmentValues) {
    return (reader, writer) -> {
      final int ledsPerStatus = reader.getLength() / segmentValues.length;
      int ledIndex = 0;
      for (int segmentId = 0; segmentId < segmentValues.length; segmentId++) {
        for (; ledIndex < (ledsPerStatus * (segmentId + 1)); ledIndex++) {
          writer.setLED(ledIndex, segmentValues[segmentId].getAsBoolean() ? color : Color.kBlack);
        }
      }
    };
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
