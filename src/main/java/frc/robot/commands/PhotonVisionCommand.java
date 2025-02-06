package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.CAMERA_NAMES;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionConsumer;
import java.util.Arrays;

/**
 * Command to use PhotonVision to process pose estimates and pass them to a pose estimator
 */
public class PhotonVisionCommand extends Command {
  private final Vision[] visions;
  private final VisionConsumer visionConsumer;

  /**
   * Constructs a PhotonVisionCommand
   * 
   * @param consumer consumer to receive vision estimates
   */
  public PhotonVisionCommand(VisionConsumer consumer) {
    visions = new Vision[CAMERA_NAMES.length];
    for (int i = 0; i < CAMERA_NAMES.length; i++) {
      visions[i] = new Vision(CAMERA_NAMES[i], ROBOT_TO_CAMERA_TRANSFORMS[i]);
    }
    this.visionConsumer = consumer;
  }

  @Override
  public void execute() {
    Arrays.stream(visions).forEach(vision -> {
      // Correct pose estimate with vision measurements
      var visionEst = vision.getEstimatedGlobalPose();
      visionEst.ifPresent(est -> {
        // Change our trust in the measurement based on the tags we can see
        var estStdDevs = vision.getEstimationStdDevs();

        visionConsumer.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      });
    });

  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}