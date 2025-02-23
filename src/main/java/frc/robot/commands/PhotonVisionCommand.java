package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.CAMERA_NAMES;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionConsumer;

/**
 * Command to use PhotonVision to process pose estimates and pass them to a pose estimator
 */
public class PhotonVisionCommand extends Command {
  private final Vision[] visions;
  private final VisionConsumer visionConsumer;
  @SuppressWarnings("unchecked")
  private final StructPublisher<Pose2d>[] cameraPosePublishers = new StructPublisher[CAMERA_NAMES.length];

  /**
   * Constructs a PhotonVisionCommand
   * 
   * @param consumer consumer to receive vision estimates
   */
  public PhotonVisionCommand(VisionConsumer consumer) {
    visions = new Vision[CAMERA_NAMES.length];
    var photonPosesTable = NetworkTableInstance.getDefault().getTable("PhotonPoses");
    for (int i = 0; i < CAMERA_NAMES.length; i++) {
      visions[i] = new Vision(CAMERA_NAMES[i], ROBOT_TO_CAMERA_TRANSFORMS[i]);
      cameraPosePublishers[i] = photonPosesTable.getStructTopic(CAMERA_NAMES[i], Pose2d.struct).publish();
    }
    this.visionConsumer = consumer;
  }

  @Override
  public void execute() {
    for (int i = 0; i < visions.length; i++) {
      var vision = visions[i];
      // Correct pose estimate with vision measurements
      var visionEst = vision.getEstimatedGlobalPose();
      StructPublisher<Pose2d> cameraPoseTopic = cameraPosePublishers[i];
      visionEst.ifPresent(est -> {
        // Change our trust in the measurement based on the tags we can see
        var estStdDevs = vision.getEstimationStdDevs();
        // Send pose to pose estimator
        visionConsumer.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        // Send pose to network tables for easier debugging
        cameraPoseTopic.set(est.estimatedPose.toPose2d());
      });
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}