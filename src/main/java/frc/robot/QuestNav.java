package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.FIELD_LENGTH;
import static frc.robot.Constants.FIELD_WIDTH;
import static frc.robot.Constants.QuestNavConstants.QUESTNAV_STD_DEVS;
import static frc.robot.Constants.QuestNavConstants.ROBOT_TO_QUEST;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

/**
 * The QuestNav class provides an interface to communicate with an Oculus/Meta Quest VR headset
 * for robot localization and tracking purposes. It uses NetworkTables to exchange data between
 * the robot and the Quest device.
 * <p>
 * This originated from https://github.com/QuestNav/QuestNav/blob/main/robot/QuestNav.java with slight modification to
 * remove AdvantageKit logger
 * </p>
 */
public class QuestNav implements Runnable {
  // Static inner classes for status and command codes
  public static class Status {
    /** Status indicating system is ready for commands */
    public static final int READY = 0;
    /** Status indicating heading reset completion */
    public static final int HEADING_RESET_COMPLETE = 99;
    /** Status indicating pose reset completion */
    public static final int POSE_RESET_COMPLETE = 98;
    /** Status indicating ping response receipt */
    public static final int PING_RESPONSE = 97;
  }

  public static class Command {
    /** Clear status */
    public static final int CLEAR = 0;
    /** Command to reset the heading */
    public static final int RESET_HEADING = 1;
    /** Command to reset the pose */
    public static final int RESET_POSE = 2;
    /** Command to ping the system */
    public static final int PING = 3;
  }

  @FunctionalInterface
  public interface AddVisionMeasurement {
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /** NetworkTable instance used for communication */
  private final NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();

  /** NetworkTable for Quest navigation data */
  private final NetworkTable nt4Table = nt4Instance.getTable("questnav");

  /** Subscriber for message input from Quest (MISO - Master In Slave Out) */
  private final IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(-1);

  /** Publisher for message output to Quest (MOSI - Master Out Slave In) */
  private final IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  /** Subscriber for Quest timestamp data */
  private final DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(-1.0f);

  /**
   * Subscriber for raw position data from Quest in Unity coordinate system (translation, Unity's x becomes FRC's
   * -y, and Unity's z becomes FRC's x)
   */
  private final FloatArraySubscriber questPoseArray = nt4Table.getFloatArrayTopic("poseArray")
      .subscribe(
          new float[] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
            PubSubOption.sendAll(true),
            PubSubOption.pollStorage(20));

  /** Subscriber for Quest frame counter */
  private final IntegerSubscriber questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(-1);

  /** Subscriber for Quest battery percentage */
  private final DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("device/batteryPercent")
      .subscribe(-1.0f);

  /** Subscriber for Quest tracking status */
  private final BooleanSubscriber questIsTracking = nt4Table.getBooleanTopic("device/isTracking").subscribe(false);

  /** Subscriber for Quest tracking loss counter */
  private final IntegerSubscriber questTrackingLostCount = nt4Table.getIntegerTopic("device/trackingLostCounter")
      .subscribe(-1);

  /** Subscriber for heartbeat requests from Quest */
  private final DoubleSubscriber heartbeatRequestSub = nt4Table.getDoubleTopic("heartbeat/quest_to_robot")
      .subscribe(-1.0);

  /** Publisher for heartbeat responses to Quest */
  private final DoublePublisher heartbeatResponsePub = nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish();

  /** Custom log entry */
  private final StringLogEntry logEntry = new StringLogEntry(DataLogManager.getLog(), "QuestNav/Log");

  /** Lock so resetting pose doesn't conflict with getting pose on a separate thread */
  private final ReentrantLock lock = new ReentrantLock();

  /** Last processed heartbeat request ID */
  private double lastProcessedHeartbeatId = 0;

  /** Pose of the Quest when the pose was reset */
  private Pose2d resetPoseOculus = new Pose2d();

  /** Pose of the robot when the pose was reset */
  private Pose2d resetPoseRobot = new Pose2d();

  /** Server time for the last pose update, used to calculate frequency metric */
  private double lastPoseServerTime = 0.0;

  /* Consumer of QuestNav pose estimates */
  private final AddVisionMeasurement poseConsumer;

  /** Supplier of the robot's current pose estimate */
  private final Supplier<Pose2d> poseSupplier;

  // Entries for metrics/logging
  private final NetworkTable questDataTable = NetworkTableInstance.getDefault().getTable("QuestData");
  private final StructPublisher<Pose2d> questPosePublisher = questDataTable.getStructTopic("Robot Pose", Pose2d.struct)
      .publish();
  private final DoublePublisher questPeriodPublisher = questDataTable.getDoubleTopic("Period").publish();

  private boolean hasQuestConnected = false;

  /**
   * Constructs a new QuestNav object
   * 
   * @param poseConsumer consumer for pose estimates from QuestNav
   * @param poseSupplier supplier for the robot's current pose estimate
   */
  public QuestNav(AddVisionMeasurement poseConsumer, Supplier<Pose2d> poseSupplier) {
    this.poseConsumer = poseConsumer;
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void run() {
    var waitHandle = questPoseArray.getHandle();
    while (!Thread.interrupted()) {
      // Block the thread until new data comes in from QuestNav
      try {
        WPIUtilJNI.waitForObject(waitHandle);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }

      if (isTracking()) {
        if (!hasQuestConnected) {
          // When QuestNav connects the first time and starts tracking, set the pose to the current pose of the robot
          resetRobotPose(poseSupplier.get());
          hasQuestConnected = true;
        }
        var timestampedPoseArrays = questPoseArray.readQueue();
        Arrays.stream(timestampedPoseArrays).forEach(timestampedPoseArray -> {
          // Process all of the pose estimates since the last notification
          var robotPoseEstimate = calculateRobotPose(timestampedPoseArray);

          // Make sure we are inside the field
          if (robotPoseEstimate.getX() >= 0.0 && robotPoseEstimate.getX() <= FIELD_LENGTH.in(Meters)
              && robotPoseEstimate.getY() >= 0.0 && robotPoseEstimate.getY() <= FIELD_WIDTH.in(Meters)) {
            // Add the measurement
            poseConsumer.addVisionMeasurement(robotPoseEstimate, timestampedPoseArray.serverTime, QUESTNAV_STD_DEVS);
          }

          // Publish data for debugging/logging
          questPosePublisher.accept(robotPoseEstimate);
          questPeriodPublisher.accept((timestampedPoseArray.serverTime - lastPoseServerTime) / 1000);
          lastPoseServerTime = timestampedPoseArray.serverTime;
        });
      }
      // Required cleanup
      processHeartbeat();
      cleanupResponses();
    }
  }

  private Pose2d calculateRobotPose(TimestampedFloatArray timestampedPoseArray) {
    lock.lock();
    try {
      // Calculate the Quest pose by applying the reset offset
      var rawQuestPose = unpackPose2d(timestampedPoseArray.value);
      var questOffsetRelativeToReset = rawQuestPose.minus(resetPoseOculus);
      var questPose = resetPoseRobot.transformBy(questOffsetRelativeToReset);

      // Calculate the robot pose relative to the Quest
      var robotPoseRelativeToReset = questPose.transformBy(ROBOT_TO_QUEST.inverse());
      return robotPoseRelativeToReset;
    } finally {
      lock.unlock();
    }
  }

  /**
   * Set the robot's pose on the field. This is useful to seed the robot to a known position. This is usually called at
   * the start of the autonomous period.
   * 
   * @param newPose new robot pose
   */
  public void resetRobotPose(Pose2d newPose) {
    lock.lock();
    try {
      var questPose = unpackPose2d(questPoseArray.get());
      resetPoseOculus = questPose.transformBy(ROBOT_TO_QUEST.inverse());
      resetPoseRobot = newPose;
    } finally {
      lock.unlock();
    }
  }

  /**
   * Unpacks the pose data from the Quest headset into a Pose2d object.
   *
   * @param poseArray The array containing pose data from the Quest headset
   * @return A Pose2d object representing the unpacked pose
   */
  private Pose2d unpackPose2d(float[] poseArray) {
    return new Pose2d(new Translation2d(poseArray[2], -poseArray[0]), Rotation2d.fromDegrees(-poseArray[4]));
  }

  /**
   * Processes heartbeat requests from the Quest headset and responds with the same ID.
   * This helps maintain connection status between the robot and the Quest.
   * <br/>
   * <b>MUST BE RUN IN PERIODIC METHOD</b>
   */
  private void processHeartbeat() {
    double requestId = heartbeatRequestSub.get();
    // Only respond to new requests to avoid flooding
    if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
      heartbeatResponsePub.set(requestId);
      lastProcessedHeartbeatId = requestId;
    }
  }

  /**
   * Gets the battery percentage of the Quest headset.
   *
   * @return The battery percentage as a Double value
   */
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  /**
   * Gets the current tracking state of the Quest headset.
   *
   * @return Boolean indicating if the Quest is currently tracking (true) or not (false)
   */
  public boolean isTracking() {
    return questIsTracking.get();
  }

  /**
   * Gets the current frame count from the Quest headset.
   *
   * @return The frame count as a Long value
   */
  public long getFrameCount() {
    return questFrameCount.get();
  }

  /**
   * Gets the number of tracking lost events since the Quest connected to the robot.
   *
   * @return The tracking lost counter as a Long value
   */
  public long getTrackingLostCounter() {
    return questTrackingLostCount.get();
  }

  /**
   * Determines if the Quest headset is currently connected to the robot.
   * Connection is determined by checking when the last battery update was received.
   *
   * @return Boolean indicating if the Quest is connected (true) or not (false)
   */
  public boolean isConnected() {
    return Seconds.of(Timer.getTimestamp()).minus(Microseconds.of(questTimestamp.getLastChange())).lt(Seconds.of(0.25));
  }

  /**
   * Cleans up Quest navigation subroutine messages after processing on the headset.
   * Resets the MOSI value to zero if MISO is non-zero.
   * <br/>
   * <b>MUST BE RUN IN PERIODIC METHOD</b>
   */
  private void cleanupResponses() {
    if (questMiso.get() != Status.READY) {
      switch ((int) questMiso.get()) {
        case Status.POSE_RESET_COMPLETE -> {
          logEntry.append("Pose reset complete");
          questMosi.set(Command.CLEAR);
        }
        case Status.HEADING_RESET_COMPLETE -> {
          logEntry.append("Heading reset complete");
          questMosi.set(Command.CLEAR);
        }
        case Status.PING_RESPONSE -> {
          logEntry.append("Ping response received");
          questMosi.set(Command.CLEAR);
        }
        default -> {
          // NOOP
        }
      }
    }
  }

}