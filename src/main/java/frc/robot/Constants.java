package frc.robot;

import static com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;
import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;
import static java.util.stream.Collectors.toUnmodifiableList;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.generated.TunerConstants;
import java.util.List;
import java.util.stream.Stream;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

  public static final String CANIVORE_BUS_NAME = "canivore";

  // Dimensions for the WELDED field
  public static final Distance FIELD_LENGTH = Meters.of(17.548);
  public static final Distance FIELD_WIDTH = Meters.of(8.052);

  /**
   * Constants for teleoperated driver control
   */
  public static class TeleopDriveConstants {
    /** Max velocity the driver can request */
    public static final LinearVelocity MAX_TELEOP_VELOCITY = TunerConstants.kSpeedAt12Volts;
    /** Max angular velicity the driver can request */
    public static final AngularVelocity MAX_TELEOP_ANGULAR_VELOCITY = RotationsPerSecond.of(1.25);
  }

  /**
   * Constants for automatically driving to a pose
   * 
   * @see DriveToPoseCommand
   */
  public static class DriveToPoseConstants {
    public static final LinearVelocity MAX_DRIVE_TO_POSE_TRANSLATION_VELOCITY = MAX_TELEOP_VELOCITY.div(2.0);
    public static final LinearAcceleration MAX_DRIVE_TO_POSE_TRANSLATION_ACCELERATION = MetersPerSecondPerSecond
        .of(2.0);
    public static final AngularVelocity MAX_DRIVE_TO_POSE_ANGULAR_VELOCITY = MAX_TELEOP_ANGULAR_VELOCITY.times(0.75);
    public static final AngularAcceleration MAX_DRIVE_TO_POSE_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond
        .of(6.0 * Math.PI);

    public static final double THETA_kP = 3.0;
    public static final double THETA_kI = 0.0;
    public static final double THETA_kD = 0.0;

    public static final double X_kP = 5.0;
    public static final double X_kI = 0.0;
    public static final double X_kD = 0.0;

    public static final double Y_kP = 5.0;
    public static final double Y_kI = 0.0;
    public static final double Y_kD = 0.0;
  }

  /**
   * Constants for vision processing
   */
  public static class VisionConstants {
    public static final String[] CAMERA_NAMES = new String[] { "Left" };
    public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = new Transform3d[] {
        new Transform3d(
            new Translation3d(inchesToMeters(10.846), inchesToMeters(13.55), inchesToMeters(10.775)),
            new Rotation3d(0, 0, degreesToRadians(91))),
        new Transform3d(
            new Translation3d(inchesToMeters(-13.472), inchesToMeters(-8.088), inchesToMeters(8.541)),
            new Rotation3d(0, 0, degreesToRadians(180))),
        new Transform3d(
            new Translation3d(inchesToMeters(8.5), inchesToMeters(13.5), inchesToMeters(37.065)),
            new Rotation3d(0, degreesToRadians(70), degreesToRadians(90))) };

    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(2, 2, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

    /**
     * Minimum target ambiguity. Targets with higher ambiguity will be discarded. Not appliable when
     * multiple tags are in view in a single camera.
     */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final Distance SINGLE_TAG_DISTANCE_THRESHOLD = Meters.of(4.5);

  }

  /**
   * Constants for the indexer subsystem
   */
  public static class IndexerConstants {
    public static final int DEVICE_ID_BELT = 35;
    public static final int DEVICE_ID_GAME_PIECE_CANRANGE = 36;

    public static final Distance CORAL_DETECTION_THRESHOLD = Meters.of(0.06);

    public static final Current INDEXER_STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Current INDEXER_TORQUE_CURRENT_LIMIT = Amps.of(40);
    public static final Current INDEXER_SUPPLY_CURRENT_LIMIT = Amps.of(20);

    public static final SlotConfigs SLOT_CONFIGS = new SlotConfigs().withKP(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0)
        .withKA(0.0);
  }

  /**
   * Constants for the climb subsystem
   */
  public static class ClimbConstants {
    public static final int DEVICE_ID_CLIMB_MOTOR = 31;
    public static final int DEVICE_ID_CLIMB_CANDI = 32;

    public static final Angle CLIMB_MAGNETIC_OFFSET_FRONT = Rotations.of(0.0);

    public static final Current CLIMB_STATOR_CURRENT_LIMIT = Amps.of(150);
    public static final Current CLIMB_SUPPLY_CURRENT_LIMIT = Amps.of(40);
    public static final double CLIMB_ROTOR_TO_SENSOR_RATIO = 144;

    public static final Angle CLIMB_FORWARD_SOFT_LIMIT = Rotations.of(0.826);
    public static final Angle CLIMB_START_POSITION = Rotations.of(0.23);

    public static final Voltage CLIMB_VOLTAGE = Volts.of(12);
  }

  /**
   * Constants for the arm subsystem
   */
  public static class ArmConstants {
    public static final int DEVICE_ID_ELEVATOR_MOTOR_LEADER = 40;
    public static final int DEVICE_ID_ELEVATOR_MOTOR_FOLLOWER = 41;
    public static final int DEVICE_ID_ELEVATOR_CANDI = 42;

    public static final int DEVICE_ID_ARM_MOTOR = 45;
    public static final int DEVICE_ID_ARM_CANDI = 46;

    public static final SlotConfigs ELEVATOR_SLOT_CONFIGS = new SlotConfigs().withKP(0.33)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.135)
        .withKA(0.002) // V*s^2/m
        .withKG(0.39); // Volts

    public static final MotionMagicConfigs ELEVATOR_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(40)
        .withMotionMagicAcceleration(120);

    public static final Current ELEVATOR_SUPPLY_CURRENT_LIMIT = Amps.of(80);
    public static final Current ELEVATOR_STATOR_CURRENT_LIMIT = Amps.of(100);

    public static final Measure<PerUnit<DistanceUnit, AngleUnit>> ELEVATOR_DISTANCE_PER_ROTATION = Meters.per(Rotation)
        .ofNative(0.05489);

    public static final Distance ELEVATOR_TOP_LIMIT = Meters.of(0.727075);
    public static final Distance ELEVATOR_BOTTOM_LIMIT = Meters.of(0.0);

    /** Minimun height where the manipulator won't hit the elevator when it's holding coral */
    public static final Distance ELEVATOR_SAFE_HEIGHT = Meters.of(0.017);
    public static final Distance ELEVATOR_SAFE_TARGET = Meters.of(0.05);
    /** Min of the range where the manipulator hits the belt when holding coral */

    /** Min of the zone that the arm is never allowed to be commanded into, or to pass through. */
    public static final Angle ARM_FORBIDDEN_ZONE_MIN = Rotations.of(0.9);
    /** Max of the zone that the arm is never allowed to be commanded into, or to pass through. */
    public static final Angle ARM_FORBIDDEN_ZONE_MAX = Rotations.of(0.999999);

    public static final Distance ELEVATOR_POSITION_TOLERANCE = Inches.of(0.5);
    public static final Angle ARM_POSITION_TOLERANCE = Degrees.of(5);

    public static final Distance ELEVATOR_PARK_HEIGHT = Meters.of(0.0);
    public static final Distance ELEVATOR_PARK_TOLERANCE = Meters.of(0.01);
    public static final Angle ARM_PARK_ANGLE = Rotations.of(0.35);

    public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(40);
    public static final double ARM_ROTOR_TO_SENSOR_RATIO = (68.0 / 14.0) * (68.0 / 26.0) * (34.0 / 16.0);
    public static final Angle ARM_MAGNETIC_OFFSET = Rotations.of(-0.04248046875);

    public static final SlotConfigs ARM_SLOT_CONFIGS = new SlotConfigs().withGravityType(Arm_Cosine)
        .withKP(30.0)
        .withKD(0.0)
        .withKS(0.15341)
        .withKG(0.4) // Volts
        .withKV(0.65) // V*s/rotation
        .withKA(0.03); // V*s^2/rotation

    public static final MotionMagicConfigs ARM_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(4.0)
        .withMotionMagicCruiseVelocity(5.0);

    public static final Distance ELEVATOR_INTAKE_POSITION = Meters.of(0);
    public static final Distance LEVEL_2_HEIGHT = Meters.of(0.62);
    public static final Distance LEVEL_3_HEIGHT = Meters.of(0.0);
    public static final Distance LEVEL_4_HEIGHT = Meters.of(0.71);
    public static final Distance LEVEL_4_HEIGHT_RELEASE = Meters.of(0.71);
    public static final Distance ALGAE_LOWER_HEIGHT = Meters.of(0.0);
    public static final Distance ALGAE_UPPER_HEIGHT = Meters.of(0.25);
    public static final Distance ALGAE_BARGE_HEIGHT = Meters.of(0.727);
    public static final Distance ALGAE_PROCESSOR_HEIGHT = Meters.of(0.0);
    public static final Distance ALGAE_HOLD_HEIGHT = Meters.zero();

    public static final Angle ARM_INTAKE_ANGLE = Rotations.of(0.765);
    public static final Angle LEVEL_2_ANGLE = Rotations.of(0.72);
    public static final Angle LEVEL_3_ANGLE_ALIGN = Rotations.of(0.174);
    public static final Angle LEVEL_3_ANGLE_RELEASE = Rotations.of(0.16);
    public static final Angle LEVEL_4_ANGLE_ALIGN = Rotations.of(0.18);
    public static final Angle LEVEL_4_ANGLE_RELEASE = Rotations.of(0.16);
    public static final Angle ALGAE_LOWER_ANGLE = Rotations.of(0.05);
    public static final Angle ALGAE_UPPER_ANGLE = Rotations.of(0.105);
    public static final Angle ALGAE_BARGE_ANGLE = Rotation.of(0.33);
    public static final Angle ALGAE_PROCESSOR_ANGLE = Rotation.of(0.963);
    public static final Angle ALGAE_HOLD_ANGLE = Rotations.of(0.4);

    // Values for Mechanism2d visualization
    public static final Distance ARM_PIVOT_LENGTH = Meters.of(0.577);
    public static final Distance ELEVATOR_BASE_HEIGHT = Meters.of(1.0);
  }

  /**
   * Constants for the game piece manipulator
   */
  public static class GamePieceManipulatorConstants {

    public static final int DEVICE_ID_MANIPULATOR_MOTOR = 20;

    public static final SlotConfigs MANIPULATION_SLOT_CONFIGS = new SlotConfigs().withKP(10)
        .withKD(0.0)
        .withKS(20.0)
        .withKV(0.25);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(150);
    public static final Current TORQUE_CURRENT_LIMIT = Amps.of(50);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

    public static final Current WHEEL_HOLD_CORAL_CURRENT = Amps.of(5.0);
    public static final Current WHEEL_HOLD_ALGAE_CURRENT = Amps.of(-20.0);

    public static final AngularVelocity INTAKE_ALGAE_VELOCITY = RotationsPerSecond.of(-20);

  }

  /**
   * Constants for test mode
   */
  public static class TestingConstants {
    public static final AngularVelocity INDEXER_TESTING_SPEED_TOLERANCE = RotationsPerSecond.of(3);
    public static final AngularVelocity MANIPULATOR_TESTING_SPEED_TOLERANCE = RotationsPerSecond.of(2);
    public static final AngularVelocity WHEEL_VELOCITY_TOLERANCE = RotationsPerSecond.of(3);
  }

  /**
   * Constants for the MitoCANDria
   */
  public static class MitoCANDriaConstants {
    public static final int DEVICE_ID_MITOCANDRIA = 0;
  }

  /** Constants for aligning to the reef */
  public static class AlignmentConstants {
    public static final int DEVICE_ID_FRONT_CANRANGE = 38;
    public static final int DEVICE_ID_BACK_CANRANGE = 39;

    public static final LinearVelocity MAX_ALIGN_TRANSLATION_VELOCITY = MetersPerSecond.of(1);
    public static final LinearAcceleration MAX_ALIGN_TRANSLATION_ACCELERATION = MetersPerSecondPerSecond.of(1);
    public static final AngularVelocity MAX_ALIGN_ANGULAR_VELOCITY = MAX_TELEOP_ANGULAR_VELOCITY.times(0.75);
    public static final AngularAcceleration MAX_ALIGN_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond
        .of(6.0 * Math.PI);

    public static final double ALIGN_DISTANCE_kP = 8.0;
    public static final double ALIGN_DISTANCE_kI = 0.0;
    public static final double ALIGN_DISTANCE_kD = 0.0;

    public static final double ALIGN_LATERAL_kP = 8.0;
    public static final double ALIGN_LATERAL_kI = 0.0;
    public static final double ALIGN_LATERAL_kD = 0.0;

    public static final double ALIGN_THETA_kP = 6.0;
    public static final double ALIGN_THETA_kI = 0.0;
    public static final double ALIGN_THETA_kD = 0.0;

    public static final double SIGNAL_STRENGTH_THRESHOLD = 2000;

    /** Pose of the robot relative to a reef branch for scoring coral on L4 */
    public static final Transform2d RELATIVE_SCORING_POSE_CORAL_L4 = new Transform2d(
        inchesToMeters(-39),
        inchesToMeters(10),
        Rotation2d.fromDegrees(-90));
    /** Pose of the robot relative to a reef branch for scoring coral on L3 */
    public static final Transform2d RELATIVE_SCORING_POSE_CORAL_L3 = new Transform2d(
        inchesToMeters(-38),
        inchesToMeters(9.75),
        Rotation2d.fromDegrees(-90));

    // spotless:off
    /* The reef branches are in the arrays like this:
     *    ----------------------------------------
     *    |     5  / \ 6      |     11 / \ 0     |
     *    B    4 /     \ 7    |   10 /     \ 1   |
     *    L   3 |       | 8   |   9 |       | 2  R
     * +X U   2 |       | 9   |   8 |       | 3  E
     *    E    1 \     / 10   |    7 \     / 4   D
     *    |      0 \ / 11     |      6 \ / 5     |
     *    |___________________|__________________|
     * (0, 0)               +Y
     */
    // spotless:on
    /**
     * Poses of the right branches on the blue reef. Translation is the branch pipe base, rotation is pointing toward
     * reef center.
     */
    public static final List<Pose2d> REEF_BRANCH_POSES_BLUE_RIGHT = Stream
        .of(
            new Pose2d(4.347746, 3.467, Rotation2d.fromDegrees(60)), // 0
              new Pose2d(3.942648, 3.840490, Rotation2d.fromDegrees(0)), // 2
              new Pose2d(4.062584, 4.398912, Rotation2d.fromDegrees(-60)), // 4
              new Pose2d(4.588763, 4.542161, Rotation2d.fromDegrees(-120)), // 6
              new Pose2d(4.98, 4.215, Rotation2d.fromDegrees(180)), // 8
              new Pose2d(4.873353, 3.632614, Rotation2d.fromDegrees(120))) // 10
        .collect(toUnmodifiableList());

    /**
     * Poses of the left branches on the blue reef. Translation is the branch pipe base, rotation is pointing toward
     * reef center.
     */
    public static final List<Pose2d> REEF_BRANCH_POSES_BLUE_LEFT = Stream
        .of(
            new Pose2d(4.062584, 3.630770, Rotation2d.fromDegrees(60)), // 1
              new Pose2d(3.942648, 4.169106, Rotation2d.fromDegrees(0)), // 3
              new Pose2d(4.347175, 4.515, Rotation2d.fromDegrees(-60)), // 5
              new Pose2d(4.873926, 4.378820, Rotation2d.fromDegrees(-120)), // 7
              new Pose2d(4.994328, 3.841097, Rotation2d.fromDegrees(180)), // 9
              new Pose2d(4.589334, 3.466500, Rotation2d.fromDegrees(120)))// 11
        .collect(toUnmodifiableList());

    /**
     * Poses of the right branches on the red reef. Translation is the branch pipe base, rotation is pointing toward
     * reef center.
     */
    public static final List<Pose2d> REEF_BRANCH_POSES_RED_RIGHT = Stream
        .of(
            new Pose2d(13.200254, 4.585000, Rotation2d.fromDegrees(-120)), // 0
              new Pose2d(13.605352, 4.211510, Rotation2d.fromDegrees(-180)), // 2
              new Pose2d(13.485416, 3.653088, Rotation2d.fromDegrees(120)), // 4
              new Pose2d(12.959237, 3.509839, Rotation2d.fromDegrees(60)), // 6
              new Pose2d(12.568000, 3.837000, Rotation2d.fromDegrees(0)), // 8
              new Pose2d(12.598000, 4.292000, Rotation2d.fromDegrees(-60))) // 10
        .collect(toUnmodifiableList());

    /**
     * Poses of the left branches on the red reef. Translation is the branch pipe base, rotation is pointing toward reef
     * center.
     */
    public static final List<Pose2d> REEF_BRANCH_POSES_RED_LEFT = Stream
        .of(
            new Pose2d(13.485416, 4.421230, Rotation2d.fromDegrees(-120)), // 1
              new Pose2d(13.605352, 3.882894, Rotation2d.fromDegrees(-180)), // 3
              new Pose2d(13.200825, 3.537000, Rotation2d.fromDegrees(120)), // 5
              new Pose2d(12.674074, 3.673180, Rotation2d.fromDegrees(60)), // 7
              new Pose2d(12.553672, 4.210903, Rotation2d.fromDegrees(0)), // 9
              new Pose2d(12.958666, 4.585500, Rotation2d.fromDegrees(-60)))// 11
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L4 left branches on the red alliance */
    public static final List<Pose2d> REEF_L4_SCORE_POSES_RED_LEFT = REEF_BRANCH_POSES_RED_LEFT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L4 right branches on the red alliance */
    public static final List<Pose2d> REEF_L4_SCORE_POSES_RED_RIGHT = REEF_BRANCH_POSES_RED_RIGHT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L4 left branches on the blue alliance */
    public static final List<Pose2d> REEF_L4_SCORE_POSES_BLUE_LEFT = REEF_BRANCH_POSES_BLUE_LEFT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L4 right branches on the blue alliance */
    public static final List<Pose2d> REEF_L4_SCORE_POSES_BLUE_RIGHT = REEF_BRANCH_POSES_BLUE_RIGHT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L3 left branch on the red alliance */
    public static final List<Pose2d> REEF_L3_SCORE_POSES_RED_LEFT = REEF_BRANCH_POSES_RED_LEFT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L3 right branch on the red alliance */
    public static final List<Pose2d> REEF_L3_SCORE_POSES_RED_RIGHT = REEF_BRANCH_POSES_RED_RIGHT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L3 left branch on the blue alliance */
    public static final List<Pose2d> REEF_L3_SCORE_POSES_BLUE_LEFT = REEF_BRANCH_POSES_BLUE_LEFT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L3 right branch on the blue alliance */
    public static final List<Pose2d> REEF_L3_SCORE_POSES_BLUE_RIGHT = REEF_BRANCH_POSES_BLUE_RIGHT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        .collect(toUnmodifiableList());

    public static final Distance DISTANCE_TARGET_L4 = Meters.of(0.37);
    public static final Distance DISTANCE_TARGET_L3 = Meters.of(0.34);

    public static final Distance LATERAL_TARGET_L3_LEFT = Meters.of(0.05);
    public static final Distance LATERAL_TARGET_L3_RIGHT = Meters.of(0.02);

    public static final Distance LATERAL_TARGET_L4_LEFT = Meters.of(0.05);
    public static final Distance LATERAL_TARGET_L4_RIGHT = Meters.of(0.03);
  }

  /**
   * Constants for the LEDs
   */
  public static class LEDConstants {
    public static final int DEVICE_ID_LEDS = 9;

    public static final int LED_STRIP_LENGTH = 49;

    public static final int TOTAL_LEDS = 2 * LED_STRIP_LENGTH;
  }
}