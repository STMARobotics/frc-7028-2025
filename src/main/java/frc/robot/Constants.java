package frc.robot;

import static com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;
import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
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
    public static final String[] CAMERA_NAMES = new String[] { "Left", "Front", "Right", "Back" };
    public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = new Transform3d[] {
        new Transform3d(
            new Translation3d(inchesToMeters(10.846), inchesToMeters(12.472), inchesToMeters(10.775)),
            new Rotation3d(0, 0, degreesToRadians(90))),
        new Transform3d(
            new Translation3d(inchesToMeters(12.610), inchesToMeters(-4.332), inchesToMeters(15.943)),
            new Rotation3d(0, 0, 0)),
        new Transform3d(
            new Translation3d(inchesToMeters(8.286), inchesToMeters(-12.691), inchesToMeters(13.642)),
            new Rotation3d(0, 0, degreesToRadians(-90))),
        new Transform3d(
            new Translation3d(inchesToMeters(-13.472), inchesToMeters(-8.088), inchesToMeters(8.541)),
            new Rotation3d(0, 0, degreesToRadians(180))) };

    public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);

    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
  }

  /**
   * Constants for the indexer subsystem
   */
  public static class IndexerConstants {
    public static final int DEVICE_ID_BELT = 35;
    public static final int DEVICE_ID_GAME_PIECE_CANRANGE = 36;

    public static final AngularVelocity INTAKE_VELOCITY = RadiansPerSecond.of(1);

    public static final AngularVelocity SCORE_VELOCITY_LEVEL_1 = RadiansPerSecond.of(-1);
    public static final AngularVelocity EJECT_VELOCITY = RadiansPerSecond.of(-2);
    public static final AngularVelocity INDEXER_SPEED_TOLERANCE = RotationsPerSecond.of(3);
    public static final Distance CORAL_DETECTION_THRESHOLD = Meters.of(0.076);

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

    public static final Angle CLIMB_FORWARD_SOFT_LIMIT = Rotations.of(0.831299);

    public static final Voltage CLIMB_VOLTAGE = Volts.of(6);
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

    public static final SlotConfigs ELEVATOR_SLOT_CONFIGS = new SlotConfigs().withKP(0.01)
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

    public static final double ELEVATOR_ROTOR_GEAR_RATIO = 2.85714286;
    public static final Measure<PerUnit<DistanceUnit, AngleUnit>> ELEVATOR_DISTANCE_PER_ROTATION = Meters.per(Rotation)
        .ofNative(0.05489);

    public static final Distance ELEVATOR_TOP_LIMIT = Meters.of(0.727075);
    public static final Distance ELEVATOR_BOTTOM_LIMIT = Meters.of(0.0);

    /** Minimun height where the manipulator won't hit the elevator when it's holding coral */
    public static final Distance ELEVATOR_SAFE_HEIGHT = Meters.of(0.017);
    public static final Distance ELEVATOR_SAFE_TARGET = Meters.of(0.05);
    /** Min of the range where the manipulator hits the belt when holding coral */
    public static final Angle ARM_DANGER_MIN = Rotations.of(0.775);
    /** Max of the range where the manipulator hits the belt when holding coral */
    public static final Angle ARM_DANGER_MAX = Rotations.of(0.874);
    public static final Angle ARM_DANGER_TOLERANCE = Degrees.of(20);

    /** Min of the zone that the arm is never allowed to be commanded into, or to pass through. */
    public static final Angle ARM_FORBIDDEN_ZONE_MIN = Rotations.of(0.8);
    /** Max of the zone that the arm is never allowed to be commanded into, or to pass through. */
    public static final Angle ARM_FORBIDDEN_ZONE_MAX = Rotations.of(0.9);

    public static final Distance ELEVATOR_POSITION_TOLERANCE = Inches.of(0.5);
    public static final Angle ARM_POSITION_TOLERANCE = Degrees.of(5);

    public static final Distance ELEVATOR_PARK_HEIGHT = Meters.of(0.0);
    public static final Distance ELEVATOR_PARK_TOLERANCE = Meters.of(0.01);
    public static final Angle ARM_PARK_ANGLE = Rotations.of(0.726074);

    public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(40);
    public static final double ARM_ROTOR_TO_SENSOR_RATIO = (68.0 / 14.0) * (68.0 / 26.0) * (34.0 / 16.0);
    public static final Angle ARM_MAGNETIC_OFFSET = Rotations.of(-0.299316);

    public static final SlotConfigs ARM_SLOT_CONFIGS = new SlotConfigs().withGravityType(Arm_Cosine)
        .withKP(30.0)
        .withKD(0.0)
        .withKS(0.15341)
        .withKG(0.4) // Volts
        .withKV(1.0) // V*s/rotation
        .withKA(0.03); // V*s^2/rotation

    public static final MotionMagicConfigs ARM_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(2.0)
        .withMotionMagicCruiseVelocity(6.0);

    public static final Distance ELEVATOR_INTAKE_POSITION = Meters.of(0);
    public static final Distance LEVEL_2_HEIGHT = Meters.of(0.1946611669921875);
    public static final Distance LEVEL_3_HEIGHT = Meters.of(0);
    public static final Distance LEVEL_4_HEIGHT = Meters.of(0.7186891357421875);
    public static final Distance ALGAE_LEVEL_1_HEIGHT = Meters.of(0.35);
    public static final Distance ALGAE_LEVEL_2_HEIGHT = Meters.of(0.9);

    public static final Angle ARM_INTAKE_ANGLE = Rotations.of(0.765);
    public static final Angle LEVEL_2_ANGLE = Rotations.of(0.68212890625);
    public static final Angle LEVEL_3_ANGLE = Rotations.of(0.172);
    public static final Angle LEVEL_4_ANGLE = Rotations.of(0.16);
    public static final Angle ALGAE_LEVEL_1_ANGLE = Rotations.of(0.94);
    public static final Angle ALGAE_LEVEL_2_ANGLE = Rotations.of(0.94);

    // Values for Mechanism2d visualization
    public static final Distance ARM_PIVOT_LENGTH = Meters.of(0.577);
    public static final Distance ELEVATOR_BASE_HEIGHT = Meters.of(1.0);
  }

  /**
   * Constants for the game piece manipulator
   */
  public static class GamePieceManipulatorConstants {

    public static final int DEVICE_ID_MANIPULATOR_MOTOR = 20;

    public static final SlotConfigs MANIPULATION_SLOT_CONFIGS = new SlotConfigs().withKP(0.0).withKD(0.0).withKS(0.0);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
    public static final Current TORQUE_CURRENT_LIMIT = Amps.of(10);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(20);

    public static final AngularVelocity INAKE_VELOCITY = RadiansPerSecond.of(5);
    public static final AngularVelocity EJECT_VELOCITY = RadiansPerSecond.of(-5);
    public static final AngularVelocity SCORE_VELOCITY = RadiansPerSecond.of(-5);

    public static final AngularVelocity WHEEL_VELOCITY_TOLERANCE = RotationsPerSecond.of(3);

    public static final Current WHEEL_HOLDING_CURRENT = Amps.of(8.0);

  }

  /**
   * Constants for test mode
   */
  public static class TestingConstants {
    public static final AngularVelocity INDEXER_TESTING_SPEED = RadiansPerSecond.of(2);
    public static final AngularVelocity INDEXER_BACKWARDS_TESTING_SPEED = INDEXER_TESTING_SPEED.unaryMinus();
    public static final AngularVelocity INDEXER_TESTING_SPEED_TOLERANCE = RotationsPerSecond.of(3);

    public static final AngularVelocity MANIPULATOR_TESTING_SPEED = RotationsPerSecond.of(10);
    public static final AngularVelocity MANIPULATOR_BACKWARDS_TESTING_SPEED = MANIPULATOR_TESTING_SPEED.unaryMinus();
    public static final AngularVelocity MANIPULATOR_TESTING_SPEED_TOLERANCE = RotationsPerSecond.of(3);

    public static final AngularVelocity ROLLER_TESTING_SPEED = RadiansPerSecond.of(5);
    public static final AngularVelocity ROLLER_BACKWARDS_TESTING_SPEED = ROLLER_TESTING_SPEED.unaryMinus();
  }

  /**
   * Constants for the MitoCANDria
   */
  public static class MitoCANDriaConstants {
    public static final int DEVICE_ID_MITOCANDRIA = 0;
  }

  public static class AlignmentConstants {
    public static final int DEVICE_ID_RIGHT_CANRANGE = 38;
    public static final int DEVICE_ID_LEFT_CANRANGE = 39;

    public static final Distance ALIGNMENT_TOLERANCE = Inches.of(1);
  }
}