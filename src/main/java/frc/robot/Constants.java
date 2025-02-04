package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
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
   * Constants for vision processing
   */
  public static class VisionConstants {
    public static final String kCameraName = "YOUR CAMERA NAME";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam = new Transform3d(
        new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  /**
   * Constants for the climb subsystem
   */
  public static class ClimbConstants {
    public static final int DEVICE_ID_CLIMB_MOTOR_FRONT = 60;
    public static final int DEVICE_ID_CLIMB_MOTOR_BACK = 65;
    public static final int DEVICE_ID_CLIMB_ENCODER_FRONT = 61;
    public static final int DEVICE_ID_CLIMB_ENCODER_BACK = 66;

    public static final Angle CLIMB_MAGNETIC_OFFSET_FRONT = Rotations.of(0.0);
    public static final Angle CLIMB_MAGNETIC_OFFSET_BACK = Rotations.of(0.0);

    // configuraton {
    public static final Current CLIMB_STATOR_CURRENT_LIMIT = Amps.of(100);
    public static final Current CLIMB_SUPPLY_CURRENT_LIMIT = Amps.of(40);
    public static final double CLIMB_ROTOR_TO_SENSOR_RATIO = (25 / 1); // 25 rotor turns = 1 mechanism turn

    public static final Angle LEFT_CLIMB_MAGNETIC_OFFSET = Radians.of(0);
    public static final Angle RIGHT_CLIMB_MAGNETIC_OFFSET = Radians.of(0);

    public static final Voltage MAX_CLIMB_VOLTAGE = Volts.of(2);
  }

  /**
   * Constants for the arm subsystem
   */
  public static class ArmConstants {
    public static final int DEVICE_ID_ELEVATOR_MOTOR_LEADER = 80;
    public static final int DEVICE_ID_ELEVATOR_MOTOR_FOLLOWER = 81;
    public static final int DEVICE_ID_ELEVATOR_CANDI = 85;

    public static final int DEVICE_ID_ARM_MOTOR = 90;
    public static final int DEVICE_ID_ARM_CANDI = 95;

    public static final SlotConfigs ELEVATOR_SLOT_CONFIGS = new SlotConfigs().withKP(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKG(0.34) // Volts
        .withKV(2.22) // V*s/m
        .withKA(0.05); // V*s^2/m

    public static final MotionMagicConfigs ELEVATOR_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(0.01)
        .withMotionMagicCruiseVelocity(0.01);

    public static final Current ELEVATOR_SUPPLY_CURRENT_LIMIT = Amps.of(80);
    public static final Current ELEVATOR_STATOR_CURRENT_LIMIT = Amps.of(100);

    public static final double ELEVATOR_ROTOR_GEAR_RATIO = 2.85714286;
    public static final Distance ELEVATOR_GEAR_DIAMETER = Inches.of(2);
    public static final Distance ELEVATOR_GEAR_CIRCUMFERENCE = ELEVATOR_GEAR_DIAMETER.times(Math.PI);
    public static final Distance ELEVATOR_DISTANCE_PER_ROTATION = ELEVATOR_GEAR_CIRCUMFERENCE
        .div(ELEVATOR_ROTOR_GEAR_RATIO);

    public static final Distance ELEVATOR_TOP_LIMIT = Meters.of(1.0); // Placeholder
    public static final Distance ELEVATOR_BOTTOM_LIMIT = Meters.of(0.0);

    public static final Distance ELEVATOR_POSITION_TOLERANCE = Inches.of(0.5);
    public static final Angle ARM_POSITION_TOLERANCE = Degrees.of(1);

    public static final Distance ELEVATOR_DEFAULT_HEIGHT = Meters.of(0);

    public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(40);
    public static final double ARM_ROTOR_TO_SENSOR_RATIO = 45;
    public static final Angle ARM_MAGNETIC_OFFSET = Rotations.of(0.0);
    public static final double ARM_SENSOR_TO_MECHANISM_RATIO = 1.0;

    public static final SlotConfigs ARM_SLOT_CONFIGS = new SlotConfigs().withKP(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKG(0.12) // Volts
        .withKV(5.59) // V*s/rotation
        .withKA(0.03); // V*s^2/rotation

    public static final MotionMagicConfigs ARM_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(0.01)
        .withMotionMagicCruiseVelocity(0.01);

    public static final Distance LEVEL_1_HEIGHT = Meters.of(0);
    public static final Distance LEVEL_2_HEIGHT = Meters.of(0);
    public static final Distance LEVEL_3_HEIGHT = Meters.of(0);
    public static final Distance LEVEL_4_HEIGHT = Meters.of(0);

    public static final Angle LEVEL_2_ANGLE = Radian.of(0);
    public static final Angle LEVEL_3_ANGLE = Radian.of(0);
    public static final Angle LEVEL_4_ANGLE = Radian.of(0);
    public static final Angle INTAKE_ANGLE = Radian.of(0);

    // Values for Mechanism2d visualization
    public static final Distance ARM_PIVOT_LENGTH = Meters.of(0.577);
    public static final Distance ELEVATOR_BASE_HEIGHT = Meters.of(1.0);
  }

  /**
   * Constants for the game piece manipulator
   */
  public static class GamePieceManipulatorConstants {

    public static final int DEVICE_ID_MANIPULATOR_MOTOR = 50;
    public static final int DEVICE_ID_GAME_PIECE_CANRANGE = 71;

    public static final SlotConfigs MANIPULATION_SLOT_CONFIGS = new SlotConfigs().withKP(0.0).withKD(0.0).withKS(0.0);
    public static final SlotConfigs HOLD_SLOT_CONFIGS = new SlotConfigs().withKP(0.0).withKD(0.0);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(20);
    public static final Current TORQUE_CURRENT_LIMIT = Amps.of(20);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(10);

    public static final Distance CORAL_DETECTION_THRESHOLD = Millimeters.of(2);

    public static final AngularVelocity INAKE_VELOCITY = RadiansPerSecond.of(5);
    public static final AngularVelocity EJECT_VELOCITY = RadiansPerSecond.of(-5);
    public static final AngularVelocity SCORE_VELOCITY = RadiansPerSecond.of(-5);

    public static final AngularVelocity WHEEL_SPEED_TOLERANCE = RotationsPerSecond.of(3);

  }

  /**
   * Constants for test mode
   */
  public static class TestingConstants {
    public static final AngularVelocity INDEXER_TESTING_SPEED = RadiansPerSecond.of(2);
    public static final AngularVelocity INDEXER_BACKWARDS_TESTING_SPEED = INDEXER_TESTING_SPEED.unaryMinus();
    public static final AngularVelocity INDEXER_TESTING_SPEED_TOLERANCE = RotationsPerSecond.of(3);

    public static final AngularVelocity MANIPULATOR_TESTING_SPEED = RadiansPerSecond.of(2);
    public static final AngularVelocity MANIPULATOR_BACKWARDS_TESTING_SPEED = MANIPULATOR_TESTING_SPEED.unaryMinus();
    public static final AngularVelocity MANIPULATOR_TESTING_SPEED_TOLERANCE = RotationsPerSecond.of(3);

    public static final AngularVelocity ROLLER_TESTING_SPEED = RadiansPerSecond.of(5);
    public static final AngularVelocity ROLLER_BACKWARDS_TESTING_SPEED = ROLLER_TESTING_SPEED.unaryMinus();

    public static final Voltage CLIMB_TESTING_VOLTAGE = Volts.of(5);
  }
}
