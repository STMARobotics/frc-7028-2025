package frc.robot;

import static com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
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
import edu.wpi.first.units.measure.Voltage;

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

  public static class VisionConstants {
    public static final String kCameraName = "YOUR CAMERA NAME";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam = new Transform3d(
        new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

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

    public static final Distance CAGE_DETECTION_THRESHOLD_DISTANCE = Millimeters.of(0); // TODO determine distance

    public static final Angle LEFT_CLIMB_MAGNETIC_OFFSET = Radians.of(0);
    public static final Angle RIGHT_CLIMB_MAGNETIC_OFFSET = Radians.of(0);

    public static final Voltage MAX_CLIMB_VOLTAGE = Volts.of(2);

  }

  /**
   * Constants for the algae subsystem
   */
  public static class AlgaeConstants {
    public static final int DEVICE_ID_ROLLERMOTOR = 40;
    public static final int DEVICE_ID_WRISTMOTOR = 45;
    public static final int DEVICE_ID_CANRANGE = 46;
    public static final int DEVICE_ID_CANCODER = 47;

    public static final Angle WRIST_ENCODER_OFFSET = Rotations.of(0.0);

    public static final Current WRIST_STATOR_CURRENT_LIMIT = Amps.of(100);
    public static final Current WRIST_SUPPLY_CURRENT_LIMIT = Amps.of(40);
    public static final double WRIST_ROTOR_TO_SENSOR_RATIO = 1; // TODO Need to get this from design team

    public static final Angle WRIST_SOFT_LIMIT_FORWARD = Rotations.of(0.4);
    public static final Angle WRIST_SOFT_LIMIT_REVERSE = Rotations.of(0.0);

    // roller constants
    public static final AngularVelocity INTAKE_SPEED = RadiansPerSecond.of(5); // 5 is probably a wonky number
    public static final AngularVelocity EJECT_SPEED = RadiansPerSecond.of(-5);
    public static final AngularVelocity SCORE_SPEED = RadiansPerSecond.of(-5); // score speed probably lower number

    // wrist constants
    public static final AngularVelocity WRIST_DOWN_SPEED = RadiansPerSecond.of(-5);
    public static final AngularVelocity WRIST_UP_SPEED = RadiansPerSecond.of(5);
    public static final double ROLLER_SPEED_TOLERANCE = 0;

    // numbers are probably wonky here
    public static final Angle WRIST_DOWN_POSITION = Degrees.of(180);
    public static final Angle WRIST_UP_POSITION = Degrees.of(90);
    public static final Angle WRIST_PROCESSOR_POSITION = Rotations.of(0.2);
    public static final Angle WRIST_TOLERANCE = Degrees.of(1);

    public static final SlotConfigs WRIST_SLOT_CONFIGS = new SlotConfigs().withGravityType(Arm_Cosine)
        .withKP(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0)
        .withKA(0.0);

    public static final SlotConfigs ROLLER_SLOT_CONFIGS = new SlotConfigs().withKP(0.0).withKD(0.0).withKS(0.0);

    public static final MotionMagicConfigs WRIST_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5))
        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(5));

    public static final Current ROLLER_STATOR_CURRENT_LIMIT = Amps.of(100);
    public static final Current ROLLER_SUPPLY_CURRENT_LIMIT = Amps.of(40);

  }

  /**
   * Constants for the indexer subsystem
   */
  public static class IndexerConstants {
    public static final int DEVICE_ID_BELT = 70;

    public static final AngularVelocity INTAKE_VELOCITY = RadiansPerSecond.of(1);
    public static final AngularVelocity SCORE_VELOCITY_LEVEL_1 = RadiansPerSecond.of(-1);
    public static final AngularVelocity EJECT_VELOCITY = RadiansPerSecond.of(-2);
    public static final double INDEXER_SPEED_TOLERANCE = 0;

    public static final Current TORQUE_CURRENT_LIMIT = Amps.of(100);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(10); // Placeholder, will have to change

    public static final SlotConfigs SLOT_CONFIGS = new SlotConfigs().withKP(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0)
        .withKA(0.0);
  }

  /**
   * Constants for the arm subsystem
   */
  public static class ArmConstants {
    public static final int DEVICE_ID_ELEVATOR_MOTOR_LEADER = 80;
    public static final int DEVICE_ID_ELEVATOR_MOTOR_FOLLOWER = 81;
    public static final int DEVICE_ID_ELEVATOR_CANDI = 85;

    public static final int DEVICE_ID_ARM_MOTOR = 45;
    public static final int DEVICE_ID_ARM_LASERCAN = 46;
    public static final int DEVICE_ID_ARM_CANCODER = 47;

    public static final SlotConfigs ELEVATOR_SLOT_CONFIGS = new SlotConfigs().withKP(0.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0);

    public static final MotionMagicConfigs ELEVATOR_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(0.01)
        .withMotionMagicCruiseVelocity(0.01);

    public static final Current ELEVATOR_SUPPLY_CURRENT_LIMIT = Amps.of(0); // Placeholder

    public static final Distance ELEVATOR_METERS_PER_ROTATION = Meters.of(0); // Placeholder

    public static final Distance ELEVATOR_TOP_LIMIT = Meters.of(0); // Placeholder
    public static final Distance ELEVATOR_BOTTOM_LIMIT = Meters.of(0); // Placeholder

    public static final Angle ELEVATOR_POSITION_TOLERANCE = Rotations.of(0); // Placeholder
    public static final Angle ARM_POSITION_TOLERANCE = Rotations.of(0); // Placeholder

    /**
     * The positions in meters the elevator could travel to with placeholder numbers for now
     */
    public static final Distance ELEVATOR_DEFAULT_HEIGHT = Meters.of(0);
    public static final Distance LEVEL_1_HEIGHT = Meters.of(0);
    public static final Distance LEVEL_2_HEIGHT = Meters.of(0);
    public static final Distance LEVEL_3_HEIGHT = Meters.of(0);
    public static final Distance LEVEL_4_HEIGHT = Meters.of(0);

    public static final Angle LEVEL_2_ANGLE = Radian.of(0);
    public static final Angle LEVEL_3_ANGLE = Radian.of(0);
    public static final Angle LEVEL_4_ANGLE = Radian.of(0);
    public static final Angle INTAKE_ANGLE = Radian.of(0);

    public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(0);
    public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(0);
    public static final double ARM_ROTOR_TO_SENSOR_RATIO = 0;

    public static final Angle ARM_MAGNETIC_OFFSET = Rotations.of(0.0);

    public static final SlotConfigs ARM_SLOT_CONFIGS = new SlotConfigs().withKP(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0)
        .withKA(0.0);

    public static final MotionMagicConfigs ARM_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(0.01)
        .withMotionMagicCruiseVelocity(0.01);
  }

  /**
   * Constants for the game piece manipulator
   */
  public static class GamePieceManipulatorConstants {
    public static final AngularVelocity INTAKE_SPEED = RadiansPerSecond.of(5);
    public static final AngularVelocity EJECT_SPEED = RadiansPerSecond.of(-5);
    public static final AngularVelocity SCORE_SPEED = RadiansPerSecond.of(-5);

    public static final AngularVelocity INTAKE_ALGAE_VELOCITY = RadiansPerSecond.of(5);
    public static final AngularVelocity EJECT_ALGAE_VELOCITY = RadiansPerSecond.of(-5);
    public static final AngularVelocity SCORE_ALGAE_VELOCITY = RadiansPerSecond.of(-5);

    public static final int DEVICE_ID_MANIPULATOR_MOTOR = 50;

    public static final SlotConfigs MANIPULATION_SLOT_CONFIGS = new SlotConfigs().withKP(0.0).withKD(0.0).withKS(0.0);
    public static final SlotConfigs HOLD_SLOT_CONFIGS = new SlotConfigs().withKP(0.0).withKD(0.0);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(20);
    public static final Current TORQUE_CURRENT_LIMIT = Amps.of(20);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(10);
  }

  /**
   * Constants for test mode
   */
  public static class TestingConstants {
    public static final AngularVelocity INDEXER_TESTING_SPEED = RadiansPerSecond.of(2);
    public static final AngularVelocity INDEXER_BACKWARDS_TESTING_SPEED = RadiansPerSecond
        .of(-(INDEXER_TESTING_SPEED.in(RadiansPerSecond)));
    public static final double INDEXER_TESTING_TOLERANCE = 0;

    public static final AngularVelocity MANIPULATOR_TESTING_SPEED = RadiansPerSecond.of(2);
    public static final AngularVelocity MANIPULATOR_BACKWARDS_TESTING_SPEED = RadiansPerSecond
        .of(-(MANIPULATOR_TESTING_SPEED.in(RadiansPerSecond)));
    public static final double MANIPULATOR_TESTING_TOLERANCE = 0;

    public static final AngularVelocity ROLLER_TESTING_SPEED = RadiansPerSecond.of(5);
    public static final AngularVelocity ROLLER_BACKWARDS_TESTING_SPEED = RadiansPerSecond
        .of(-(ROLLER_TESTING_SPEED.in(RadiansPerSecond)));

    public static final Voltage CLIMB_TESTING_VOLTAGE = Volts.of(5);
  }
}
