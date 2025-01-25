package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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
    public static final int DEVICE_ID_CLIMB_MOTOR_1 = 60;
    public static final int DEVICE_ID_CLIMB_MOTOR_2 = 65;
    public static final int DEVICE_ID_CLIMB_ENCODER_1 = 61;
    public static final int DEVICE_ID_CLIMB_DETECTION = 67;

    // configuraton {
    public static final Current CLIMB_STATOR_CURRENT_LIMIT = Amps.of(100);
    public static final Current CLIMB_SUPPLY_CURRENT_LIMIT = Amps.of(40);
    public static final double CLIMB_ROTOR_TO_SENSOR_RATIO = (25 / 1); // 25 rotor turns = 1 mechanism turn

    public static final Angle CLIMB_LIMIT_FORWARD = Radians.of(3.1);
    public static final Angle CLIMB_LIMIT_REVERSE = Radians.of(-3.08);

    public static final Distance CAGE_DETECTION_THRESHOLD_DISTANCE = Millimeters.of(0); // TODO determine distance

    public static final Angle LEFT_CLIMB_MAGNETIC_OFFSET = Radians.of(0);
    public static final Angle RIGHT_CLIMB_MAGNETIC_OFFSET = Radians.of(0);

    public static final Voltage MAX_CLIMB_VOLTAGE = Volts.of(2);

  }

  /**
   * Constants for the algae subsystem
   */
  public static class AlgaeConstants {

  }

  /**
   * Constants for the indexer subsystem
   */
  public static class IndexerConstants {
    public static final int DEVICE_ID_BELT = 70;

    public static final AngularVelocity INTAKE_VELOCITY = RadiansPerSecond.of(1);
    public static final AngularVelocity SCORE_VELOCITY_LEVEL_1 = RadiansPerSecond.of(-1);
    public static final AngularVelocity EJECT_VELOCITY = RadiansPerSecond.of(-2);

    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(10); // Placeholder, will have to change

    public static final SlotConfigs SLOT_CONFIGS = new SlotConfigs().withKP(0.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0);
  }

  /*
   * Constants for the arm subsystem
   */
  public static class ArmConstants {
    public static final int DEVICE_ID_ELEVATOR_MOTOR_1 = 80;
    public static final int DEVICE_ID_ELEVATOR_MOTOR_2 = 81;
    public static final int DEVICE_ID_CANDI = 85;

    public static final SlotConfigs SLOT_CONFIGS = new SlotConfigs().withKP(0.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(0.01)
        .withMotionMagicCruiseVelocity(0.01);

    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(0); // Placeholder

    public static final Distance METERS_PER_REVOLUTION = Meters.of(0); // Placeholder

    public static final Distance TOP_LIMIT = Meters.of(0); // Placeholder
    public static final Distance BOTTOM_LIMIT = Meters.of(0); // Placeholder

    /*
     * The position in meters the elevator has to arrive at in order to score with placeholder numbers for now
     */
    public static final Distance LEVEL_1_HEIGHT = Meters.of(0);
    public static final Distance LEVEL_2_HEIGHT = Meters.of(0);
    public static final Distance LEVEL_3_HEIGHT = Meters.of(0);
    public static final Distance LEVEL_4_HEIGHT = Meters.of(0);

  }

  /*
   * Constants for the game piece manipulator
   */
  public static class GamePieceManipulatorConstants {
    public static final AngularVelocity INTAKE_SPEED = RadiansPerSecond.of(5);

    public static final AngularVelocity OUTTAKE_SPEED = RadiansPerSecond.of(-5);

    public static final AngularVelocity SCORE_SPEED = RadiansPerSecond.of(-5);

    public static final int DEVICE_ID_MANIPULATORMOTOR = 50;
  }

  /*
   * Constants for test mode
   */
  public static class TestingConstants {
    public static final AngularVelocity INDEXER_TESTING_SPEED = RadiansPerSecond.of(2);
    public static final AngularVelocity INDEXER_BACKWARDS_TESTING_SPEED = RadiansPerSecond.of(-2);
    public static final AngularVelocity MANIPULATOR_TESTING_SPEED = RadiansPerSecond.of(2);
    public static final AngularVelocity MANIPULATOR_BACKWARDS_TESTING_SPEED = RadiansPerSecond.of(-2);
  }
}
