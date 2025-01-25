package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
    public static final int DEVICE_ID_ROLLERMOTOR = 40;
    // I never saw what the id for the wrist motor would be, this is a placeholder
    public static final int DEVICE_ID_WRISTMOTOR = 45;
    // Same thing with the CANcoder
    public static final int DEVICE_ID_CANRANGE = 46;
    // Same thing with the CANcoder
    public static final int DEVICE_ID_CANCODER = 47;

    // roller constants
    public static final AngularVelocity INTAKE_SPEED = RadiansPerSecond.of(5); // 5 is probably a wonky number
    public static final AngularVelocity OUTTAKE_SPEED = RadiansPerSecond.of(-5);
    public static final AngularVelocity SCORE_SPEED = RadiansPerSecond.of(-5); // score speed probably lower number

    // wrist constants
    public static final AngularVelocity WRIST_DOWN_SPEED = RadiansPerSecond.of(-5);
    public static final AngularVelocity WRIST_UP_SPEED = RadiansPerSecond.of(5);

    // numbers are probably wonky here
    public static final Angle WRIST_DOWN_POSITION = Degrees.of(180);
    public static final Angle WRIST_UP_POSITION = Degrees.of(90);

    // Configs
    // I also have no idea what the numbers for these are, probably update them later
    public static final SlotConfigs ALGAE_SLOT_CONFIGS = new SlotConfigs().withKP(0.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0);
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
