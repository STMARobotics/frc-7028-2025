package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import edu.wpi.first.units.measure.AngularVelocity;

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

    public static final SlotConfigs SLOT_CONFIGS = new SlotConfigs().withKP(0.0) // Not sure why spotless wants it this
                                                                                 // way
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

  }
}