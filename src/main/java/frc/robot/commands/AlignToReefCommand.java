package frc.robot.commands;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.Constants.AlignmentConstants.ALIGN_DISTANCE_kD;
import static frc.robot.Constants.AlignmentConstants.ALIGN_DISTANCE_kI;
import static frc.robot.Constants.AlignmentConstants.ALIGN_DISTANCE_kP;
import static frc.robot.Constants.AlignmentConstants.ALIGN_LATERAL_kD;
import static frc.robot.Constants.AlignmentConstants.ALIGN_LATERAL_kI;
import static frc.robot.Constants.AlignmentConstants.ALIGN_LATERAL_kP;
import static frc.robot.Constants.AlignmentConstants.ALIGN_THETA_kD;
import static frc.robot.Constants.AlignmentConstants.ALIGN_THETA_kI;
import static frc.robot.Constants.AlignmentConstants.ALIGN_THETA_kP;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_ANGULAR_ACCELERATION;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_ANGULAR_VELOCITY;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_TRANSLATION_ACCELERATION;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_TRANSLATION_VELOCITY;
import static frc.robot.Constants.AlignmentConstants.SIGNAL_STRENGTH_THRESHOLD;
import static java.util.stream.Collectors.toUnmodifiableSet;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Stream;
import org.photonvision.PhotonCamera;

/**
 * Command to align parallel to the reef at a specific distance. This uses CANRanges and an AprilTag.
 */
public class AlignToReefCommand extends Command {

  // ID of the tags on the reef
  private static final Set<Integer> FIDUCIAL_IDS = Stream.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11)
      .collect(toUnmodifiableSet());

  private static final Distance DISTANCE_TOLERANCE = Inches.of(0.5);
  private static final Distance LATERAL_TOLERANCE = Inch.of(1.0);
  // Theta is the difference between the two CANrange distances, in meters
  private static final double THETA_TOLERANCE = 0.03;

  private final CommandSwerveDrivetrain drivetrain;
  private final AlignmentSubsystem alignmentSubsystem;
  private final PhotonCamera photonCamera;
  private final Distance targetDistance;

  private static final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ALIGN_TRANSLATION_VELOCITY.in(MetersPerSecond),
      MAX_ALIGN_TRANSLATION_ACCELERATION.in(MetersPerSecondPerSecond));
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ALIGN_ANGULAR_VELOCITY.in(RadiansPerSecond),
      MAX_ALIGN_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

  private final ProfiledPIDController distanceController = new ProfiledPIDController(
      ALIGN_DISTANCE_kP,
      ALIGN_DISTANCE_kI,
      ALIGN_DISTANCE_kD,
      TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController lateralController = new ProfiledPIDController(
      ALIGN_LATERAL_kP,
      ALIGN_LATERAL_kI,
      ALIGN_LATERAL_kD,
      TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
      ALIGN_THETA_kP,
      ALIGN_THETA_kI,
      ALIGN_THETA_kD,
      OMEGA_CONSTRAINTS);

  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final boolean allowScoreWithoutTag;
  private final double tagLateralTarget;
  private boolean sawTag = false;
  private boolean wasThetaAtGoal = false;

  /**
   * Constructs a new command
   * 
   * @param drivetrain drivetrain subsystem
   * @param alignmentSubsystem alignment subsystem
   * @param targetDistance distance the robot should be from the reef
   * @param tagLateralTarget lateral target location for the apriltag
   * @param photonCamera photon camera to use for AprilTag
   * @param allowScoreWithoutTag true to allow scoring if the apriltag was never seen, otherwise false
   */
  public AlignToReefCommand(
      CommandSwerveDrivetrain drivetrain,
      AlignmentSubsystem alignmentSubsystem,
      Distance targetDistance,
      Distance tagLateralTarget,
      PhotonCamera photonCamera,
      boolean allowScoreWithoutTag) {
    this.drivetrain = drivetrain;
    this.alignmentSubsystem = alignmentSubsystem;
    this.tagLateralTarget = tagLateralTarget.in(Meters);
    this.photonCamera = photonCamera;
    this.targetDistance = targetDistance;
    this.allowScoreWithoutTag = allowScoreWithoutTag;

    distanceController.setTolerance(DISTANCE_TOLERANCE.in(Meters));
    lateralController.setTolerance(LATERAL_TOLERANCE.in(Meters));
    thetaController.setTolerance(THETA_TOLERANCE);

    addRequirements(drivetrain, alignmentSubsystem);
  }

  @Override
  public void initialize() {
    distanceController.setGoal(targetDistance.in(Meters));
    thetaController.setGoal(0);
    lateralController.setGoal(tagLateralTarget);

    var frontDistance = alignmentSubsystem.getFrontDistance().in(Meters);
    var backDistance = alignmentSubsystem.getBackDistance().in(Meters);
    var theta = backDistance - frontDistance;
    var averageDistance = (frontDistance + backDistance) / 2;

    wasThetaAtGoal = false;

    thetaController.reset(theta);
    distanceController.reset(averageDistance);
    lateralController.reset(100);
    sawTag = false;
  }

  @Override
  public void execute() {
    boolean isThetaAtGoal = false;

    // Note: The robot is rotated 90 degrees, so robot Y is distance and X is lateral
    var frontDistance = alignmentSubsystem.getFrontDistance().in(Meters);
    var backDistance = alignmentSubsystem.getBackDistance().in(Meters);
    var frontSignalStrength = alignmentSubsystem.getFrontSignalStrength();
    var backSignalStrength = alignmentSubsystem.getBackSignalStrength();

    // We see something to align to
    var theta = backDistance - frontDistance;
    var averageDistance = (frontDistance + backDistance) / 2;

    if ((frontSignalStrength >= SIGNAL_STRENGTH_THRESHOLD) && (backSignalStrength >= SIGNAL_STRENGTH_THRESHOLD)) {
      // Strong distance sensor values, so try to align
      var thetaCorrection = thetaController.calculate(theta);
      if (thetaController.atGoal()) {
        isThetaAtGoal = true;
        thetaCorrection = 0;
      }
      // Look for an Apriltag to align laterally
      var tagYOpt = getTagY();
      if (tagYOpt.isPresent() && (isThetaAtGoal || wasThetaAtGoal)) {
        // Tag is visible, do alignment
        var tagY = tagYOpt.get();
        if (!sawTag || (isThetaAtGoal && !wasThetaAtGoal)) {
          // This is the first time a tag has been seen, or theta just got aligned reset controller
          lateralController.reset(tagY);
          sawTag = true;
        }
        var lateralCorrection = lateralController.calculate(tagY);
        if (lateralController.atGoal()) {
          lateralCorrection = 0;
        }
        robotCentricRequest.withVelocityX(lateralCorrection);
      } else {
        robotCentricRequest.withVelocityX(0);
      }

      wasThetaAtGoal = isThetaAtGoal || wasThetaAtGoal;
      var distanceCorrection = -distanceController.calculate(averageDistance);
      if (distanceController.atGoal() || !wasThetaAtGoal) {
        distanceCorrection = 0;
      }

      drivetrain.setControl(robotCentricRequest.withRotationalRate(thetaCorrection).withVelocityY(distanceCorrection));
    } else {
      // Weak distance sensor values, so try to get into a better position
      distanceController.reset(averageDistance);
      thetaController.reset(theta);
      if ((frontSignalStrength >= SIGNAL_STRENGTH_THRESHOLD) || (backSignalStrength >= SIGNAL_STRENGTH_THRESHOLD)) {
        // One signal is strong, so the other sensor is probably off the edge of the reef. Move toward the strong signal
        var velocityX = frontSignalStrength > backSignalStrength ? 0.5 : -0.5;
        drivetrain.setControl(robotCentricRequest.withRotationalRate(0).withVelocityY(0).withVelocityX(velocityX));
      } else {
        // Both sensor have weak signals so drive forward, hoping to find the reef
        drivetrain.setControl(robotCentricRequest.withRotationalRate(0).withVelocityY(0.5).withVelocityX(0));
      }
    }
  }

  private Optional<Double> getTagY() {
    var photoResults = photonCamera.getAllUnreadResults();
    var lastTagResult = photoResults.stream()
        .filter(result -> result.hasTargets())
        .flatMap(result -> result.getTargets().stream())
        .filter(target -> FIDUCIAL_IDS.contains(target.getFiducialId()))
        .findFirst();

    if (lastTagResult.isPresent()) {
      var tag = lastTagResult.get();
      var cameraToTarget = tag.bestCameraToTarget;
      return Optional.of(cameraToTarget.getY());
    }
    return Optional.empty();
  }

  @Override
  public boolean isFinished() {
    return atDistanceGoal() && atLateralGoal() && atThetaGoal();
  }

  /**
   * Indicates if the robot is at the distance target
   * 
   * @return true if the robot is at the distance target
   */
  public boolean atDistanceGoal() {
    return distanceController.atGoal();
  }

  /**
   * Indicates if the robot is at the lateral target
   * 
   * @return true if the robot is at the lateral target
   */
  public boolean atLateralGoal() {
    if (allowScoreWithoutTag) {
      return !sawTag || (sawTag && lateralController.atGoal());
    }
    return lateralController.atGoal();
  }

  /**
   * Indicates if the robot is at the theta target
   * 
   * @return true if the robot is at the theta target
   */
  public boolean atThetaGoal() {
    return thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }

}
