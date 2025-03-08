package frc.robot.commands;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.Constants.AlignmentConstants.ALIGNMENT_TOLERANCE;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_ANGULAR_ACCELERATION;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_ANGULAR_VELOCITY;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_TRANSLATION_ACCELERATION;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_TRANSLATION_VELOCITY;
import static frc.robot.Constants.DriveToPoseConstants.X_kD;
import static frc.robot.Constants.DriveToPoseConstants.X_kI;
import static frc.robot.Constants.DriveToPoseConstants.X_kP;
import static frc.robot.Constants.DriveToPoseConstants.Y_kD;
import static frc.robot.Constants.DriveToPoseConstants.Y_kI;
import static frc.robot.Constants.DriveToPoseConstants.Y_kP;
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
      X_kP,
      X_kI,
      X_kD,
      TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController lateralController = new ProfiledPIDController(
      Y_kP,
      Y_kI,
      Y_kD,
      TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController thetaController = new ProfiledPIDController(5.0, 0.0, 0.0, OMEGA_CONSTRAINTS);

  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final boolean allowScoreWithoutTag;
  private final double tagLateralTarget;
  private boolean sawTag = false;

  /**
   * Constructs a new command
   * 
   * @param drivetrain drivetrain subsystem
   * @param alignmentSubsystem alignment subsystem
   * @param targetDistance distance the robot should be from the reef
   * @param photonCamera photon camera to use for AprilTag
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

    thetaController.reset(theta);
    distanceController.reset(averageDistance);
    lateralController.reset(100);
    sawTag = false;

    // If a tag is visible, set side-to-side goal
    getTagY().ifPresent(tagY -> initLateralTag(tagY));
  }

  private void initLateralTag(double tagY) {
    lateralController.reset(tagY);
    sawTag = true;
  }

  @Override
  public void execute() {
    var frontDistance = alignmentSubsystem.getFrontDistance().in(Meters);
    var backDistance = alignmentSubsystem.getBackDistance().in(Meters);

    if (frontDistance < 1.2 && backDistance < 1.2) {
      // We see something to align to
      var theta = backDistance - frontDistance;
      var averageDistance = (frontDistance + backDistance) / 2;

      double thetaCorrection = thetaController.calculate(theta);
      if (thetaController.atGoal()) {
        thetaCorrection = 0;
      }
      var distanceCorrection = -distanceController.calculate(averageDistance);
      if (distanceController.atGoal()) {
        distanceCorrection = 0;
      }
      getTagY().ifPresentOrElse(tagY -> {
        if (!sawTag) {
          // This is the first time a tag has been seen, set goal
          initLateralTag(tagY);
        }
        // Tag is visible, do alignment
        var lateralCorrection = lateralController.calculate(tagY);
        if (lateralController.atGoal()) {
          lateralCorrection = 0;
        }
        robotCentricRequest.withVelocityX(lateralCorrection);
      }, () -> robotCentricRequest.withVelocityX(0));

      // The robot is rotated 90 degrees, so Y is distance and X is lateral
      drivetrain.setControl(robotCentricRequest.withRotationalRate(thetaCorrection).withVelocityY(distanceCorrection));
    } else {
      // We don't see anything or it's too far away to safely align
      drivetrain.setControl(new SwerveRequest.Idle());
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
    return atDistanceGoal() && atLateralGoal();
  }

  public boolean atDistanceGoal() {
    return alignmentSubsystem.getFrontDistance().isNear(targetDistance, ALIGNMENT_TOLERANCE)
        && alignmentSubsystem.getBackDistance().isNear(targetDistance, ALIGNMENT_TOLERANCE);
  }

  public boolean atLateralGoal() {
    if (allowScoreWithoutTag) {
      return !sawTag || (sawTag && lateralController.atGoal());
    }
    return lateralController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }

}
