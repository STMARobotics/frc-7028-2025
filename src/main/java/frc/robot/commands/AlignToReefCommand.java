package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.Constants.AlignmentConstants.ALIGNMENT_TOLERANCE;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_ANGULAR_ACCELERATION;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_ANGULAR_VELOCITY;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_TRANSLATION_ACCELERATION;
import static frc.robot.Constants.AlignmentConstants.MAX_ALIGN_TRANSLATION_VELOCITY;
import static frc.robot.Constants.DriveToPoseConstants.THETA_kD;
import static frc.robot.Constants.DriveToPoseConstants.THETA_kI;
import static frc.robot.Constants.DriveToPoseConstants.THETA_kP;
import static frc.robot.Constants.DriveToPoseConstants.X_kD;
import static frc.robot.Constants.DriveToPoseConstants.X_kI;
import static frc.robot.Constants.DriveToPoseConstants.X_kP;
import static frc.robot.Constants.DriveToPoseConstants.Y_kD;
import static frc.robot.Constants.DriveToPoseConstants.Y_kI;
import static java.util.stream.Collectors.toUnmodifiableSet;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
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
  // Alignment, depending which reef branch we want to score on
  private static final double LEFT_TAG_ALIGNMENT = -0.23;
  private static final double RIGHT_TAG_ALIGNMENT = 0.08;

  private static final Distance TRANSLATION_TOLERANCE = Inches.of(0.5);
  private static final Distance TAG_TOLERANCE = Inch.of(0.6);
  private static final Angle THETA_TOLERANCE = Degrees.of(1.0);

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

  private final ProfiledPIDController robotYController = new ProfiledPIDController(
      X_kP,
      X_kI,
      X_kD,
      TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController robotXController = new ProfiledPIDController(
      6.0,
      Y_kI,
      Y_kD,
      TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
      THETA_kP,
      THETA_kI,
      THETA_kD,
      OMEGA_CONSTRAINTS);

  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private double alignmentTarget;
  private boolean sawTag = false;

  /**
   * Constructs a new command
   * 
   * @param drivetrain drivetrain subsystem
   * @param alignmentSubsystem alignment subsystem
   * @param targetDistance distance the robot should be from the reef
   */
  public AlignToReefCommand(
      CommandSwerveDrivetrain drivetrain,
      AlignmentSubsystem alignmentSubsystem,
      Distance targetDistance,
      PhotonCamera photonCamera) {
    this.drivetrain = drivetrain;
    this.alignmentSubsystem = alignmentSubsystem;
    this.photonCamera = photonCamera;
    this.targetDistance = targetDistance;

    robotYController.setTolerance(TRANSLATION_TOLERANCE.in(Meters));
    robotXController.setTolerance(TAG_TOLERANCE.in(Meters));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE.in(Radians));

    addRequirements(drivetrain, alignmentSubsystem);
  }

  @Override
  public void initialize() {
    robotYController.setGoal(targetDistance.in(Meters));
    thetaController.setGoal(0);

    var leftDistance = alignmentSubsystem.getLeftDistance().in(Meters);
    var rightDistance = alignmentSubsystem.getRightDistance().in(Meters);
    var rotation = rightDistance - leftDistance;
    var averageDistance = (leftDistance + rightDistance) / 2;

    thetaController.reset(rotation);
    robotYController.reset(averageDistance);

    getTagY().ifPresentOrElse(tagY -> {
      alignmentTarget = Math.abs(tagY - RIGHT_TAG_ALIGNMENT) < Math.abs(tagY - LEFT_TAG_ALIGNMENT) ? RIGHT_TAG_ALIGNMENT
          : LEFT_TAG_ALIGNMENT;
      robotXController.setGoal(alignmentTarget);
      robotXController.reset(tagY);
      sawTag = true;
    }, () -> {
      sawTag = false;
      robotXController.reset(0);
    });
  }

  @Override
  public void execute() {
    var leftDistance = alignmentSubsystem.getLeftDistance().in(Meters);
    var rightDistance = alignmentSubsystem.getRightDistance().in(Meters);
    var tagY = getTagY();

    if (leftDistance < 1.2 && rightDistance < 1.2) {
      // We see something to align to
      var rotation = rightDistance - leftDistance;
      var averageDistance = (leftDistance + rightDistance) / 2;

      var rotationCorrection = thetaController.calculate(rotation);
      var distanceCorrection = -robotYController.calculate(averageDistance);
      if (tagY.isPresent() && sawTag) {
        var xCorrection = robotXController.calculate(tagY.get());
        robotCentricRequest.withVelocityX(xCorrection);
      }

      drivetrain
          .setControl(robotCentricRequest.withRotationalRate(rotationCorrection).withVelocityY(distanceCorrection));
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
        .reduce((first, second) -> second);

    if (lastTagResult.isPresent()) {
      var tag = lastTagResult.get();
      var cameraToTarget = tag.bestCameraToTarget;
      return Optional.of(cameraToTarget.getY());
    }
    return Optional.empty();
  }

  @Override
  public boolean isFinished() {
    return alignmentSubsystem.getLeftDistance().isNear(targetDistance, ALIGNMENT_TOLERANCE)
        && alignmentSubsystem.getRightDistance().isNear(targetDistance, ALIGNMENT_TOLERANCE)
        && robotXController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }

}
