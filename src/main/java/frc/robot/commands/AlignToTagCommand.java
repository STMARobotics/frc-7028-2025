package frc.robot.commands;

import static java.util.stream.Collectors.toUnmodifiableSet;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Set;
import java.util.stream.Stream;
import org.photonvision.PhotonCamera;

/**
 * Command to align parallel to the reef at a specific distance
 */
public class AlignToTagCommand extends Command {

  // TODO get tags on reef
  private final Set<Integer> FIDUCIAL_IDS = Stream.of(1, 2, 3, 4, 5).collect(toUnmodifiableSet());

  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonCamera photonCamera;

  private final PIDController distanceController = new PIDController(0, 0, 0);

  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  // TODO should we move the center of rotation so it rotates the sensor evenly?

  /**
   * Constructs a new command
   * 
   * @param drivetrain drivetrain subsystem
   */
  public AlignToTagCommand(CommandSwerveDrivetrain drivetrain, PhotonCamera photonCamera) {
    this.drivetrain = drivetrain;
    this.photonCamera = photonCamera;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    distanceController.reset();
  }

  @Override
  public void execute() {
    var photoResults = photonCamera.getAllUnreadResults();
    var lastTagResult = photoResults.stream()
        .filter(result -> result.hasTargets())
        .flatMap(result -> result.getTargets().stream())
        .filter(target -> FIDUCIAL_IDS.contains(target.getFiducialId()))
        .reduce((first, second) -> second);

    lastTagResult.ifPresentOrElse(tag -> {
      var cameraToTarget = tag.bestCameraToTarget;
      var correction = distanceController.calculate(cameraToTarget.getY());
      drivetrain.setControl(robotCentricRequest.withVelocityY(correction));
    }, () -> {
      // TODO what do you do here?
      // Did we ever see a tag, if so, keep moving that direction for until a timeout? What did we do in 2022?
    });

  }

  @Override
  public boolean isFinished() {
    return false; // TODO when to stop?
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }

}
