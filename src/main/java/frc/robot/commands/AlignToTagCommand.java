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
 * Command to align to an AprilTag on the reef
 */
public class AlignToTagCommand extends Command {

  // ID of the tags on the reef
  private final Set<Integer> FIDUCIAL_IDS = Stream.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11)
      .collect(toUnmodifiableSet());

  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonCamera photonCamera;

  private final PIDController positionController = new PIDController(5.0, 0, 0);

  private final double leftAlign = -0.2;
  private final double rightAlignY = 0.2;

  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);

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
    positionController.reset();
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
      if (Math.abs(cameraToTarget.getY() - leftAlign) < Math.abs(cameraToTarget.getY() - rightAlignY)) {
        positionController.setSetpoint(leftAlign);
      } else {
        positionController.setSetpoint(rightAlignY);
      }

      var correction = positionController.calculate(cameraToTarget.getY());
      drivetrain.setControl(robotCentricRequest.withVelocityX(correction));
    }, () -> {
      // TODO what do you do here?
      // Did we ever see a tag, if so, keep moving that direction for until a timeout? What did we do in 2022?
    });

  }

  @Override
  public boolean isFinished() {
    return positionController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }

}
