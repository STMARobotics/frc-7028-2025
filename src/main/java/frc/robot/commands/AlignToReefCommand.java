package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.AlignmentConstants.ALIGNMENT_TOLERANCE;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command to align parallel to the reef at a specific distance
 */
public class AlignToReefCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final AlignmentSubsystem alignmentSubsystem;
  private final Distance targetDistance;

  private final PIDController rotationController = new PIDController(0, 0, 0);
  private final PIDController distanceController = new PIDController(0, 0, 0);

  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  // TODO should we move the center of rotation so it rotates the sensor evenly?

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
      Distance targetDistance) {
    this.drivetrain = drivetrain;
    this.alignmentSubsystem = alignmentSubsystem;
    this.targetDistance = targetDistance;

    addRequirements(drivetrain, alignmentSubsystem);
  }

  @Override
  public void initialize() {
    rotationController.reset();
    distanceController.reset();
  }

  @Override
  public void execute() {
    var leftDistance = alignmentSubsystem.getLeftDistance().in(Meters);
    var rightDistance = alignmentSubsystem.getRightDistance().in(Meters);

    var rotation = leftDistance - rightDistance;
    var averageDistance = leftDistance + rightDistance / 2;

    var rotationCorrection = rotationController.calculate(rotation);
    var distanceCorrection = distanceController.calculate(averageDistance);

    drivetrain.setControl(robotCentricRequest.withRotationalRate(rotationCorrection).withVelocityX(distanceCorrection));
  }

  @Override
  public boolean isFinished() {
    return alignmentSubsystem.getLeftDistance().isNear(targetDistance, ALIGNMENT_TOLERANCE)
        && alignmentSubsystem.getRightDistance().isNear(targetDistance, ALIGNMENT_TOLERANCE);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }

}
