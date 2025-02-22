package frc.robot.commands;

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

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  private static final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ALIGN_TRANSLATION_VELOCITY.in(MetersPerSecond),
      MAX_ALIGN_TRANSLATION_ACCELERATION.in(MetersPerSecondPerSecond));
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ALIGN_ANGULAR_VELOCITY.in(RadiansPerSecond),
      MAX_ALIGN_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

  private final ProfiledPIDController rotationController = new ProfiledPIDController(
      5.0,
      0.0,
      0.0,
      TRANSLATION_CONSTRAINTS);
  private final ProfiledPIDController distanceController = new ProfiledPIDController(5.0, 0.0, 0.0, OMEGA_CONSTRAINTS);

  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);

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
    var leftDistance = alignmentSubsystem.getLeftDistance().in(Meters);
    var rightDistance = alignmentSubsystem.getRightDistance().in(Meters);
    var rotation = rightDistance - leftDistance;
    var averageDistance = (leftDistance + rightDistance) / 2;

    rotationController.reset(rotation);
    distanceController.reset(averageDistance);

    distanceController.setGoal(targetDistance.in(Meters));
    rotationController.setGoal(0);
  }

  @Override
  public void execute() {
    var leftDistance = alignmentSubsystem.getLeftDistance().in(Meters);
    var rightDistance = alignmentSubsystem.getRightDistance().in(Meters);

    if (leftDistance < 1.2 && rightDistance < 1.2) {
      // We see something to align to
      var rotation = rightDistance - leftDistance;
      var averageDistance = (leftDistance + rightDistance) / 2;

      var rotationCorrection = rotationController.calculate(rotation);
      var distanceCorrection = -distanceController.calculate(averageDistance);

      drivetrain
          .setControl(robotCentricRequest.withRotationalRate(rotationCorrection).withVelocityY(distanceCorrection));
    } else {
      // We don't see anything or it's too far away to safely align
      drivetrain.setControl(new SwerveRequest.Idle());
    }
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
