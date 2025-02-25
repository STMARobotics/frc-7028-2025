package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.Constants.DriveToPoseConstants.MAX_DRIVE_TO_POSE_ANGULAR_ACCELERATION;
import static frc.robot.Constants.DriveToPoseConstants.MAX_DRIVE_TO_POSE_ANGULAR_VELOCITY;
import static frc.robot.Constants.DriveToPoseConstants.MAX_DRIVE_TO_POSE_TRANSLATION_ACCELERATION;
import static frc.robot.Constants.DriveToPoseConstants.MAX_DRIVE_TO_POSE_TRANSLATION_VELOCITY;
import static frc.robot.Constants.DriveToPoseConstants.THETA_kD;
import static frc.robot.Constants.DriveToPoseConstants.THETA_kI;
import static frc.robot.Constants.DriveToPoseConstants.THETA_kP;
import static frc.robot.Constants.DriveToPoseConstants.X_kD;
import static frc.robot.Constants.DriveToPoseConstants.X_kI;
import static frc.robot.Constants.DriveToPoseConstants.X_kP;
import static frc.robot.Constants.DriveToPoseConstants.Y_kD;
import static frc.robot.Constants.DriveToPoseConstants.Y_kI;
import static frc.robot.Constants.DriveToPoseConstants.Y_kP;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Command to drive to a pose.
 */
public class DriveToPoseCommand extends Command {

  private static final Distance TRANSLATION_TOLERANCE = Inches.of(0.5);
  private static final Angle THETA_TOLERANCE = Degrees.of(1.0);

  protected static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_DRIVE_TO_POSE_TRANSLATION_VELOCITY.in(MetersPerSecond),
      MAX_DRIVE_TO_POSE_TRANSLATION_ACCELERATION.in(MetersPerSecondPerSecond));
  protected static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_DRIVE_TO_POSE_ANGULAR_VELOCITY.in(RadiansPerSecond),
      MAX_DRIVE_TO_POSE_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final FieldCentric fieldCentricSwerveRequest = new FieldCentric()
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance); // Always Blue coordinate system for auto drive
  protected final Supplier<Pose2d> poseProvider;

  /**
   * Constructs a DriveToPoseCommand
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param goalPose goal pose to drive to
   */
  public DriveToPoseCommand(CommandSwerveDrivetrain drivetrainSubsystem, Supplier<Pose2d> poseProvider) {
    this(drivetrainSubsystem, poseProvider, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS);
  }

  /**
   * Constructs a DriveToPoseCommand with specific motion profile constraints
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param poseProvider provider to call to get the robot pose
   * @param translationConstraints translation motion profile constraints
   * @param omegaConstraints rotation motion profile constraints
   */
  public DriveToPoseCommand(
      CommandSwerveDrivetrain drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      TrapezoidProfile.Constraints translationConstraints,
      TrapezoidProfile.Constraints omegaConstraints) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    xController = new ProfiledPIDController(X_kP, X_kI, X_kD, translationConstraints);
    xController.setTolerance(TRANSLATION_TOLERANCE.in(Meters));

    yController = new ProfiledPIDController(Y_kP, Y_kI, Y_kD, translationConstraints);
    yController.setTolerance(TRANSLATION_TOLERANCE.in(Meters));

    thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE.in(Radians));

    addRequirements(drivetrainSubsystem);
  }

  /**
   * Sets the goal to drive to. This should be set before the command is scheduled.
   * 
   * @param goalPose goal pose
   */
  public void setGoal(Pose2d goalPose) {
    thetaController.setGoal(goalPose.getRotation().getRadians());
    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
  }

  @Override
  public void initialize() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();

    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    drivetrainSubsystem.setControl(
        fieldCentricSwerveRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(omegaSpeed));
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setControl(new SwerveRequest.Idle());
  }

}