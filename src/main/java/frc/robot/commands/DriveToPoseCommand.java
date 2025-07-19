package frc.robot.commands;

import static frc.robot.Constants.DriveToPoseConstants.DRIVE_TO_POSE_AUTOPILOT_PROFILE;
import static frc.robot.subsystems.LEDSubsystem.ledSegments;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import java.util.function.Supplier;

/**
 * Command to drive to a pose.
 */
public class DriveToPoseCommand extends Command {
  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Color ledColor;
  private final Autopilot autopilot = new Autopilot(DRIVE_TO_POSE_AUTOPILOT_PROFILE);

  private final APTarget autopilotTarget;

  // private final FieldCentric fieldCentricSwerveRequest = new FieldCentric()
  // .withSteerRequestType(SteerRequestType.MotionMagicExpo)
  // .withDriveRequestType(DriveRequestType.Velocity)
  // .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance); // Always Blue coordinate system for auto drive
  private final FieldCentricFacingAngle fieldCentricSwerveRequest = new FieldCentricFacingAngle().withDeadband(0)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
      .withHeadingPID(2.5, 0, 0);
  protected final Supplier<Pose2d> poseProvider;

  private final NetworkTable autoPilotTable = NetworkTableInstance.getDefault().getTable("AutoPilot");
  private final ProtobufPublisher<Transform2d> targetPublisher = autoPilotTable
      .getProtobufTopic("AutoPilot Output", Transform2d.proto)
      .publish();

  /**
   * Constructs a DriveToPoseCommand
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param ledSubsystem LED subsystem
   * @param goalPose goal pose to drive to
   */
  public DriveToPoseCommand(
      CommandSwerveDrivetrain drivetrainSubsystem,
      LEDSubsystem ledSubsystem,
      Color ledColor,
      Supplier<Pose2d> poseProvider,
      Pose2d goalPose) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.poseProvider = poseProvider;
    this.ledColor = ledColor;
    this.autopilotTarget = new APTarget(goalPose);

    addRequirements(drivetrainSubsystem, ledSubsystem);
  }

  /**
   * Constructs a DriveToPoseCommand with specific motion profile constraints
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param ledSubsystem LED subsystem
   * @param poseProvider provider to call to get the robot pose
   */
  public DriveToPoseCommand(
      CommandSwerveDrivetrain drivetrainSubsystem,
      LEDSubsystem ledSubsystem,
      Color ledColor,
      Supplier<Pose2d> poseProvider,
      Pose2d goalPose,
      Rotation2d entryAngle) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.poseProvider = poseProvider;
    this.ledColor = ledColor;
    this.autopilotTarget = new APTarget().withReference(goalPose).withEntryAngle(entryAngle);

    addRequirements(drivetrainSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    ledSubsystem.off();
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();

    var velocities = new Translation2d(
        drivetrainSubsystem.getState().Speeds.vxMetersPerSecond,
        drivetrainSubsystem.getState().Speeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());

    Transform2d output = autopilot.calculate(robotPose, velocities, autopilotTarget);

    targetPublisher.accept(output);

    drivetrainSubsystem.setControl(
        fieldCentricSwerveRequest.withVelocityX(output.getX())
            .withVelocityY(output.getY())
            .withTargetDirection(output.getRotation()));

    ledSubsystem.runPattern(ledSegments(ledColor, () -> autopilot.atTarget(robotPose, autopilotTarget)));
  }

  @Override
  public boolean isFinished() {
    return autopilot.atTarget(poseProvider.get(), autopilotTarget);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setControl(new SwerveRequest.Idle());
    ledSubsystem.off();
  }

}