package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;
import java.util.function.Supplier;

/**
 * Command to drive to the pose for the nearest to the list of poses provided.
 */
public class DriveToNearestPose extends DriveToPoseCommand {

  private final List<Pose2d> redPoses;
  private final List<Pose2d> bluePoses;

  /**
   * Constructs a DriveToNearestPose
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param poseProvider provider to call to get the robot pose
   * @param redPses list of poses for when the robot is on the red alliance
   * @param bluePoses list of poses for when the robot is on the blue alliance
   */
  public DriveToNearestPose(
      CommandSwerveDrivetrain drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      List<Pose2d> redPoses,
      List<Pose2d> bluePoses) {
    this(drivetrainSubsystem, poseProvider, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, redPoses, bluePoses);
  }

  /**
   * Constructs a DriveToReefCommand with specific motion profile constraints
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param poseProvider provider to call to get the robot pose
   * @param translationConstraints translation motion profile constraints
   * @param omegaConstraints rotation motion profile constraints
   * @param redPses list of poses for when the robot is on the red alliance
   * @param bluePoses list of poses for when the robot is on the blue alliance
   */
  public DriveToNearestPose(
      CommandSwerveDrivetrain drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      TrapezoidProfile.Constraints translationConstraints,
      TrapezoidProfile.Constraints omegaConstraints,
      List<Pose2d> redPoses,
      List<Pose2d> bluePoses) {
    super(drivetrainSubsystem, poseProvider, translationConstraints, omegaConstraints);
    this.redPoses = redPoses;
    this.bluePoses = bluePoses;
  }

  @Override
  public void initialize() {
    var robotPose = poseProvider.get();
    var isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    setGoal(robotPose.nearest(isRed ? redPoses : bluePoses));
    super.initialize();
  }

}