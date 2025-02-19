package frc.robot.commands;

import static frc.robot.Constants.AlignmentConstants.REEF_BRANCH_POSES_BLUE;
import static frc.robot.Constants.AlignmentConstants.REEF_BRANCH_POSES_RED;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Command to drive to the scoring pose at the nearest reef node.
 */
public class DriveToReefCommand extends DriveToPoseCommand {

  /**
   * Constructs a DriveToReefCommand
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param poseProvider provider to call to get the robot pose
   */
  public DriveToReefCommand(CommandSwerveDrivetrain drivetrainSubsystem, Supplier<Pose2d> poseProvider) {
    super(drivetrainSubsystem, poseProvider);
  }

  /**
   * Constructs a DriveToPoseCommand with specific motion profile constraints
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param poseProvider provider to call to get the robot pose
   * @param translationConstraints translation motion profile constraints
   * @param omegaConstraints rotation motion profile constraints
   */
  public DriveToReefCommand(
      CommandSwerveDrivetrain drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      TrapezoidProfile.Constraints translationConstraints,
      TrapezoidProfile.Constraints omegaConstraints) {
    super(drivetrainSubsystem, poseProvider, translationConstraints, omegaConstraints);
  }

  @Override
  public void initialize() {
    var robotPose = poseProvider.get();
    var isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    setGoal(robotPose.nearest(isRed ? REEF_BRANCH_POSES_RED : REEF_BRANCH_POSES_BLUE));
    super.initialize();
  }

}