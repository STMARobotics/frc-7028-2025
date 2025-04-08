package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
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
   * @param ledSubsystem LED subsystem
   * @param ledColor color for the LED progress segments
   * @param poseProvider provider to call to get the robot pose
   * @param redPses list of poses for when the robot is on the red alliance
   * @param bluePoses list of poses for when the robot is on the blue alliance
   */
  public DriveToNearestPose(
      CommandSwerveDrivetrain drivetrainSubsystem,
      LEDSubsystem ledSubsystem,
      Color ledColor,
      Supplier<Pose2d> poseProvider,
      List<Pose2d> redPoses,
      List<Pose2d> bluePoses) {
    super(drivetrainSubsystem, ledSubsystem, ledColor, poseProvider);
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