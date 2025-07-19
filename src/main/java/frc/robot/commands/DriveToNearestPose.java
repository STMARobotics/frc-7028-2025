package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import java.util.List;

/**
 * Command that uses PathPlanner path finding to drive to the nearest pose from a list.
 */
public class DriveToNearestPose extends Command {

  private final List<Pose2d> redPoses;
  private final List<Pose2d> bluePoses;
  private final CommandSwerveDrivetrain drivetrain;
  private final LEDSubsystem ledSubsystem;

  private Command pathCommand;

  /**
   * Constructs a DriveToNearestPose
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param redPoses list of poses for when the robot is on the red alliance
   * @param bluePoses list of poses for when the robot is on the blue alliance
   */
  public DriveToNearestPose(
      CommandSwerveDrivetrain drivetrainSubsystem,
      LEDSubsystem ledSubsystem,
      List<Pose2d> redPoses,
      List<Pose2d> bluePoses) {

    this.drivetrain = drivetrainSubsystem;
    this.redPoses = redPoses;
    this.bluePoses = bluePoses;
    this.ledSubsystem = ledSubsystem;

    addRequirements(drivetrainSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    var robotPose = drivetrain.getState().Pose;
    var isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    Pose2d targetPose = robotPose.nearest(isRed ? redPoses : bluePoses);
    pathCommand = new DriveToPoseCommand(
        drivetrain,
        ledSubsystem,
        Color.kGreen,
        () -> drivetrain.getState().Pose,
        targetPose,
        targetPose.getRotation().rotateBy(Rotation2d.kCCW_90deg));
    pathCommand.initialize();
  }

  @Override
  public void execute() {
    pathCommand.execute();
  }

  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
    drivetrain.setControl(new SwerveRequest.Idle());
  }

}