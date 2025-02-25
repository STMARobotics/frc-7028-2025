package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static frc.robot.Constants.AlignmentConstants.REEF_L2_SCORE_POSES_RED;
import static frc.robot.Constants.AlignmentConstants.REEF_L2_SCORE_POSE_BLUE;
import static frc.robot.Constants.AlignmentConstants.REEF_L3_SCORE_POSES_RED;
import static frc.robot.Constants.AlignmentConstants.REEF_L3_SCORE_POSE_BLUE;
import static frc.robot.Constants.AlignmentConstants.REEF_L4_SCORE_POSES_RED;
import static frc.robot.Constants.AlignmentConstants.REEF_L4_SCORE_POSE_BLUE;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.DriveToNearestPose;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import java.util.List;
import org.photonvision.PhotonCamera;

/**
 * Library of commands to perform complex actions
 */
public class AutoCommands {

  private final CommandSwerveDrivetrain drivetrain;
  private final ArmSubsystem armSubsystem;
  private final AlignmentSubsystem alignmentSubsystem;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final PhotonCamera highCamera;

  /**
   * Constructs a new AutoCommands object
   * 
   * @param drivetrain drivetrain subsystem
   * @param armSubsystem arm subsystem
   * @param alignmentSubsystem alinment subsystem
   * @param gamePieceManipulatorSubsystem game piece manipulator subsystem
   * @param indexerSubsystem indexer subsystem
   */
  public AutoCommands(
      CommandSwerveDrivetrain drivetrain,
      ArmSubsystem armSubsystem,
      AlignmentSubsystem alignmentSubsystem,
      GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem,
      IndexerSubsystem indexerSubsystem,
      LEDSubsystem ledSubsystem,
      PhotonCamera highCamera) {
    this.drivetrain = drivetrain;
    this.armSubsystem = armSubsystem;
    this.alignmentSubsystem = alignmentSubsystem;
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.highCamera = highCamera;

    publishScoringPoses();
  }

  /** Sends the reef poses to the dashboard for debugging */
  private void publishScoringPoses() {
    NetworkTableInstance.getDefault()
        .getTable("reef_blue_L4")
        .getStructArrayTopic("branches", Pose2d.struct)
        .publish()
        .set(REEF_L4_SCORE_POSE_BLUE.toArray(new Pose2d[REEF_L4_SCORE_POSE_BLUE.size()]));

    NetworkTableInstance.getDefault()
        .getTable("reef_red_L4")
        .getStructArrayTopic("branches", Pose2d.struct)
        .publish()
        .set(REEF_L4_SCORE_POSES_RED.toArray(new Pose2d[REEF_L4_SCORE_POSES_RED.size()]));

    NetworkTableInstance.getDefault()
        .getTable("reef_blue_L3")
        .getStructArrayTopic("branches", Pose2d.struct)
        .publish()
        .set(REEF_L3_SCORE_POSE_BLUE.toArray(new Pose2d[REEF_L3_SCORE_POSE_BLUE.size()]));

    NetworkTableInstance.getDefault()
        .getTable("reef_red_L3")
        .getStructArrayTopic("branches", Pose2d.struct)
        .publish()
        .set(REEF_L3_SCORE_POSES_RED.toArray(new Pose2d[REEF_L3_SCORE_POSES_RED.size()]));

    NetworkTableInstance.getDefault()
        .getTable("reef_blue_L2")
        .getStructArrayTopic("branches", Pose2d.struct)
        .publish()
        .set(REEF_L2_SCORE_POSE_BLUE.toArray(new Pose2d[REEF_L2_SCORE_POSE_BLUE.size()]));

    NetworkTableInstance.getDefault()
        .getTable("reef_red_L2")
        .getStructArrayTopic("branches", Pose2d.struct)
        .publish()
        .set(REEF_L2_SCORE_POSES_RED.toArray(new Pose2d[REEF_L2_SCORE_POSES_RED.size()]));
  }

  /**
   * Creates a command that will:
   * <ol>
   * <li>Drive to the nearest L4 reef scoring location</li>
   * <li>Align to the reef using the CANranges</li>
   * <li>Get the arm and elevator into position to score on level 4</li>
   * <li>Eject the coral</li>
   * </ol>
   * 
   * @return new command
   */
  public Command autoScoreCoralLevel4() {
    return autoScoreCoral(armSubsystem::moveToLevel4, REEF_L4_SCORE_POSES_RED, REEF_L4_SCORE_POSE_BLUE);
  }

  /**
   * Creates a command that will:
   * <ol>
   * <li>Drive to the nearest L3 reef scoring location</li>
   * <li>Align to the reef using the CANranges</li>
   * <li>Get the arm and elevator into position to score on level 3</li>
   * <li>Eject the coral</li>
   * </ol>
   * 
   * @return new command
   */
  public Command autoScoreCoralLevel3() {
    return autoScoreCoral(armSubsystem::moveToLevel3, REEF_L3_SCORE_POSES_RED, REEF_L3_SCORE_POSE_BLUE);
  }

  /**
   * Creates a command that will drive to the nearest L2 reef scoring location
   * 
   * @return new command
   */
  public Command driveToCoralLevel2() {
    return new DriveToNearestPose(
        drivetrain,
        () -> drivetrain.getState().Pose,
        REEF_L2_SCORE_POSES_RED,
        REEF_L2_SCORE_POSE_BLUE);
  }

  private Command autoScoreCoral(Runnable armMethod, List<Pose2d> redPoses, List<Pose2d> bluePoses) {
    var driveToReef = new DriveToNearestPose(drivetrain, () -> drivetrain.getState().Pose, redPoses, bluePoses);
    var alignToReef = new AlignToReefCommand(drivetrain, alignmentSubsystem, Meters.of(0.34), highCamera);

    return ledSubsystem
        .setLEDSegmentsAsCommand(kGreen, armSubsystem::isAtPosition, driveToReef::isFinished, alignToReef::isFinished)
        .withDeadline(
            driveToReef.andThen(drivetrain.runOnce(() -> drivetrain.setControl(new SwerveRequest.SwerveDriveBrake())))
                .andThen(
                    armSubsystem.run(armMethod)
                        .until(armSubsystem::isAtPosition)
                        .alongWith(alignToReef)
                        .andThen(
                            armSubsystem.run(armMethod)
                                .alongWith(gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::ejectCoral))
                                .withTimeout(1.0))));
  }

}
