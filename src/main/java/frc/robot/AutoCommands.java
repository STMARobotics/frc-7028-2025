package frc.robot;

import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static frc.robot.Constants.AlignmentConstants.DISTANCE_TARGET_L3;
import static frc.robot.Constants.AlignmentConstants.DISTANCE_TARGET_L4;
import static frc.robot.Constants.AlignmentConstants.LATERAL_TARGET_L3_LEFT;
import static frc.robot.Constants.AlignmentConstants.LATERAL_TARGET_L3_RIGHT;
import static frc.robot.Constants.AlignmentConstants.LATERAL_TARGET_L4_LEFT;
import static frc.robot.Constants.AlignmentConstants.LATERAL_TARGET_L4_RIGHT;
import static frc.robot.Constants.AlignmentConstants.REEF_L1_SCORE_POSES_BLUE;
import static frc.robot.Constants.AlignmentConstants.REEF_L1_SCORE_POSES_RED;
import static frc.robot.Constants.AlignmentConstants.REEF_L2_SCORE_POSES_BLUE_LEFT;
import static frc.robot.Constants.AlignmentConstants.REEF_L2_SCORE_POSES_BLUE_RIGHT;
import static frc.robot.Constants.AlignmentConstants.REEF_L2_SCORE_POSES_RED_LEFT;
import static frc.robot.Constants.AlignmentConstants.REEF_L2_SCORE_POSES_RED_RIGHT;
import static frc.robot.Constants.AlignmentConstants.REEF_L3_SCORE_POSES_BLUE_LEFT;
import static frc.robot.Constants.AlignmentConstants.REEF_L3_SCORE_POSES_BLUE_RIGHT;
import static frc.robot.Constants.AlignmentConstants.REEF_L3_SCORE_POSES_RED_LEFT;
import static frc.robot.Constants.AlignmentConstants.REEF_L3_SCORE_POSES_RED_RIGHT;
import static frc.robot.Constants.AlignmentConstants.REEF_L4_SCORE_POSES_BLUE_LEFT;
import static frc.robot.Constants.AlignmentConstants.REEF_L4_SCORE_POSES_BLUE_RIGHT;
import static frc.robot.Constants.AlignmentConstants.REEF_L4_SCORE_POSES_RED_LEFT;
import static frc.robot.Constants.AlignmentConstants.REEF_L4_SCORE_POSES_RED_RIGHT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
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
@SuppressWarnings("unused")
public class AutoCommands {

  private final CommandSwerveDrivetrain drivetrain;
  private final ArmSubsystem armSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final AlignmentSubsystem alignmentSubsystem;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final PhotonCamera highCameraForLeft;
  private final PhotonCamera highCameraForRight;

  /**
   * Constructs a new AutoCommands object
   * 
   * @param drivetrain drivetrain subsystem
   * @param armSubsystem arm subsystem
   * @param indexerSubsystem indexer subsystem
   * @param alignmentSubsystem alinment subsystem
   * @param gamePieceManipulatorSubsystem game piece manipulator subsystem
   * @param indexerSubsystem indexer subsystem
   * @param ledSubsystem led subsystem
   * @param highCameraForLeft high camera for scoring on the left branch
   * @param highCameraForRight high camera for scoring on the right branch
   */
  public AutoCommands(
      CommandSwerveDrivetrain drivetrain,
      ArmSubsystem armSubsystem,
      IndexerSubsystem indexerSubsysteml,
      AlignmentSubsystem alignmentSubsystem,
      GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem,
      IndexerSubsystem indexerSubsystem,
      LEDSubsystem ledSubsystem,
      PhotonCamera highCameraForLeft,
      PhotonCamera highCameraForRight) {
    this.drivetrain = drivetrain;
    this.armSubsystem = armSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.alignmentSubsystem = alignmentSubsystem;
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.highCameraForLeft = highCameraForLeft;
    this.highCameraForRight = highCameraForRight;

    publishScoringPoses();
  }

  /** Sends the reef poses to the dashboard for debugging */
  private void publishScoringPoses() {
    var table = NetworkTableInstance.getDefault().getTable("scoring_poses");

    // L4 Blue
    table.getStructArrayTopic("reef_l4_blue_left", Pose2d.struct)
        .publish()
        .set(REEF_L4_SCORE_POSES_BLUE_LEFT.toArray(Pose2d[]::new));

    table.getStructArrayTopic("reef_l4_blue_right", Pose2d.struct)
        .publish()
        .set(REEF_L4_SCORE_POSES_BLUE_RIGHT.toArray(Pose2d[]::new));

    // L4 Red
    table.getStructArrayTopic("reef_l4_red_left", Pose2d.struct)
        .publish()
        .set(REEF_L4_SCORE_POSES_RED_LEFT.toArray(Pose2d[]::new));

    table.getStructArrayTopic("reef_l4_red_right", Pose2d.struct)
        .publish()
        .set(REEF_L4_SCORE_POSES_RED_RIGHT.toArray(Pose2d[]::new));

    // L3 Blue
    table.getStructArrayTopic("reef_l3_blue_left", Pose2d.struct)
        .publish()
        .set(REEF_L3_SCORE_POSES_BLUE_LEFT.toArray(Pose2d[]::new));

    table.getStructArrayTopic("reef_l3_blue_right", Pose2d.struct)
        .publish()
        .set(REEF_L3_SCORE_POSES_BLUE_RIGHT.toArray(Pose2d[]::new));

    // L3 Red
    table.getStructArrayTopic("reef_l3_red_left", Pose2d.struct)
        .publish()
        .set(REEF_L3_SCORE_POSES_RED_LEFT.toArray(Pose2d[]::new));

    table.getStructArrayTopic("reef_l3_red_right", Pose2d.struct)
        .publish()
        .set(REEF_L3_SCORE_POSES_RED_RIGHT.toArray(Pose2d[]::new));

    // L2 Blue
    table.getStructArrayTopic("reef_l2_blue_left", Pose2d.struct)
        .publish()
        .set(REEF_L2_SCORE_POSES_BLUE_LEFT.toArray(Pose2d[]::new));

    table.getStructArrayTopic("reef_l2_blue_right", Pose2d.struct)
        .publish()
        .set(REEF_L2_SCORE_POSES_BLUE_RIGHT.toArray(Pose2d[]::new));

    // L2 Red
    table.getStructArrayTopic("reef_l2_red_left", Pose2d.struct)
        .publish()
        .set(REEF_L2_SCORE_POSES_RED_LEFT.toArray(Pose2d[]::new));

    table.getStructArrayTopic("reef_l2_red_right", Pose2d.struct)
        .publish()
        .set(REEF_L2_SCORE_POSES_RED_RIGHT.toArray(Pose2d[]::new));

    // L1 Blue
    table.getStructArrayTopic("reef_l1_blue", Pose2d.struct)
        .publish()
        .set(REEF_L1_SCORE_POSES_BLUE.toArray(Pose2d[]::new));

    // L1 Red
    table.getStructArrayTopic("reef_l1_red", Pose2d.struct)
        .publish()
        .set(REEF_L1_SCORE_POSES_RED.toArray(Pose2d[]::new));
  }

  /**
   * Creates a command that will auto align and score on the left side of level 4
   * 
   * @return new command
   */
  public Command scoreCoralLevel4Left() {
    return scoreCoralTeleop(
        armSubsystem::moveToLevel4,
          REEF_L4_SCORE_POSES_RED_LEFT,
          REEF_L4_SCORE_POSES_BLUE_LEFT,
          DISTANCE_TARGET_L4,
          LATERAL_TARGET_L4_LEFT,
          highCameraForLeft);
  }

  /**
   * Creates a command that will auto align and score on the right side of level 4
   * 
   * @return new command
   */
  public Command scoreCoralLevel4Right() {
    return scoreCoralTeleop(
        armSubsystem::moveToLevel4,
          REEF_L4_SCORE_POSES_RED_RIGHT,
          REEF_L4_SCORE_POSES_BLUE_RIGHT,
          DISTANCE_TARGET_L4,
          LATERAL_TARGET_L4_RIGHT,
          highCameraForRight);
  }

  /**
   * Creates a command that will auto align and score on the left side of level 3
   * 
   * @return new command
   */
  public Command scoreCoralLevel3Left() {
    return scoreCoralTeleop(
        armSubsystem::moveToLevel3,
          REEF_L3_SCORE_POSES_RED_LEFT,
          REEF_L3_SCORE_POSES_BLUE_LEFT,
          DISTANCE_TARGET_L3,
          LATERAL_TARGET_L3_LEFT,
          highCameraForLeft);
  }

  /**
   * Creates a command that will auto align and score on the right side of level 3
   * 
   * @return new command
   */
  public Command scoreCoralLevel3Right() {
    return scoreCoralTeleop(
        armSubsystem::moveToLevel3,
          REEF_L3_SCORE_POSES_RED_RIGHT,
          REEF_L3_SCORE_POSES_BLUE_RIGHT,
          DISTANCE_TARGET_L3,
          LATERAL_TARGET_L3_RIGHT,
          highCameraForRight);
  }

  /**
   * Creates a command that will drive to the nearest L2 left reef scoring location
   * 
   * @return new command
   */
  public Command driveToCoralLevel2Left() {
    return new DriveToNearestPose(
        drivetrain,
        () -> drivetrain.getState().Pose,
        REEF_L2_SCORE_POSES_RED_LEFT,
        REEF_L2_SCORE_POSES_BLUE_LEFT);
  }

  /**
   * Creates a command that will drive to the nearest L2 right reef scoring location
   * 
   * @return new command
   */
  public Command driveToCoralLevel2Right() {
    return new DriveToNearestPose(
        drivetrain,
        () -> drivetrain.getState().Pose,
        REEF_L2_SCORE_POSES_RED_RIGHT,
        REEF_L2_SCORE_POSES_BLUE_RIGHT);
  }

  /**
   * Creates a command that will drive to the nearest L1 reef scoring location
   * 
   * @return new command
   */
  public Command driveToCoralLevel1() {
    return new DriveToNearestPose(
        drivetrain,
        () -> drivetrain.getState().Pose,
        REEF_L1_SCORE_POSES_RED,
        REEF_L1_SCORE_POSES_BLUE);
  }

  private Command scoreCoralTeleop(
      Runnable armMethod,
      List<Pose2d> redPoses,
      List<Pose2d> bluePoses,
      Distance distanceTarget,
      Distance lateralTarget,
      PhotonCamera highCamera) {
    var driveToReef = new DriveToNearestPose(drivetrain, () -> drivetrain.getState().Pose, redPoses, bluePoses);
    var alignToReef = new AlignToReefCommand(
        drivetrain,
        alignmentSubsystem,
        distanceTarget,
        lateralTarget,
        highCamera,
        false);

    return ledSubsystem
        .setLEDSegmentsAsCommand(
            kGreen,
              driveToReef::isFinished,
              armSubsystem::isAtPosition,
              alignToReef::atDistanceGoal,
              alignToReef::atLateralGoal)
        .withDeadline(
            driveToReef.andThen(
                armSubsystem.run(armMethod)
                    .until(armSubsystem::isAtPosition)
                    .alongWith(alignToReef)
                    .andThen(
                        armSubsystem.run(armMethod)
                            .alongWith(gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::ejectCoral))
                            .withTimeout(0.25))));
  }

  /**
   * Creates a command to score coral on left level 4 using the sequence for autonomous
   * 
   * @return new command
   */
  public Command scoreCoralLevel4AutoLeft() {
    return autoScoreCoralAuto(
        armSubsystem::moveToLevel4,
          REEF_L4_SCORE_POSES_RED_LEFT,
          REEF_L4_SCORE_POSES_BLUE_LEFT,
          LATERAL_TARGET_L4_LEFT,
          highCameraForLeft);
  }

  private Command autoScoreCoralAuto(
      Runnable armMethod,
      List<Pose2d> redPoses,
      List<Pose2d> bluePoses,
      Distance lateralTarget,
      PhotonCamera highCamera) {
    var alignToReef = new AlignToReefCommand(
        drivetrain,
        alignmentSubsystem,
        DISTANCE_TARGET_L4,
        lateralTarget,
        highCamera,
        true);

    return ledSubsystem
        .setLEDSegmentsAsCommand(
            kGreen,
              armSubsystem::isAtPosition,
              alignToReef::atDistanceGoal,
              alignToReef::atLateralGoal)
        .withDeadline(
            armSubsystem.run(armMethod)
                .until(armSubsystem::isAtPosition)
                .alongWith(alignToReef)
                .andThen(
                    armSubsystem.run(armMethod)
                        .alongWith(gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::ejectCoral))
                        .withTimeout(0.25)))
        .finallyDo(() -> ledSubsystem.runPattern(LEDPattern.kOff));
  }
}