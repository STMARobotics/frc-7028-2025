package frc.robot;

import static edu.wpi.first.wpilibj.util.Color.*;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.Constants.AlignmentConstants.DISTANCE_TARGET_L3;
import static frc.robot.Constants.AlignmentConstants.DISTANCE_TARGET_L4;
import static frc.robot.Constants.AlignmentConstants.LATERAL_TARGET_L3_LEFT;
import static frc.robot.Constants.AlignmentConstants.LATERAL_TARGET_L3_RIGHT;
import static frc.robot.Constants.AlignmentConstants.LATERAL_TARGET_L4_LEFT;
import static frc.robot.Constants.AlignmentConstants.LATERAL_TARGET_L4_RIGHT;
import static frc.robot.Constants.AlignmentConstants.REEF_L3_SCORE_POSES_BLUE_LEFT;
import static frc.robot.Constants.AlignmentConstants.REEF_L3_SCORE_POSES_BLUE_RIGHT;
import static frc.robot.Constants.AlignmentConstants.REEF_L3_SCORE_POSES_RED_LEFT;
import static frc.robot.Constants.AlignmentConstants.REEF_L3_SCORE_POSES_RED_RIGHT;
import static frc.robot.Constants.AlignmentConstants.REEF_L4_SCORE_POSES_BLUE_LEFT;
import static frc.robot.Constants.AlignmentConstants.REEF_L4_SCORE_POSES_BLUE_RIGHT;
import static frc.robot.Constants.AlignmentConstants.REEF_L4_SCORE_POSES_RED_LEFT;
import static frc.robot.Constants.AlignmentConstants.REEF_L4_SCORE_POSES_RED_RIGHT;
import static frc.robot.subsystems.LEDSubsystem.ledSegments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.HoldCoralCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import java.util.function.BooleanSupplier;
import org.photonvision.PhotonCamera;

/**
 * Library of commands to perform complex actions
 */
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
  }

  /**
   * Creates a command to intake coral. The command finishes when coral is in the arm.
   * 
   * @return new command
   */
  public Command intakeCoral() {
    return new IntakeCoralCommand(indexerSubsystem, gamePieceManipulatorSubsystem, armSubsystem, ledSubsystem);
  }

  /**
   * Creates a command to hold coral. The command finishes when there is no coral in the arm.
   * 
   * @return new command
   */
  public Command holdCoral() {
    return new HoldCoralCommand(
        indexerSubsystem,
        gamePieceManipulatorSubsystem,
        armSubsystem,
        ledSubsystem,
        () -> false);
  }

  /**
   * Creates a command to hold coral and park. The parking position is controlled by the park for level 1 supplier.
   * 
   * @param parkForLevel1 when the supplier returns true, the arm will park at a position best for scoring L1, otherwise
   *          it will park in the standard park position
   * @return new command
   */
  public Command holdCoral(BooleanSupplier parkForLevel1) {
    return new HoldCoralCommand(
        indexerSubsystem,
        gamePieceManipulatorSubsystem,
        armSubsystem,
        ledSubsystem,
        parkForLevel1);
  }

  /**
   * Creates a command that will auto align and score on the left side of level 4
   * 
   * @param allowScoreWithoutTag true to allow scoring if the apriltag was never seen, otherwise false
   * @return new command
   */
  public Command scoreCoralLevel4Left(boolean allowScoreWithoutTag) {
    return autoScoreCoral(
        armSubsystem::moveToLevel4,
          DISTANCE_TARGET_L4,
          LATERAL_TARGET_L4_LEFT,
          highCameraForLeft,
          allowScoreWithoutTag,
          kOrange);
  }

  /**
   * Creates a command that will auto align and score on the right side of level 4
   * 
   * @param allowScoreWithoutTag true to allow scoring if the apriltag was never seen, otherwise false
   * @return new command
   */
  public Command scoreCoralLevel4Right(boolean allowScoreWithoutTag) {
    return autoScoreCoral(
        armSubsystem::moveToLevel4,
          DISTANCE_TARGET_L4,
          LATERAL_TARGET_L4_RIGHT,
          highCameraForRight,
          allowScoreWithoutTag,
          kBlue);
  }

  /**
   * Creates a command that will auto align and score on the left side of level 3
   * 
   * @param allowScoreWithoutTag true to allow scoring if the apriltag was never seen, otherwise false
   * @return new command
   */
  public Command scoreCoralLevel3Left(boolean allowScoreWithoutTag) {
    return autoScoreCoral(
        armSubsystem::moveToLevel3,
          DISTANCE_TARGET_L3,
          LATERAL_TARGET_L3_LEFT,
          highCameraForLeft,
          allowScoreWithoutTag,
          kOrange);
  }

  /**
   * Creates a command that will auto align and score on the right side of level 3
   * 
   * @param allowScoreWithoutTag true to allow scoring if the apriltag was never seen, otherwise false
   * @return new command
   */
  public Command scoreCoralLevel3Right(boolean allowScoreWithoutTag) {
    return autoScoreCoral(
        armSubsystem::moveToLevel3,
          DISTANCE_TARGET_L3,
          LATERAL_TARGET_L3_RIGHT,
          highCameraForRight,
          allowScoreWithoutTag,
          kBlue);
  }

  private Command autoScoreCoral(
      Runnable armMethod,
      Distance targetDistance,
      Distance lateralTarget,
      PhotonCamera highCamera,
      boolean allowScoreWithoutTag,
      Color ledColor) {
    var alignToReef = new AlignToReefCommand(
        drivetrain,
        alignmentSubsystem,
        targetDistance,
        lateralTarget,
        highCamera,
        allowScoreWithoutTag);

    return ledSubsystem.runPatternAsCommand(
        ledSegments(
            ledColor,
              // segment order is bottom up
              armSubsystem::isElevatorAtPosition,
              armSubsystem::isArmAtAngle,
              alignToReef::atDistanceGoal,
              alignToReef::atLateralGoal,
              alignToReef::atThetaGoal))
        .withDeadline(
            armSubsystem.run(armMethod)
                .alongWith(gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::activeHoldCoral))
                .until(armSubsystem::isAtPosition)
                .alongWith(alignToReef)
                .andThen(waitSeconds(0.2))
                .andThen(
                    armSubsystem.run(armMethod)
                        .alongWith(gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::ejectCoral))
                        .alongWith(indexerSubsystem.run(indexerSubsystem::eject))
                        .until(() -> !armSubsystem.hasCoral())))
        .finallyDo(() -> ledSubsystem.runPattern(LEDPattern.kOff))
        .finallyDo(armSubsystem::stop);
  }

  /**
   * Creates a command that will auto align on the left side of level 4, and then runs <code>driveCommand</code>.
   * This command does not score nor end on its own.
   * 
   * @param driveCommand command to run after alignment, while holding the arm up
   * @return new command
   */
  public Command autoAlignCoralLevel4Left(Command driveCommand) {
    return autoAlignCoral(
        armSubsystem::moveToLevel4,
          DISTANCE_TARGET_L4,
          LATERAL_TARGET_L4_LEFT,
          highCameraForLeft,
          kOrange,
          driveCommand);
  }

  /**
   * Creates a command that will auto align on the right side of level 4, and then runs <code>driveCommand</code>. This
   * command does not score nor end on its own.
   * 
   * @param driveCommand command to run after alignment, while holding the arm up
   * @return new command
   */
  public Command autoAlignCoralLevel4Right(Command driveCommand) {
    return autoAlignCoral(
        armSubsystem::moveToLevel4,
          DISTANCE_TARGET_L4,
          LATERAL_TARGET_L4_RIGHT,
          highCameraForRight,
          kBlue,
          driveCommand);
  }

  private Command autoAlignCoral(
      Runnable armMethod,
      Distance targetDistance,
      Distance lateralTarget,
      PhotonCamera highCamera,
      Color ledColor,
      Command driveCommand) {
    var alignToReef = new AlignToReefCommand(
        drivetrain,
        alignmentSubsystem,
        targetDistance,
        lateralTarget,
        highCamera,
        true);

    return ledSubsystem.runPatternAsCommand(
        ledSegments(
            ledColor,
              // segment order is bottom up
              armSubsystem::isElevatorAtPosition,
              armSubsystem::isArmAtAngle,
              alignToReef::atDistanceGoal,
              alignToReef::atLateralGoal,
              alignToReef::atThetaGoal))
        .withDeadline(
            parallel(armSubsystem.run(armMethod).until(armSubsystem::isAtPosition), alignToReef)
                .andThen(parallel(driveCommand, armSubsystem.run(armMethod))))
        .finallyDo(() -> ledSubsystem.runPattern(LEDPattern.kOff))
        .finallyDo(armSubsystem::stop);
  }
}