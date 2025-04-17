package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.LEDPattern.solid;
import static edu.wpi.first.wpilibj.util.Color.*;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
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

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.DriveToNearestPose;
import frc.robot.commands.HoldCoralCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.ShootAlgaeCommand;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import java.util.List;
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
  private final GamePieceManipulatorSubsystem gamePieceSubsystem;
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
    this.gamePieceSubsystem = gamePieceManipulatorSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.highCameraForLeft = highCameraForLeft;
    this.highCameraForRight = highCameraForRight;
  }

  /**
   * Creates a command to intake coral. The command finishes when coral is in the arm.
   * 
   * @return new command
   */
  public Command intakeCoral() {
    return new IntakeCoralCommand(indexerSubsystem, gamePieceSubsystem, armSubsystem, ledSubsystem);
  }

  /**
   * Creates a command to hold coral. The command finishes when there is no coral in the arm.
   * 
   * @return new command
   */
  public Command holdCoral() {
    return new HoldCoralCommand(indexerSubsystem, gamePieceSubsystem, armSubsystem, ledSubsystem, () -> false);
  }

  /**
   * Creates a command to hold algae. The command has no end condition, so it must be interupted.
   * 
   * @return new command
   */
  public Command holdAlgae() {
    return run(() -> {
      armSubsystem.moveToHoldAlgae();
      gamePieceSubsystem.activeHoldAlgae();
    }, armSubsystem, gamePieceSubsystem).finallyDo(() -> {
      armSubsystem.stop();
      gamePieceSubsystem.stop();
    });
  }

  /**
   * Creates a command to hold coral and park. The parking position is controlled by the park for level 1 supplier.
   * 
   * @param parkForLevel1 when the supplier returns true, the arm will park at a position best for scoring L1, otherwise
   *          it will park in the standard park position
   * @return new command
   */
  public Command holdCoral(BooleanSupplier parkForLevel1) {
    return new HoldCoralCommand(indexerSubsystem, gamePieceSubsystem, armSubsystem, ledSubsystem, parkForLevel1);
  }

  /**
   * Shoots algae into the barge
   * 
   * @return new command
   */
  public Command shootAlgae() {
    return new ShootAlgaeCommand(armSubsystem, gamePieceSubsystem, ledSubsystem);
  }

  /**
   * Creates a command that will auto align and score on the left side of level 4
   * 
   * @param allowScoreWithoutTag true to allow scoring if the apriltag was never seen, otherwise false
   * @return new command
   */
  public Command scoreCoralLevel4Left(boolean allowScoreWithoutTag) {
    return autoScoreCoral(
        armSubsystem::moveToLevel4Align,
          armSubsystem::moveToLevel4Release,
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
        armSubsystem::moveToLevel4Align,
          armSubsystem::moveToLevel4Release,
          DISTANCE_TARGET_L4,
          LATERAL_TARGET_L4_RIGHT,
          highCameraForRight,
          allowScoreWithoutTag,
          kBlue);
  }

  /**
   * Creates a command that will auto align and score on the left side of level 3
   * 
   * @return new command
   */
  public Command scoreCoralLevel3Left() {
    return driveToReefAndAutoScoreCoral(
        armSubsystem::moveToLevel3Align,
          armSubsystem::moveToLevel3Release,
          REEF_L3_SCORE_POSES_RED_LEFT,
          REEF_L3_SCORE_POSES_BLUE_LEFT,
          DISTANCE_TARGET_L3,
          LATERAL_TARGET_L3_LEFT,
          highCameraForLeft,
          kOrange);
  }

  /**
   * Creates a command that will auto align and score on the right side of level 3
   * 
   * @return new command
   */
  public Command scoreCoralLevel3Right() {
    return driveToReefAndAutoScoreCoral(
        armSubsystem::moveToLevel3Align,
          armSubsystem::moveToLevel3Release,
          REEF_L3_SCORE_POSES_RED_RIGHT,
          REEF_L3_SCORE_POSES_BLUE_RIGHT,
          DISTANCE_TARGET_L3,
          LATERAL_TARGET_L3_RIGHT,
          highCameraForRight,
          kBlue);
  }

  /**
   * Creates a command that will auto align on the left side of level 4, and then runs <code>driveCommand</code>.
   * This command does not score nor end on its own.
   * 
   * @param driveCommand command to run after alignment, while holding the arm up
   * @return new command
   */
  public Command autoAlignCoralLevel4Left(Command driveCommand) {
    return driveToReefAndAlign(
        armSubsystem::moveToLevel4Align,
          armSubsystem::moveToLevel4Release,
          REEF_L4_SCORE_POSES_RED_LEFT,
          REEF_L4_SCORE_POSES_BLUE_LEFT,
          DISTANCE_TARGET_L4,
          LATERAL_TARGET_L4_LEFT,
          highCameraForLeft,
          kGreenYellow,
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
    return driveToReefAndAlign(
        armSubsystem::moveToLevel4Align,
          armSubsystem::moveToLevel4Release,
          REEF_L4_SCORE_POSES_RED_RIGHT,
          REEF_L4_SCORE_POSES_BLUE_RIGHT,
          DISTANCE_TARGET_L4,
          LATERAL_TARGET_L4_RIGHT,
          highCameraForRight,
          kPurple,
          kBlue,
          driveCommand);
  }

  /**
   * Plucks algae from the reef in the upper position, when approaching from the left. It aligns to the reef, then moves
   * the arm and drives forward and back
   * 
   * @return new command
   */
  public Command autoPluckAlgaeHighFromLeft() {
    return autoPluckAlgae(armSubsystem::moveToAlgaeLevel2, highCameraForLeft);
  }

  /**
   * Plucks algae from the reef in the upper position, when approaching from the right. It aligns to the reef, then
   * moves the arm and drives forward and back
   * 
   * @return new command
   */
  public Command autoPluckAlgaeHighFromRight() {
    return autoPluckAlgae(armSubsystem::moveToAlgaeLevel2, highCameraForRight);
  }

  /**
   * Plucks algae from the reef in the lower position, when approaching from the left. It aligns to the reef, then moves
   * the arm and drives forward and back
   * 
   * @return new command
   */
  public Command autoPluckAlgaeLowFromLeft() {
    return autoPluckAlgae(armSubsystem::moveToAlgaeLevel1, highCameraForLeft);
  }

  /**
   * Plucks algae from the reef in the lower position, when approaching from the right. It aligns to the reef, then
   * moves the arm and drives forward and back
   * 
   * @return new command
   */
  public Command autoPluckAlgaeLowFromRight() {
    return autoPluckAlgae(armSubsystem::moveToAlgaeLevel1, highCameraForRight);
  }

  private Command autoPluckAlgae(Runnable armMethod, PhotonCamera camera) {
    return sequence(
        armSubsystem.run(armSubsystem::park).until(armSubsystem::isAtPosition).withTimeout(0.8),
          alignToReef(Meters.of(.39), Meters.of(-0.13), camera, true),
          armSubsystem.run(armMethod).until(armSubsystem::isAtPosition).withTimeout(0.8),
          deadline(
              sequence(
                  drive(new ChassisSpeeds(0, .5, 0)).withTimeout(1.0),
                    waitSeconds(0.5),
                    drive(new ChassisSpeeds(0, -.5, 0)).withTimeout(0.5)),
                armSubsystem.run(armMethod),
                gamePieceSubsystem.run(gamePieceSubsystem::intakeAlgae)),
          holdAlgae().until(armSubsystem::isAtPosition))
        .finallyDo(() -> {
          armSubsystem.stop();
          gamePieceSubsystem.stop();
        });
  }

  private Command drive(ChassisSpeeds chassisSpeeds) {
    return drivetrain.applyRequest(() -> new SwerveRequest.ApplyRobotSpeeds().withSpeeds(chassisSpeeds))
        .finallyDo(() -> drivetrain.setControl(new SwerveRequest.Idle()));
  }

  private Command strobeLeds(Color ledColor) {
    return ledSubsystem.runPatternAsCommand(solid(ledColor).blink(Seconds.of(0.05)));
  }

  private Command driveToReef(Color ledColor, List<Pose2d> redPoses, List<Pose2d> bluePoses) {
    return new DriveToNearestPose(drivetrain, redPoses, bluePoses)
        .deadlineFor(ledSubsystem.runPatternAsCommand(solid(ledColor)))
        .finallyDo(ledSubsystem::off);
  }

  private AlignToReefCommand alignToReef(
      Distance targetDistance,
      Distance lateralTarget,
      PhotonCamera highCamera,
      boolean allowScoreWithoutTag) {
    return new AlignToReefCommand(
        drivetrain,
        alignmentSubsystem,
        targetDistance,
        lateralTarget,
        highCamera,
        allowScoreWithoutTag);
  }

  private Command ledAlignmentSegments(Color ledColor, AlignToReefCommand alignToReef) {
    return ledSubsystem.runPatternAsCommand(
        ledSegments(
            ledColor,
              // segment order is bottom up
              armSubsystem::isElevatorAtPosition,
              armSubsystem::isArmAtAngle,
              alignToReef::atDistanceGoal,
              alignToReef::atLateralGoal,
              alignToReef::atThetaGoal))
        .finallyDo(ledSubsystem::off);
  }

  private Command driveToReefAndAlign(
      Runnable armMethod,
      Runnable armMethodRelease,
      List<Pose2d> redPoses,
      List<Pose2d> bluePoses,
      Distance targetDistance,
      Distance lateralTarget,
      PhotonCamera highCamera,
      Color driveLedColor,
      Color alignLedColor,
      Command driveCommand) {

    var driveToReef = driveToReef(driveLedColor, redPoses, bluePoses);
    var alignToReef = alignToReef(targetDistance, lateralTarget, highCamera, true);
    var alignLedCommand = ledAlignmentSegments(alignLedColor, alignToReef);

    return armSubsystem.run(armMethod)
        .withDeadline(driveToReef)
        .andThen(
            alignLedCommand.withDeadline(
                parallel(armSubsystem.run(armMethod).until(armSubsystem::isAtPosition), alignToReef).withTimeout(3.0))
                .andThen(parallel(strobeLeds(kGreen), armSubsystem.run(armMethodRelease), driveCommand)))
        .finallyDo(armSubsystem::stop);
  }

  private Command autoScoreCoral(
      Runnable armMethod,
      Runnable armMethodRelease,
      Distance targetDistance,
      Distance lateralTarget,
      PhotonCamera highCamera,
      boolean allowScoreWithoutTag,
      Color ledColor) {

    var alignToReef = alignToReef(targetDistance, lateralTarget, highCamera, allowScoreWithoutTag);
    var ledCommand = ledAlignmentSegments(ledColor, alignToReef);

    return ledCommand
        .withDeadline(
            gamePieceSubsystem.run(gamePieceSubsystem::activeHoldCoral)
                .withDeadline(
                    parallel(armSubsystem.run(armMethod).until(armSubsystem::isAtPosition), alignToReef)
                        .withTimeout(3.0)
                        .andThen(armSubsystem.run(armMethodRelease).withTimeout(0.2)))
                .andThen(
                    parallel(armSubsystem.run(armMethodRelease), gamePieceSubsystem.run(gamePieceSubsystem::ejectCoral))
                        .until(() -> !armSubsystem.hasCoral())))
        .finallyDo(armSubsystem::stop)
        .finallyDo(gamePieceSubsystem::stop);
  }

  private Command driveToReefAndAutoScoreCoral(
      Runnable armMethod,
      Runnable armMethodRelease,
      List<Pose2d> redPoses,
      List<Pose2d> bluePoses,
      Distance targetDistance,
      Distance lateralTarget,
      PhotonCamera highCamera,
      Color ledColor) {

    var driveToReef = driveToReef(ledColor, redPoses, bluePoses);
    var autoScoreCommand = autoScoreCoral(
        armMethod,
          armMethodRelease,
          targetDistance,
          lateralTarget,
          highCamera,
          false,
          ledColor);

    return parallel(gamePieceSubsystem.run(gamePieceSubsystem::activeHoldCoral), armSubsystem.run(armMethod))
        .withDeadline(driveToReef)
        .andThen(autoScoreCommand);
  }

}