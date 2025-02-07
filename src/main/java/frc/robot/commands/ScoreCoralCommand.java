package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import java.util.function.Supplier;

/**
 * A GamePieceManipulatorCommands uses the GamepieceMainpulatorSubsytems to score the coral.
 */
public class ScoreCoralCommand extends Command {

  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final CommandSwerveDrivetrain swerveDrivetrain;

  private final Supplier<Pose2d> poseSupplier;
  private final int levelToScore;

  /**
   * Construcnter define subsytems, Pose Estimator, and all varibles needed.
   * 
   * @param manipulator
   * @param arm
   * @param drive
   * @param poseSupplier
   * @param robotAngleSupplier
   * @param chassSupplier
   * @param levelToScore
   */
  public ScoreCoralCommand(
      GamePieceManipulatorSubsystem manipulator,
      ArmSubsystem arm,
      CommandSwerveDrivetrain drive,
      Supplier<Pose2d> poseSupplier,
      int levelToScore) {
    this.gamePieceManipulatorSubsystem = manipulator;
    this.armSubsystem = arm;
    this.levelToScore = levelToScore;
    this.swerveDrivetrain = drive;
    addRequirements(swerveDrivetrain, armSubsystem, gamePieceManipulatorSubsystem);

    this.poseSupplier = poseSupplier;

  }

  /**
   * Allows the robot to decide what elave it should go to.
   */
  @Override
  public void initialize() {
    // Will replace with better drive to pose.
    // var robotPose = poseSupplier.get();
    // final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    // var fieldSpeeds = new Translation2d().rotateBy(robotPose.getRotation());
    switch (levelToScore) {
      // The case the numbers refere to leaves of the reef with the trench being 0.
      case 1:
        armSubsystem.moveElevatorLevel2();
        armSubsystem.moveArmToLevel2();
        break;
      case 2:
        armSubsystem.moveElevatorLevel3();
        armSubsystem.moveArmToLevel3();
        break;
      case 3:
        armSubsystem.moveElevatorLevel4();
        armSubsystem.moveArmToLevel4();
        break;
      default:
        break;
    }
    // The Number are way oof and X and Y are probaly near 0 and were more worried about rotanioanl rate.
    // swerveDrivetrain.setControl(drive.withVelocityX(50).withVelocityY(50).withRotationalRate(50));
  }

  /**
   * Tells the robot that if its at the right postion to keep ging if not then hold game pieace.
   */
  @Override
  public void execute() {
    var isElevatorAtPosition = armSubsystem.isElevatorAtPosition();
    var isArmAtPosition = armSubsystem.isArmAtPosition();
    if (isElevatorAtPosition && isArmAtPosition) {
      armSubsystem.stopArm();
      armSubsystem.stopElevator();
      gamePieceManipulatorSubsystem.scoreCoral();
    } else {
      gamePieceManipulatorSubsystem.activeHoldGamePiece();
    }
  }

  public boolean isFinished() {
    return !gamePieceManipulatorSubsystem.isCoralInPickupPosition();
  }

  /**
   * Stops and resets the command.
   */
  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveElevatorToDefault();
    armSubsystem.moveArmToIntake();
    gamePieceManipulatorSubsystem.stop();
    armSubsystem.stopArm();
    armSubsystem.stopElevator();
  }
}
