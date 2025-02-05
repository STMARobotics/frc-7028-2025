package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import java.util.function.Supplier;

/**
 * A GamePieceManipulatorCommands uses the GamepieceMainpulatorSubsytems to score the coral.
 */
public class ScoreCoralCommand extends Command {
  // The Subsystem the command runs on.
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final CommandSwerveDrivetrain swerveDrivetrain;

  private final Supplier<Rotation2d> robotAngleSupplier;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassSupplier;
  private final int levelToScore;

  /**
   * The construcnter of the command and tell the perimeteres.
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
      Supplier<Rotation2d> robotAngleSupplier,
      Supplier<ChassisSpeeds> chassSupplier,
      int levelToScore) {
    this.gamePieceManipulatorSubsystem = manipulator;
    this.armSubsystem = arm;
    this.levelToScore = levelToScore;
    this.swerveDrivetrain = drive;
    addRequirements(swerveDrivetrain, armSubsystem, gamePieceManipulatorSubsystem);

    this.poseSupplier = poseSupplier;
    this.robotAngleSupplier = robotAngleSupplier;
    this.chassSupplier = chassSupplier;
  }

  /**
   * Allows the robot to decide what elave it should go to.
   */
  @Override
  public void initialize() {
    var robotPose = poseSupplier.get();
    var robotAngle = robotAngleSupplier.get();
    final SwerveRequest.FieldCentric _drive = new SwerveRequest.FieldCentric();
    var fieldSpeeds = new Translation2d().rotateBy(robotAngle);
    switch (levelToScore) {
      case 1:
        armSubsystem.moveElevatorLevel1();
        armSubsystem.moveArmToIntake();
        break;
      case 2:
        armSubsystem.moveElevatorLevel2();
        armSubsystem.moveArmToLevel2();
        break;
      case 3:
        armSubsystem.moveElevatorLevel3();
        armSubsystem.moveArmToLevel3();
        break;
      case 4:
        armSubsystem.moveElevatorLevel4();
        armSubsystem.moveArmToLevel4();
        break;
      default:
        break;
    }
    // The Number are way oof and X and Y are probaly near 0 and were more worried about rotanioanl rate.
    swerveDrivetrain.setControl(_drive.withVelocityX(50).withVelocityY(50).withRotationalRate(50));
  }

  /**
   * Tells the robot that if its at the right postion to keep ging if not then hold game pieace.
   */
  @Override
  public void execute() {
    var isElevatorAtPosition = armSubsystem.isElevatorAtPosition();
    var isArmAtPosition = armSubsystem.isArmAtPosition();
    if (isElevatorAtPosition && isArmAtPosition) {
      gamePieceManipulatorSubsystem.scoreCoral();
    } else {
      gamePieceManipulatorSubsystem.activeHoldGamePiece();
    }
  }

  public boolean isFinished() {
    return false;
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
