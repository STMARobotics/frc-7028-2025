package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * Command to score coral in the reef.
 */
public class ScoreCoralCommand extends Command {

  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final CommandSwerveDrivetrain swerveDrivetrain;
  private final int levelToScore;

  /**
   * Constructs a new ScoreCoralCommand
   * 
   * @param manipulatorSubsystem manipulator subsytem
   * @param armSubsystem arm subsystem
   * @param drivetrainSubsystem drivetrain subsystem
   * @param levelToScore level to score [1, 4]
   */
  public ScoreCoralCommand(
      GamePieceManipulatorSubsystem manipulatorSubsystem,
      ArmSubsystem armSubsystem,
      CommandSwerveDrivetrain drivetrainSubsystem,
      int levelToScore) {

    this.gamePieceManipulatorSubsystem = manipulatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.levelToScore = levelToScore;
    this.swerveDrivetrain = drivetrainSubsystem;

    addRequirements(swerveDrivetrain, armSubsystem, gamePieceManipulatorSubsystem);
  }

  @Override
  public void initialize() {
    switch (levelToScore) {
      // Branch level on the reef to score
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
  }

  @Override
  public void execute() {
    var isElevatorAtPosition = armSubsystem.isElevatorAtPosition();
    var isArmAtPosition = armSubsystem.isArmAtPosition();
    var isHoldingCoral = gamePieceManipulatorSubsystem.isCoralInEffector();
    if (isElevatorAtPosition && isArmAtPosition && isHoldingCoral) {
      armSubsystem.stopArm();
      armSubsystem.stopElevator();
      gamePieceManipulatorSubsystem.scoreCoral();
    } else {
      gamePieceManipulatorSubsystem.activeHoldGamePiece();
    }
  }

  public boolean isFinished() {
    return !gamePieceManipulatorSubsystem.isCoralInEffector();
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveElevatorToDefault();
    armSubsystem.moveArmToIntake();
    gamePieceManipulatorSubsystem.stop();
  }
}
