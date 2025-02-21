package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.DriveToReefCommand;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Library of commands to perform complex actions
 */
public class AutoCommands {

  private final CommandSwerveDrivetrain drivetrain;
  private final ArmSubsystem armSubsystem;
  private final AlignmentSubsystem alignmentSubsystem;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;

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
      IndexerSubsystem indexerSubsystem) {
    this.drivetrain = drivetrain;
    this.armSubsystem = armSubsystem;
    this.alignmentSubsystem = alignmentSubsystem;
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
  }

  /**
   * Creates a command that will:
   * <ol>
   * <li>Drive to the nearest reef scoring location</li>
   * <li>Align to the reef using the CANranges</li>
   * <li>Get the arm and elevator into position to score on level 4</li>
   * <li>Eject the coral</li>
   * </ol>
   * 
   * @return new command
   */
  public Command autoScoreCoralLevel4() {
    return autoScoreCoral(armSubsystem::moveToLevel4);
  }

  /**
   * Creates a command that will:
   * <ol>
   * <li>Drive to the nearest reef scoring location</li>
   * <li>Align to the reef using the CANranges</li>
   * <li>Get the arm and elevator into position to score on level 3</li>
   * <li>Eject the coral</li>
   * </ol>
   * 
   * @return new command
   */
  public Command autoScoreCoralLevel3() {
    return autoScoreCoral(armSubsystem::moveToLevel3);
  }

  private Command autoScoreCoral(Runnable armMethod) {
    return new DriveToReefCommand(drivetrain, () -> drivetrain.getState().Pose).andThen(
        armSubsystem.run(armMethod)
            .until(armSubsystem::isAtPosition)
            .alongWith(new AlignToReefCommand(drivetrain, alignmentSubsystem, Meters.of(0.34)))
            .andThen(
                armSubsystem.run(armMethod)
                    .alongWith(
                        gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::ejectCoral)
                            .alongWith(drivetrain.applyRequest(SwerveRequest.SwerveDriveBrake::new)))
                    .withTimeout(1.0)));
  }

}
