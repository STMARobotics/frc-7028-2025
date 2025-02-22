package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj.util.Color.kGreen;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.DriveToReefAlgaeCommand;
import frc.robot.commands.DriveToReefCommand;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Library of commands to perform complex actions
 */
public class AutoCommands {

  private final CommandSwerveDrivetrain drivetrain;
  private final ArmSubsystem armSubsystem;
  private final AlignmentSubsystem alignmentSubsystem;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final LEDSubsystem ledSubsystem;

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
      LEDSubsystem ledSubsystem) {
    this.drivetrain = drivetrain;
    this.armSubsystem = armSubsystem;
    this.alignmentSubsystem = alignmentSubsystem;
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
    this.ledSubsystem = ledSubsystem;
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

  /**
   * Creates a command that will:
   * <ol>
   * <li>Drive to the nearest algae location</li>
   * <li>Align to the reef using the CANranges</li>
   * <li>Get the arm and elevator into position to grab the algae on level 1</li>
   * <li>Intake the algae</li>
   * </ol>
   * 
   * @return new command
   */
  public Command autoScoreAlgaeLevel1() {
    return autoScoreAlgae(armSubsystem::moveToAlgaeLevel1);
  }

  /**
   * Creates a command that will:
   * <ol>
   * <li>Drive to the nearest algae location</li>
   * <li>Align to the reef using the CANranges</li>
   * <li>Get the arm and elevator into position to grab the algae on level 2</li>
   * <li>Intake the algae</li>
   * </ol>
   * 
   * @return new command
   */
  public Command autoScoreAlgaeLevel2() {
    return autoScoreAlgae(armSubsystem::moveToAlgaeLevel2);
  }

  private Command autoScoreCoral(Runnable armMethod) {
    // I'd like to try raising the arm right away
    var driveToReef = new DriveToReefCommand(drivetrain, () -> drivetrain.getState().Pose);
    var alignToReef = new AlignToReefCommand(drivetrain, alignmentSubsystem, Meters.of(0.34));
    return ledSubsystem
        .setLEDSegmentsAsCommand(kGreen, armSubsystem::isAtPosition, driveToReef::isFinished, alignToReef::isFinished)
        .alongWith(
            armSubsystem.run(armMethod)
                .until(armSubsystem::isAtPosition)
                .alongWith(
                    driveToReef.andThen(alignToReef)
                        .andThen(drivetrain.applyRequest(SwerveRequest.SwerveDriveBrake::new)))
                .andThen(
                    gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::ejectCoral).withTimeout(1.0)));
  }

  private Command autoScoreAlgae(Runnable armMethod) {
    var driveToReef = new DriveToReefAlgaeCommand(drivetrain, () -> drivetrain.getState().Pose);
    var alignToReef = new AlignToReefCommand(drivetrain, alignmentSubsystem, Meters.of(0.34));
    return driveToReef.andThen(alignToReef)
        .andThen(drivetrain.applyRequest(SwerveRequest.SwerveDriveBrake::new))
        .andThen(armSubsystem.run(armMethod))
        .alongWith(gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::intakeAlgae));
  }
}
