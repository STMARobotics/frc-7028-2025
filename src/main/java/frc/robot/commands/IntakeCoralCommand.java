package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command to intake coral from the human player at the Coral Station
 */
public class IntakeCoralCommand extends Command {

  private final IndexerSubsystem indexerSubsystem;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final LEDSubsystem ledSubsystem;

  /**
   * Constructs an IntakeCoralAndHoldCommand
   *
   * @param indexerSubsystem indexer subsystem
   * @param gamePieceManipulatorSubsystem manipulator subsystem
   * @param armSubsystem arm subsystem
   * @param ledSubsystem LED subsystem
   */
  public IntakeCoralCommand(
      IndexerSubsystem indexerSubsystem,
      GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem,
      ArmSubsystem armSubsystem,
      LEDSubsystem ledSubsystem) {
    this.indexerSubsystem = indexerSubsystem;
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.ledSubsystem = ledSubsystem;

    addRequirements(indexerSubsystem, gamePieceManipulatorSubsystem, armSubsystem, ledSubsystem);
  }

  @Override
  public void execute() {
    armSubsystem.moveToCoralIntakePosition();
    if (armSubsystem.isAtPosition()) {
      // Run the belt if the arm is in position
      gamePieceManipulatorSubsystem.intakeCoral();
      indexerSubsystem.intake();
    } else {
      gamePieceManipulatorSubsystem.activeHoldCoral();
      indexerSubsystem.stop();
    }
    ledSubsystem.runPattern(
        LEDPattern.gradient(GradientType.kContinuous, Color.kBlack, Color.kWhite)
            .scrollAtRelativeSpeed(Percent.per(Second).of(300))
            .reversed());
  }

  @Override
  public boolean isFinished() {
    return armSubsystem.hasCoral();
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    gamePieceManipulatorSubsystem.stop();
    armSubsystem.stop();
    ledSubsystem.runPattern(LEDPattern.kOff);
  }
}