package frc.robot.commands;

import static edu.wpi.first.wpilibj.LEDPattern.GradientType.kDiscontinuous;
import static edu.wpi.first.wpilibj.LEDPattern.gradient;
import static edu.wpi.first.wpilibj.LEDPattern.solid;
import static edu.wpi.first.wpilibj.util.Color.kBlack;
import static edu.wpi.first.wpilibj.util.Color.kCornflowerBlue;
import static edu.wpi.first.wpilibj.util.Color.kWhite;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import java.util.function.BooleanSupplier;

/**
 * Command to hold coral in the game piece manipulator with the arm parked
 */
public class HoldCoralCommand extends Command {

  private final IndexerSubsystem indexerSubsystem;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final BooleanSupplier parkForLevel1;

  /**
   * Constructs an IntakeCoralAndHoldCommand
   *
   * @param indexerSubsystem indexer subsystem
   * @param gamePieceManipulatorSubsystem manipulator subsystem
   * @param armSubsystem arm subsystem
   * @param ledSubsystem LED subsystem
   */
  public HoldCoralCommand(
      IndexerSubsystem indexerSubsystem,
      GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem,
      ArmSubsystem armSubsystem,
      LEDSubsystem ledSubsystem,
      BooleanSupplier parkForlevel1) {
    this.indexerSubsystem = indexerSubsystem;
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.parkForLevel1 = parkForlevel1;

    addRequirements(indexerSubsystem, gamePieceManipulatorSubsystem, armSubsystem, ledSubsystem);
  }

  @Override
  public void execute() {
    if (parkForLevel1.getAsBoolean()) {
      armSubsystem.moveToCoralIntakePosition();
      ledSubsystem.runPattern(solid(kCornflowerBlue));
    } else {
      armSubsystem.park();
      ledSubsystem.runPattern(gradient(kDiscontinuous, kWhite, kBlack));
    }
    gamePieceManipulatorSubsystem.activeHoldCoral();
    indexerSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return !armSubsystem.hasCoral();
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    gamePieceManipulatorSubsystem.stop();
    armSubsystem.stop();
    ledSubsystem.off();
  }
}