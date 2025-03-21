package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
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
    } else {
      armSubsystem.park();
    }
    gamePieceManipulatorSubsystem.activeHoldCoral();
    indexerSubsystem.stop();
    ledSubsystem.runPattern(LEDPattern.gradient(GradientType.kDiscontinuous, Color.kWhite, Color.kBlack));
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
    ledSubsystem.runPattern(LEDPattern.kOff);
  }
}