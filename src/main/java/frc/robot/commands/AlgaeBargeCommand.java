package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.LEDSubsystem.ledSegments;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command to try to launch algae onto the barge.
 */
public class AlgaeBargeCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final LEDSubsystem ledSubsystem;

  /**
   * Constructs a new AlgaeBargeCommand.
   * 
   * @param armSubsystem arm subsystem
   * @param gamePieceManipulatorSubsystem game piece manipulator subsystem
   * @param ledSubsystem led subsystem
   */
  public AlgaeBargeCommand(
      ArmSubsystem armSubsystem,
      GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem,
      LEDSubsystem ledSubsystem) {
    this.armSubsystem = armSubsystem;
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
    this.ledSubsystem = ledSubsystem;

    addRequirements(armSubsystem, gamePieceManipulatorSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    gamePieceManipulatorSubsystem.activeHoldAlgae();
  }

  @Override
  public void execute() {
    // Move the elevator and arm to the barge
    armSubsystem.moveToBarge();
    ledSubsystem.runPattern(ledSegments(Color.kGreen, this::isArmReady));

    if (isArmReady()) {
      // Release the algae
      gamePieceManipulatorSubsystem.shootAlgae();
    }
  }

  private boolean isArmReady() {
    return armSubsystem.getArmAngleNormalized().in(Rotations) <= 0.47;
  }

  @Override
  public void end(boolean interrupted) {
    gamePieceManipulatorSubsystem.stop();
    armSubsystem.stop();
    ledSubsystem.runPattern(LEDPattern.kOff);
  }

}
