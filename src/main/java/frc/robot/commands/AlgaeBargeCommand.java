package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ArmConstants.ALGAE_BARGE_ANGLE;
import static frc.robot.Constants.ArmConstants.ALGAE_BARGE_HEIGHT;

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
    ledSubsystem.setLEDSegments(Color.kGreen, this::isElevatorReady, this::isArmReady);

    if (isElevatorReady() && isArmReady()) {
      // Launch the algae when the elevator and arm are >= 75% of the way to the barge
      gamePieceManipulatorSubsystem.shootAlgae();
    }
  }

  private boolean isElevatorReady() {
    return armSubsystem.getElevatorMeters() >= ALGAE_BARGE_HEIGHT.in(Meters) * 0.75;
  }

  private boolean isArmReady() {
    return armSubsystem.getArmAngle().gte(ALGAE_BARGE_ANGLE);
  }

  @Override
  public void end(boolean interrupted) {
    gamePieceManipulatorSubsystem.stop();
    armSubsystem.stop();
    ledSubsystem.runPattern(LEDPattern.kOff);
  }

}
