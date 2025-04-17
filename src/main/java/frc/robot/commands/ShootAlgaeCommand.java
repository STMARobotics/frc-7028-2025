package frc.robot.commands;

import static edu.wpi.first.wpilibj.LEDPattern.progressMaskLayer;
import static edu.wpi.first.wpilibj.LEDPattern.solid;
import static edu.wpi.first.wpilibj.util.Color.kAqua;
import static edu.wpi.first.wpilibj.util.Color.kGreen;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command to launch algae onto the barge. A.K.A. LeBron or Kareem Sky Hook.
 */
public class ShootAlgaeCommand extends Command {
  private static final double LAUNCH_HEIGHT_METERS = 0.4;

  private final ArmSubsystem armSubsystem;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Timer shootTimer = new Timer();

  /**
   * Constructs a new ShootAlgaeCommand.
   * 
   * @param armSubsystem arm subsystem
   * @param gamePieceManipulatorSubsystem game piece manipulator subsystem
   * @param ledSubsystem led subsystem
   */
  public ShootAlgaeCommand(
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
    shootTimer.stop();
    shootTimer.reset();
  }

  @Override
  public void execute() {
    // Move the elevator and arm to the barge
    armSubsystem.moveToBarge();
    if (isArmReady()) {
      // Release the algae
      gamePieceManipulatorSubsystem.shootAlgae();
      ledSubsystem.runPattern(solid(kGreen));
      shootTimer.start();
    } else {
      // Hold the algae
      gamePieceManipulatorSubsystem.activeHoldAlgae();
      ledSubsystem.runPattern(solid(kAqua).mask(progressMaskLayer(this::getArmPercentReady)));
    }
  }

  private boolean isArmReady() {
    return armSubsystem.getElevatorMeters() >= LAUNCH_HEIGHT_METERS;
  }

  private double getArmPercentReady() {
    return armSubsystem.getElevatorMeters() / LAUNCH_HEIGHT_METERS;
  }

  @Override
  public boolean isFinished() {
    return armSubsystem.isAtPosition() && shootTimer.isRunning() && shootTimer.hasElapsed(0.2);
  }

  @Override
  public void end(boolean interrupted) {
    gamePieceManipulatorSubsystem.stop();
    armSubsystem.stop();
    ledSubsystem.runPattern(LEDPattern.kOff);
    shootTimer.stop();
  }

}
