package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.TestingConstants.CLIMB_TESTING_VOLTAGE;
import static frc.robot.Constants.TestingConstants.INDEXER_BACKWARDS_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.INDEXER_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.MANIPULATOR_BACKWARDS_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.MANIPULATOR_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.ROLLER_BACKWARDS_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.ROLLER_TESTING_SPEED;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class TestCommand extends Command {

  private final IndexerSubsystem indexersubsystem;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final AlgaeSubsystem algaeSubsystem;
  private final ClimbSubsystem climbSubsystem;
  private final ArmSubsystem armSubsystem;

  private boolean hasStopped = false;
  private int teststate = 0;

  private Timer timer = new Timer();

  public TestCommand(
      IndexerSubsystem indexersubsystem,
      GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem,
      AlgaeSubsystem algaeSubsystem,
      ClimbSubsystem climbSubsystem,
      ArmSubsystem armSubsystem) {
    this.indexersubsystem = indexersubsystem;
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
    this.algaeSubsystem = algaeSubsystem;
    this.climbSubsystem = climbSubsystem;
    this.armSubsystem = armSubsystem;

    addRequirements(indexersubsystem, gamePieceManipulatorSubsystem, algaeSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    teststate = 0;
    hasStopped = false;
  }

  @Override
  public void execute() {
    switch (teststate) {
      default:
        break;

      case 0:
        if (!hasStopped) {
          indexersubsystem.runBelt(INDEXER_TESTING_SPEED);
          timer.start();
        }
        if (indexersubsystem.getIndexerSpeed() == (INDEXER_TESTING_SPEED.in(RadiansPerSecond)) && !hasStopped) {
          indexersubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          hasStopped = false;
        }
        break;

      case 1:
        if (!hasStopped) {
          indexersubsystem.runBelt(INDEXER_BACKWARDS_TESTING_SPEED);
          timer.start();
        }
        if (indexersubsystem.getIndexerSpeed() == (INDEXER_BACKWARDS_TESTING_SPEED.in(RadiansPerSecond))
            && !hasStopped) {
          indexersubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 2:
        if (!hasStopped) {
          gamePieceManipulatorSubsystem.runManipulatorWheels(MANIPULATOR_TESTING_SPEED);
          timer.start();
        }
        if (gamePieceManipulatorSubsystem.getManipulatorSpeed() == (MANIPULATOR_TESTING_SPEED.in(RadiansPerSecond))
            && !hasStopped) {
          gamePieceManipulatorSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 3:
        if (!hasStopped) {
          gamePieceManipulatorSubsystem.runManipulatorWheels(MANIPULATOR_BACKWARDS_TESTING_SPEED);
          timer.start();
        }
        if (gamePieceManipulatorSubsystem
            .getManipulatorSpeed() == (MANIPULATOR_BACKWARDS_TESTING_SPEED.in(RadiansPerSecond)) && !hasStopped) {
          gamePieceManipulatorSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 4:
        if (!hasStopped) {
          algaeSubsystem.runRollers(ROLLER_TESTING_SPEED);
          timer.start();
        }
        if (algaeSubsystem.getRollerSpeed() == (ROLLER_TESTING_SPEED.in(RadiansPerSecond)) && !hasStopped) {
          algaeSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 5:
        if (!hasStopped) {
          algaeSubsystem.runRollers(ROLLER_BACKWARDS_TESTING_SPEED);
          timer.start();
        }
        if (algaeSubsystem.getRollerSpeed() == (ROLLER_BACKWARDS_TESTING_SPEED.in(RadiansPerSecond)) && !hasStopped) {
          algaeSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 6:
        if (!hasStopped) {
          algaeSubsystem.moveIntakeDown();
        }
        if (algaeSubsystem.isWristAtPosition() && !hasStopped) {
          algaeSubsystem.stop();
          hasStopped = true;
          timer.start();
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 7:
        if (!hasStopped) {
          algaeSubsystem.moveIntakeUp();
        }
        if (algaeSubsystem.isWristAtPosition() && !hasStopped) {
          algaeSubsystem.stop();
          hasStopped = true;
          timer.start();
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 8:
        if (!hasStopped) {
          climbSubsystem.runBackClimb(CLIMB_TESTING_VOLTAGE.in(Volts));
          climbSubsystem.runFrontClimb(CLIMB_TESTING_VOLTAGE.in(Volts));
          timer.start();
        }
        if (timer.hasElapsed(2) && !hasStopped) {
          climbSubsystem.stopMotors();
          hasStopped = true;
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 9:
        if (!hasStopped) {
          armSubsystem.moveElevatorLevel4();
        }
        if (armSubsystem.isElevatorAtPosition() && !hasStopped) {
          armSubsystem.moveElevatorToDefault();
          hasStopped = true;
          timer.start();
        }
        if (timer.hasElapsed(4) && hasStopped && armSubsystem.isElevatorAtPosition()) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 10:
        if (!hasStopped) {
          armSubsystem.moveArmToLevel4();
        }
        if (armSubsystem.isArmAtPosition() && !hasStopped) {
          armSubsystem.moveArmToIntake();
          hasStopped = true;
          timer.start();
        }
        if (timer.hasElapsed(4) && hasStopped && armSubsystem.isArmAtPosition()) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;
    }
  }

  /*
   * Returns the amount of tests that have been completed
   */
  public int getTestState() {
    return teststate;
  }

  @Override
  public boolean isFinished() {
    return !RobotState.isTest();
  }

  @Override
  public void end(boolean interrupted) {
    indexersubsystem.stop();
    gamePieceManipulatorSubsystem.stop();
    climbSubsystem.stopMotors();
    algaeSubsystem.stop();
  }
}
