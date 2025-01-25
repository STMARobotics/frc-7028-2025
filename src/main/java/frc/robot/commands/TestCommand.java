package main.java.frc.robot.commands;

import static frc.robot.Constants.TestingConstants.INDEXER_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.MANIPULATOR_TESTING_SPEED;

import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class TestCommand extends Command {

  private final IndexerSubsystem indexersubsystem = new Indexersubsystem();
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem = new GamePieceManipulatorSubsystem();

  private boolean hasStopped = false;
  private int teststate = 0;

  private Timer timer = new Timer();

  public TestCommand(IndexerSubsystem indexersubsystem, GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem) {
    this.indexersubsystem = indexersubsystem;
    this.GamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;

    addRequirements(indexersubsystem, gamePieceManipulatorSubsystem);
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
        }
        if (timer.hasElapsed(2) && !hasStopped) {
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

      case 1:
        if (!hasStopped) {
          indexersubsystem.runBelt(-INDEXER_TESTING_SPEED);
        }
        if (timer.hasElapsed(2) && !hasStopped) {
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
        }
        if (timer.hasElapsed(2) && !hasStopped) {
          gamePieceManipulatorSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }

      case 3:
        if (!hasStopped) {
          gamePieceManipulatorSubsystem.runManipulatorWheels(MANIPULATOR_TESTING_SPEED);
        }
        if (timer.hasElapsed(2) && !hasStopped) {
          gamePieceManipulatorSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
    }
  }

  public int getTestState() {
    return teststate;
  }

  public boolean getHasStopped() {
    return hasStopped;
  }

  @Override
  public boolean isFinished() {
    return !RobotState.isTest();
  }

  @Override
  public void end(boolean interrupted) {
    indexersubsystem.stop();
    gamePieceManipulatorSubsystem.stop();
  }
}
