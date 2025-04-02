package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.TestingConstants.INDEXER_TESTING_SPEED_TOLERANCE;
import static frc.robot.Constants.TestingConstants.MANIPULATOR_TESTING_SPEED_TOLERANCE;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import java.util.function.BooleanSupplier;

/**
 * Command factory for TestMode
 */
public class TestMode {

  private final NetworkTable testingTable = NetworkTableInstance.getDefault().getTable("TestMode");

  private final BooleanPublisher indexerForwardPublisher = testingTable.getBooleanTopic("Indexer Forward").publish();
  private final BooleanPublisher indexerBackwardPublisher = testingTable.getBooleanTopic("Indexer Backward").publish();
  private final BooleanPublisher manipulatorForwardPublisher = testingTable.getBooleanTopic("Manipulator Forward")
      .publish();
  private final BooleanPublisher manipulatorBackwardPublisher = testingTable.getBooleanTopic("Manipulator Backward")
      .publish();
  private final BooleanPublisher elevatorPubliser = testingTable.getBooleanTopic("Elevator").publish();
  private final BooleanPublisher armPublisher = testingTable.getBooleanTopic("Arm").publish();

  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private BooleanSubscriber[] testResults;

  /**
   * Constructs a test mode
   * 
   * @param gamePieceManipulatorSubsystem game piece manipulator subsystem
   * @param armSubsystem arm subsystem
   */
  public TestMode(
      GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem,
      ArmSubsystem armSubsystem,
      IndexerSubsystem indexerSubsystem) {
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    testResults = new BooleanSubscriber[] {
        indexerForwardPublisher.getTopic().subscribe(false),
        indexerBackwardPublisher.getTopic().subscribe(false),
        manipulatorForwardPublisher.getTopic().subscribe(false),
        manipulatorBackwardPublisher.getTopic().subscribe(false),
        elevatorPubliser.getTopic().subscribe(false),
        armPublisher.getTopic().subscribe(false) };
  };

  /**
   * Command to run all the tests in the TestMode routine
   */
  public Command testCommand() {
    return runOnce(this::initializeResults).andThen(testGamePieceManipulatorForwardsCommand())
        .andThen(testGamePieceManipulatorBackwardsCommand())
        .andThen(testArmElevatorCommand())
        .andThen(testArmCommand())
        .andThen(testIndexerForwardsCommand())
        .andThen(testIndexerBackwardsCommand())
        .andThen(finishTestCommand());
  }

  /**
   * Sets all of the results to false
   */
  private void initializeResults() {
    indexerForwardPublisher.set(false);
    indexerBackwardPublisher.set(false);
    manipulatorForwardPublisher.set(false);
    manipulatorBackwardPublisher.set(false);
    elevatorPubliser.set(false);
    armPublisher.set(false);
  }

  private Command testIndexerForwardsCommand() {
    return indexerSubsystem.run(indexerSubsystem::intake)
        .withTimeout(Seconds.of(5))
        .andThen(
            () -> indexerForwardPublisher.set(indexerSubsystem.getBeltVelocity().gte(INDEXER_TESTING_SPEED_TOLERANCE)))
        .finallyDo(indexerSubsystem::stop);
  }

  private Command testIndexerBackwardsCommand() {
    return indexerSubsystem.run(indexerSubsystem::eject)
        .withTimeout(Seconds.of(5))
        .andThen(
            () -> indexerBackwardPublisher.set(indexerSubsystem.getBeltVelocity().lte(INDEXER_TESTING_SPEED_TOLERANCE)))
        .finallyDo(indexerSubsystem::stop);
  }

  private Command testGamePieceManipulatorForwardsCommand() {
    return gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::ejectCoral)
        .withTimeout(Seconds.of(5))
        .andThen(
            () -> manipulatorForwardPublisher.set(
                gamePieceManipulatorSubsystem.getWheelVelocity().lte(MANIPULATOR_TESTING_SPEED_TOLERANCE.unaryMinus())))
        .finallyDo(gamePieceManipulatorSubsystem::stop);
  }

  private Command testGamePieceManipulatorBackwardsCommand() {
    return gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::intakeCoral)
        .withTimeout(Seconds.of(5))
        .andThen(
            () -> manipulatorBackwardPublisher.set(
                gamePieceManipulatorSubsystem.getWheelVelocity().gte(MANIPULATOR_TESTING_SPEED_TOLERANCE.unaryMinus())))
        .finallyDo(gamePieceManipulatorSubsystem::stop);
  }

  private Command testArmElevatorCommand() {
    return run(() -> armSubsystem.moveToLevel4(), armSubsystem).until(armSubsystem::isElevatorAtPosition)
        .withTimeout(Seconds.of(5))
        .andThen(() -> elevatorPubliser.set(armSubsystem.isElevatorAtPosition()))
        .finallyDo(armSubsystem::park);
  }

  private Command testArmCommand() {
    return run(() -> armSubsystem.moveToLevel4(), armSubsystem).until(armSubsystem::isArmAtAngle)
        .withTimeout(Seconds.of(5))
        .andThen(() -> armPublisher.set(armSubsystem.isArmAtAngle()))
        .finallyDo(armSubsystem::park);
  }

  private Command finishTestCommand() {
    return run(() -> armSubsystem.moveToCoralIntakePosition(), armSubsystem);
  }

  /**
   * Returns the results of the tests ran during the TestMode routine
   * 
   * @return test results
   */
  public BooleanSupplier[] getTestResults() {
    BooleanSupplier[] results = new BooleanSupplier[testResults.length];
    for (int i = 0; i < testResults.length; i++) {
      results[i] = testResults[i]::get;
    }
    return results;
  }
}