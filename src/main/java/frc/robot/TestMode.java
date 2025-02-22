package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.TestingConstants.INDEXER_BACKWARDS_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.INDEXER_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.MANIPULATOR_BACKWARDS_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.MANIPULATOR_TESTING_SPEED;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

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
  private final BooleanPublisher climbPublisher = testingTable.getBooleanTopic("Climb").publish();
  private final BooleanPublisher elevatorPubliser = testingTable.getBooleanTopic("Elevator").publish();
  private final BooleanPublisher armPublisher = testingTable.getBooleanTopic("Arm").publish();

  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ClimbSubsystem climbSubsystem;
  private final ArmSubsystem armSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  /**
   * Constructs a test mode
   * 
   * @param gamePieceManipulatorSubsystem game piece manipulator subsystem
   * @param climbSubsystem climb subsystem
   * @param armSubsystem arm subsystem
   */
  public TestMode(
      GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem,
      ClimbSubsystem climbSubsystem,
      ArmSubsystem armSubsystem,
      IndexerSubsystem indexerSubsystem) {
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
    this.climbSubsystem = climbSubsystem;
    this.armSubsystem = armSubsystem;
    this.indexerSubsystem = indexerSubsystem;
  }

  /**
   * Command to run all the tests in the TestMode routine
   */
  public Command testCommand() {
    return runOnce(this::initializeResults).andThen(testGamePieceManipulatorForwardsCommand())
        .andThen(testGamePieceManipulatorBackwardsCommand())
        .andThen(testClimbCommand())
        .andThen(testArmElevatorCommand())
        .andThen(testArmCommand())
        .andThen(testIndexerForwardsCommand())
        .andThen(testIndexerBackwardsCommand());

  }

  /**
   * Sets all of the results to false
   */
  private void initializeResults() {
    indexerForwardPublisher.set(false);
    indexerBackwardPublisher.set(false);
    manipulatorForwardPublisher.set(false);
    manipulatorBackwardPublisher.set(false);
    climbPublisher.set(false);
    elevatorPubliser.set(false);
    armPublisher.set(false);
  }

  private Command testIndexerForwardsCommand() {
    return run(() -> indexerSubsystem.runBelt(INDEXER_TESTING_SPEED), indexerSubsystem)
        .until(indexerSubsystem::isIndexerAtSpeed)
        .withTimeout(Seconds.of(5))
        .andThen(() -> indexerForwardPublisher.set(indexerSubsystem.isIndexerAtSpeed()))
        .finallyDo(indexerSubsystem::stop);
  }

  private Command testIndexerBackwardsCommand() {
    return run(() -> indexerSubsystem.runBelt(INDEXER_BACKWARDS_TESTING_SPEED), indexerSubsystem)
        .until(indexerSubsystem::isIndexerAtSpeed)
        .withTimeout(Seconds.of(5))
        .andThen(() -> indexerBackwardPublisher.set(indexerSubsystem.isIndexerAtSpeed()))
        .finallyDo(indexerSubsystem::stop);
  }

  private Command testGamePieceManipulatorForwardsCommand() {
    return run(
        () -> gamePieceManipulatorSubsystem.runManipulatorWheels(MANIPULATOR_TESTING_SPEED),
          gamePieceManipulatorSubsystem)
        .until(gamePieceManipulatorSubsystem::isManipulatorAtSpeed)
        .withTimeout(Seconds.of(5))
        .andThen(() -> manipulatorForwardPublisher.set(gamePieceManipulatorSubsystem.isManipulatorAtSpeed()))
        .finallyDo(gamePieceManipulatorSubsystem::stop);
  }

  private Command testGamePieceManipulatorBackwardsCommand() {
    return run(
        () -> gamePieceManipulatorSubsystem.runManipulatorWheels(MANIPULATOR_BACKWARDS_TESTING_SPEED),
          gamePieceManipulatorSubsystem)
        .until(gamePieceManipulatorSubsystem::isManipulatorAtSpeed)
        .withTimeout(Seconds.of(5))
        .andThen(() -> manipulatorBackwardPublisher.set(gamePieceManipulatorSubsystem.isManipulatorAtSpeed()))
        .finallyDo(gamePieceManipulatorSubsystem::stop);
  }

  private Command testClimbCommand() {
    return run(() -> {
      climbSubsystem.climb();
    }, climbSubsystem).until(climbSubsystem::isClimbMotorMoving)
        .withTimeout(Seconds.of(5))
        .andThen(() -> climbPublisher.set(climbSubsystem.isClimbMotorMoving()))
        .finallyDo(climbSubsystem::stop);
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

  public Boolean[] getTestResults() {
    return new Boolean[] {
        indexerForwardPublisher.getTopic().subscribe(false).get(),
        indexerBackwardPublisher.getTopic().subscribe(false).get(),
        manipulatorForwardPublisher.getTopic().subscribe(false).get(),
        manipulatorBackwardPublisher.getTopic().subscribe(false).get(),
        climbPublisher.getTopic().subscribe(false).get(),
        elevatorPubliser.getTopic().subscribe(false).get(),
        armPublisher.getTopic().subscribe(false).get() };
  }
}