package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static frc.robot.Constants.TestingConstants.CLIMB_TESTING_VOLTAGE;
import static frc.robot.Constants.TestingConstants.INDEXER_BACKWARDS_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.INDEXER_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.MANIPULATOR_BACKWARDS_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.MANIPULATOR_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.ROLLER_BACKWARDS_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.ROLLER_TESTING_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Command factory for TestMode
 */
public class TestMode {

  private boolean indexerForwardsTest;
  private boolean indexerBackwardsTest;
  private boolean manipulatorForwardsTest;
  private boolean manipulatorBackwardsTest;
  private boolean climbTest;
  private boolean algaeRollersForwardsTest;
  private boolean algaeRollersBackwardsTest;
  private boolean algaeIntakeUpTest;
  private boolean algaeIntakeDownTest;
  private boolean armElevatorTest;
  private boolean armTest;
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem = new GamePieceManipulatorSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  /**
   * Command to run all the tests in the TestMode routine
   */
  public Command testCommand() {
    return testIndexerForwardsCommand().andThen(testIndexerBackwardsCommand())
        .andThen(testGamePieceManipulatorForwardsCommand())
        .andThen(testGamePieceManipulatorBackwardsCommand())
        .andThen(testClimbCommand())
        .andThen(testAlgaeRollersForwardsCommand())
        .andThen(testAlgaeRollersBackwardsCommand())
        .andThen(testAlgaeIntakeUpCommand())
        .andThen(testAlgaeIntakeDownCommand())
        .andThen(testArmElevatorCommand())
        .andThen(testArmCommand());

  }

  private Command testIndexerForwardsCommand() {
    return run(() -> indexerSubsystem.runBelt(INDEXER_TESTING_SPEED), indexerSubsystem)
        .until(indexerSubsystem::isIndexerAtSpeed)
        .withTimeout(Seconds.of(5))
        .andThen(this::updateIndexerForwardsTestResult)
        .finallyDo(indexerSubsystem::stop);
  }

  private Command testIndexerBackwardsCommand() {
    return run(() -> indexerSubsystem.runBelt(INDEXER_BACKWARDS_TESTING_SPEED), indexerSubsystem)
        .until(indexerSubsystem::isIndexerAtSpeed)
        .withTimeout(Seconds.of(5))
        .andThen(this::updateIndexerBackwardsTestResult)
        .finallyDo(indexerSubsystem::stop);
  }

  private Command testGamePieceManipulatorForwardsCommand() {
    return run(
        () -> gamePieceManipulatorSubsystem.runManipulatorWheels(MANIPULATOR_TESTING_SPEED),
          gamePieceManipulatorSubsystem)
        .until(gamePieceManipulatorSubsystem::isManipulatorAtSpeed)
        .withTimeout(Seconds.of(5))
        .andThen(this::updateManipulatorForwardsTestResult)
        .finallyDo(gamePieceManipulatorSubsystem::stop);
  }

  private Command testGamePieceManipulatorBackwardsCommand() {
    return run(
        () -> gamePieceManipulatorSubsystem.runManipulatorWheels(MANIPULATOR_BACKWARDS_TESTING_SPEED),
          gamePieceManipulatorSubsystem)
        .until(gamePieceManipulatorSubsystem::isManipulatorAtSpeed)
        .withTimeout(Seconds.of(5))
        .andThen(this::updateManipulatorBackwardsTestResult)
        .finallyDo(gamePieceManipulatorSubsystem::stop);
  }

  private Command testClimbCommand() {
    return run(() -> testClimbMotors(), climbSubsystem).until(climbSubsystem::areClimbMotorsMoving)
        .withTimeout(Seconds.of(5))
        .andThen(this::updateClimbTestResult)
        .finallyDo(climbSubsystem::stopMotors);
  }

  private Command testAlgaeRollersForwardsCommand() {
    return run(() -> algaeSubsystem.runRollers(ROLLER_TESTING_SPEED), algaeSubsystem)
        .until(algaeSubsystem::isAtRollerSpeed)
        .withTimeout(Seconds.of(5))
        .andThen(this::updateAlgaeRollersForwardsTestResult)
        .finallyDo(algaeSubsystem::stop);
  }

  private Command testAlgaeRollersBackwardsCommand() {
    return run(() -> algaeSubsystem.runRollers(ROLLER_BACKWARDS_TESTING_SPEED), algaeSubsystem)
        .until(algaeSubsystem::isAtRollerSpeed)
        .withTimeout(Seconds.of(5))
        .andThen(this::updateAlgaeRollersBackwardsTestResult)
        .finallyDo(algaeSubsystem::stop);
  }

  private Command testAlgaeIntakeUpCommand() {
    return run(() -> algaeSubsystem.moveIntakeUp(), algaeSubsystem).until(algaeSubsystem::isWristAtPosition)
        .withTimeout(Seconds.of(5))
        .andThen(this::updateAlgaeIntakeUpTestResult)
        .finallyDo(algaeSubsystem::stop);
  }

  private Command testAlgaeIntakeDownCommand() {
    return run(() -> algaeSubsystem.moveIntakeDown(), algaeSubsystem).until(algaeSubsystem::isWristAtPosition)
        .withTimeout(Seconds.of(5))
        .andThen(this::updateAlgaeIntakeDownTestResult)
        .finallyDo(algaeSubsystem::stop);
  }

  private Command testArmElevatorCommand() {
    return run(() -> armSubsystem.moveElevatorLevel4(), armSubsystem).until(armSubsystem::isElevatorAtPosition)
        .withTimeout(Seconds.of(5))
        .andThen(this::updateElevatorTestResult)
        .finallyDo(armSubsystem::moveElevatorToDefault);
  }

  private Command testArmCommand() {
    return run(() -> armSubsystem.moveArmToLevel4(), armSubsystem).until(armSubsystem::isArmAtPosition)
        .withTimeout(Seconds.of(5))
        .andThen(this::updateArmTestResults)
        .finallyDo(armSubsystem::moveArmToIntake);
  }

  /**
   * Checks if the indexer forwards test has succeeded and updates the variable accordingly
   */
  private void updateIndexerForwardsTestResult() {
    indexerForwardsTest = indexerSubsystem.isIndexerAtSpeed();
  }

  /**
   * Checks if the indexer backwards test has succeeded and updates the variable accordingly
   */
  private void updateIndexerBackwardsTestResult() {
    indexerBackwardsTest = indexerSubsystem.isIndexerAtSpeed();
  }

  /**
   * Checks if the manipulator forwards test has succeeded and updates the variable accordingly
   */
  private void updateManipulatorForwardsTestResult() {
    manipulatorForwardsTest = gamePieceManipulatorSubsystem.isManipulatorAtSpeed();
  }

  /**
   * Checks if the manipulator backwards test has succeeded and updates the variable accordingly
   */
  private void updateManipulatorBackwardsTestResult() {
    manipulatorBackwardsTest = gamePieceManipulatorSubsystem.isManipulatorAtSpeed();
  }

  /**
   * Checks if the climb test has succeeded and updates the variable accordingly
   */
  private void updateClimbTestResult() {
    climbTest = climbSubsystem.areClimbMotorsMoving();
  }

  /**
   * Checks if the algae rollers forwards test has succeeded and updates the variable accordingly
   */
  private void updateAlgaeRollersForwardsTestResult() {
    algaeRollersForwardsTest = algaeSubsystem.isAtRollerSpeed();
  }

  /**
   * Checks if the algae rollers backwards test has succeeded and updates the variable accordingly
   */
  private void updateAlgaeRollersBackwardsTestResult() {
    algaeRollersBackwardsTest = algaeSubsystem.isAtRollerSpeed();
  }

  /**
   * Checks if the algae intake up test has succeeded and updates the variable accordingly
   */
  private void updateAlgaeIntakeUpTestResult() {
    algaeIntakeUpTest = algaeSubsystem.isWristAtPosition();
  }

  /**
   * Checks if the algae intake down test has succeeded and updates the variable accordingly
   */
  private void updateAlgaeIntakeDownTestResult() {
    algaeIntakeDownTest = algaeSubsystem.isWristAtPosition();
  }

  /**
   * Checks if the elevator test has succeeded and updates the variable accordingly
   */
  private void updateElevatorTestResult() {
    armElevatorTest = armSubsystem.isElevatorAtPosition();
  }

  /**
   * Checks if the arm test has succeeded and updates the variable accordingly
   */
  private void updateArmTestResults() {
    armTest = armSubsystem.isArmAtPosition();
  }

  /**
   * Gets the result of the indexer forwards test
   *
   * @return true if the indexer forwards test has been run successfully, otherwise false
   */
  public boolean getIndexerForwardsTestResult() {
    return indexerForwardsTest;
  }

  /**
   * Gets the result of the indexer backwards test
   *
   * @return true if the indexer backwards test has been run successfully, otherwise false
   */
  public boolean getIndexerBackwardsTestResult() {
    return indexerBackwardsTest;
  }

  /**
   * Gets the result of the manipulator forwards test
   *
   * @return true if the manipulator forwards test has been run successfully, otherwise false
   */
  public boolean getManipulatorForwardsTestResult() {
    return manipulatorForwardsTest;
  }

  /**
   * Gets the result of the manipulator backwards test
   *
   * @return true if the manipulator backwards test has been run successfully, otherwise false
   */
  public boolean getManipulatorBackwardsTestResult() {
    return manipulatorBackwardsTest;
  }

  /**
   * Gets the result of the climb test
   *
   * @return true if the climb test has been run successfully, otherwise false
   */
  public boolean getClimbTestResult() {
    return climbTest;
  }

  /**
   * Gets the result of the algae rollers forwards test
   *
   * @return true if the algae rollers forwards test has been run successfully, otherwise false
   */
  public boolean getAlgaeRollersForwardsTestResult() {
    return algaeRollersForwardsTest;
  }

  /**
   * Gets the result of the algae rollers backwards test
   *
   * @return true if the algae rollers backwards test has been run successfully, otherwise false
   */
  public boolean getAlgaeRollersBackwardsTestResult() {
    return algaeRollersBackwardsTest;
  }

  /**
   * Gets the result of the algae intake up test
   *
   * @return true if the algae intake up test has been run successfully, otherwise false
   */
  public boolean getAlgaeIntakeUpTestResult() {
    return algaeIntakeUpTest;
  }

  /**
   * Gets the result of the algae intake down test
   *
   * @return true if the algae intake down test has been run successfully, otherwise false
   */
  public boolean getAlgaeIntakeDownTestResult() {
    return algaeIntakeDownTest;
  }

  /**
   * Gets the result of the elevator test
   *
   * @return true if the elevator test has been run successfully, otherwise false
   */
  public boolean getArmElevatorTestResult() {
    return armElevatorTest;
  }

  /**
   * Gets the result of the arm test
   *
   * @return true if the arm test has been run successfully, otherwise false
   */
  public boolean getArmTestResult() {
    return armTest;
  }

  private void testClimbMotors() {
    climbSubsystem.runFrontClimb(CLIMB_TESTING_VOLTAGE.in(Volts));
    climbSubsystem.runBackClimb(CLIMB_TESTING_VOLTAGE.in(Volts));
  }
}
