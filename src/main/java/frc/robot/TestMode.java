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

  private void updateIndexerForwardsTestResult() {
    indexerForwardsTest = indexerSubsystem.isIndexerAtSpeed();
  }

  private void updateIndexerBackwardsTestResult() {
    indexerBackwardsTest = indexerSubsystem.isIndexerAtSpeed();
  }

  private void updateManipulatorForwardsTestResult() {
    manipulatorForwardsTest = gamePieceManipulatorSubsystem.isManipulatorAtSpeed();
  }

  private void updateManipulatorBackwardsTestResult() {
    manipulatorBackwardsTest = gamePieceManipulatorSubsystem.isManipulatorAtSpeed();
  }

  private void updateClimbTestResult() {
    climbTest = climbSubsystem.areClimbMotorsMoving();
  }

  private void updateAlgaeRollersForwardsTestResult() {
    algaeRollersForwardsTest = algaeSubsystem.isAtRollerSpeed();
  }

  private void updateAlgaeRollersBackwardsTestResult() {
    algaeRollersBackwardsTest = algaeSubsystem.isAtRollerSpeed();
  }

  private void updateAlgaeIntakeUpTestResult() {
    algaeIntakeUpTest = algaeSubsystem.isWristAtPosition();
  }

  private void updateAlgaeIntakeDownTestResult() {
    algaeIntakeDownTest = algaeSubsystem.isWristAtPosition();
  }

  private void updateElevatorTestResult() {
    armElevatorTest = armSubsystem.isElevatorAtPosition();
  }

  private void updateArmTestResults() {
    armTest = armSubsystem.isArmAtPosition();
  }

  public boolean getIndexerForwardsTestResult() {
    return indexerForwardsTest;
  }

  public boolean getIndexerBackwardsTestResult() {
    return indexerBackwardsTest;
  }

  public boolean getManipulatorForwardsTestResult() {
    return manipulatorForwardsTest;
  }

  public boolean getManipulatorBackwardsTestResult() {
    return manipulatorBackwardsTest;
  }

  public boolean getClimbTestResult() {
    return climbTest;
  }

  public boolean getAlgaeRollersForwardsTestResult() {
    return algaeRollersForwardsTest;
  }

  public boolean getAlgaeRollersBackwardsTestResult() {
    return algaeRollersBackwardsTest;
  }

  public boolean getAlgaeIntakeUpTestResult() {
    return algaeIntakeUpTest;
  }

  public boolean getAlgaeIntakeDownTestResult() {
    return algaeIntakeDownTest;
  }

  public boolean getArmElevatorTestResult() {
    return armElevatorTest;
  }

  public boolean getArmTestResult() {
    return armTest;
  }

  private void testClimbMotors() {
    climbSubsystem.runFrontClimb(CLIMB_TESTING_VOLTAGE.in(Volts));
    climbSubsystem.runBackClimb(CLIMB_TESTING_VOLTAGE.in(Volts));
  }
}
