package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static frc.robot.Constants.TestingConstants.CLIMB_TESTING_VOLTAGE;
import static frc.robot.Constants.TestingConstants.MANIPULATOR_BACKWARDS_TESTING_SPEED;
import static frc.robot.Constants.TestingConstants.MANIPULATOR_TESTING_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;

/**
 * Command factory for TestMode
 */
public class TestMode {

  private boolean manipulatorForwardsTest;
  private boolean manipulatorBackwardsTest;
  private boolean climbTest;
  private boolean armElevatorTest;
  private boolean armTest;
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem;
  private final ClimbSubsystem climbSubsystem;
  private final ArmSubsystem armSubsystem;

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
      ArmSubsystem armSubsystem) {
    this.gamePieceManipulatorSubsystem = gamePieceManipulatorSubsystem;
    this.climbSubsystem = climbSubsystem;
    this.armSubsystem = armSubsystem;
  }

  /**
   * Command to run all the tests in the TestMode routine
   */
  public Command testCommand() {
    return testGamePieceManipulatorForwardsCommand().andThen(testGamePieceManipulatorBackwardsCommand())
        .andThen(testClimbCommand())
        .andThen(testArmElevatorCommand())
        .andThen(testArmCommand());

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
