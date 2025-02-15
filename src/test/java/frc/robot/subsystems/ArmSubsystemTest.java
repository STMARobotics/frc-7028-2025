package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for the arm subsystem
 */
public class ArmSubsystemTest {

  private ArmSubsystem armSubsystem;

  @BeforeEach
  public void setUp() {
    armSubsystem = new ArmSubsystem();
  }

  @Test
  public void testCalculate_SimpleForward() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(0), Rotations.of(0.2), Rotations.of(.5), Rotations.of(.75));
    assertEquals(Rotations.of(0.2), result);
  }

  @Test
  public void testCalculate_AvoidForbidden() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(0), Rotations.of(0.8), Rotations.of(.5), Rotations.of(.75));
    assertEquals(Rotations.of(-0.2), result);
  }

  @Test
  public void testCalculate_UnwrappedForward() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(10.2), Rotations.of(0.3), Rotations.of(.5), Rotations.of(.75));
    assertEquals(Rotations.of(10.3), result);
  }

  @Test
  public void testCalculate_UnwrappedTarget() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(10.2), Rotations.of(-40.7), Rotations.of(.5), Rotations.of(.75));
    assertEquals(Rotations.of(10.3), result);
  }

  @Test
  public void testCalculate_UnwrappedAvoidForbidden() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(10.2), Rotations.of(0.8), Rotations.of(.5), Rotations.of(.75));
    assertEquals(Rotations.of(9.8), result);
  }

  @Test
  public void testCalculate_UnwrappedNegativeAvoidForbidden() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(-10.8), Rotations.of(0.8), Rotations.of(.5), Rotations.of(.75));
    assertEquals(Rotations.of(-11.2), result);
  }

  @Test
  public void testCalculate_TargetInForbiddenLow() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(0.4), Rotations.of(0.55), Rotations.of(.5), Rotations.of(.75));
    assertEquals(Rotations.of(0.5), result);
  }

  @Test
  public void testCalculate_TargetInForbiddenHigh() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(0.9), Rotations.of(0.7), Rotations.of(.5), Rotations.of(.75));
    assertEquals(Rotations.of(0.75), result);
  }

  @Test
  public void testCalculate_UnwrappedForbidden() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(0.4), Rotations.of(0.7), Rotations.of(30.5), Rotations.of(-10.25));
    assertEquals(Rotations.of(-0.25), result);
  }

  @Test
  public void testCalculate_Zero() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(10.99), Rotations.of(0.0), Rotations.of(0.8), Rotations.of(0.9));
    assertEquals(Rotations.of(11.0), result);
  }

  @Test
  public void testCalculate_OverZero() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(-0.02), Rotations.of(0.98), Rotations.of(0.8), Rotations.of(0.9));
    assertEquals(Rotations.of(-0.02), result);
  }

  @Test
  public void testCalculate_OverZeroLong() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(0.91), Rotations.of(0.7), Rotations.of(0.8), Rotations.of(0.9));
    assertEquals(Rotations.of(1.7), result);
  }

  @Test
  public void testCalculate_AtTarget() {
    var result = armSubsystem
        .calculateArmTarget(Rotations.of(0.91), Rotations.of(-0.09), Rotations.of(0.8), Rotations.of(0.9));
    assertEquals(Rotations.of(0.91), result);
  }

  @Test
  public void testNormalizeInRange() {
    var result = ArmSubsystem.normalizeArmAngle(Rotations.of(0.45));
    assertEquals(0.45, result, 0.00001);
  }

  @Test
  public void testNormalizeNegative() {
    var result = ArmSubsystem.normalizeArmAngle(Rotations.of(-0.55));
    assertEquals(0.45, result, 0.00001);
  }

  @Test
  public void testNormalizeLarge() {
    var result = ArmSubsystem.normalizeArmAngle(Rotations.of(500.38));
    assertEquals(0.38, result, 0.00001);
  }

  @Test
  public void testNormalizeSmall() {
    var result = ArmSubsystem.normalizeArmAngle(Rotations.of(-300.96));
    assertEquals(0.04, result, 0.00001);
  }

}