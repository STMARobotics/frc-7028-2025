// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static frc.robot.Constants.GamePieceManipulatorConstants.DEVICE_ID_CANrange;
import static frc.robot.Constants.GamePieceManipulatorConstants.DEVICE_ID_MANIPULATORMOTOR;
import static frc.robot.Constants.GamePieceManipulatorConstants.IntakeSpeed;
import static frc.robot.Constants.GamePieceManipulatorConstants.OuttakeSpeed;
import static frc.robot.Constants.GamePieceManipulatorConstants.ScoreSpeed;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePieceManipulatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // define motors
  public final TalonFX manipulatorMotor = new TalonFX(DEVICE_ID_MANIPULATORMOTOR);

  CANrange canrange = new CANrange(DEVICE_ID_CANrange);
  // Configure the bacic use
  // CANrangeConfiguration configs = new CANrangeConfiguration();

  // write these configs to CANrange
  // canrange.getConfigurator().apply(configs);

  StatusSignal<Distance> distance = canrange.getDistance();

  // Convert distance in to a float so we can use it
  String distanceString = distance.refresh().toString();
  float convertedDistance = Float.parseFloat(distanceString);

  // define how motors are controlled
  private final VelocityTorqueCurrentFOC wheelControl = new VelocityTorqueCurrentFOC(0);

  public GamePieceManipulatorSubsystem() {
  }

  public void intakeCoral() {
    manipulatorMotor.setControl(wheelControl.withVelocity(IntakeSpeed));
  }

  public void outtakeCoral() {
    manipulatorMotor.setControl(wheelControl.withVelocity(OuttakeSpeed));
  }

  public void scoreCoral() {
    manipulatorMotor.setControl(wheelControl.withVelocity(ScoreSpeed));
  }

  public void stop() {
    manipulatorMotor.stopMotor();
  }

  public void update() {
    if (convertedDistance > 20) {
      stop();
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
      /* one-time action goes here */
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
