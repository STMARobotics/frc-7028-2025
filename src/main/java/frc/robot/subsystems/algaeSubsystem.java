// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeSubsystemConstants.DEVICE_ID_ROLLERMOTOR;
import static frc.robot.Constants.AlgaeSubsystemConstants.intakeSpeed;
import static frc.robot.Constants.AlgaeSubsystemConstants.outtakeSpeed;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class algaeSubsystem extends SubsystemBase {
  
  //define motors
  public final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLERMOTOR);
  public final TalonFX wristMotor = new TalonFX(DEVICE_ID_ROLLERMOTOR);
  //define how motors are controlled
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC wristControl = new VelocityTorqueCurrentFOC(0.0);
  
  //leave, score coral holding, go back to human player to get coral and score it if time permits i.e cycle as many coral as you can

  public algaeSubsystem() {}
  
  public void intake() {
    rollerMotor.setControl(rollerControl.withVelocity(Constants.AlgaeSubsystemConstants.intakeSpeed));
  }
  
  //might want different outtake and scoring speeds, keep in mind for later
  public void outtake() {
    rollerMotor.setControl(rollerControl.withVelocity(Constants.AlgaeSubsystemConstants.outtakeSpeed));
  }

  public void stop() {
    rollerMotor.stopMotor();
  }

  //score function probably has to be different from outtake due to speeds of rollers
  public void score() {
    rollerMotor.setControl(rollerControl.withVelocity(Constants.AlgaeSubsystemConstants.scoreSpeed));
  }

  public void moveIntakeDown() {
    wristMotor.setControl(wristControl.withVelocity(Constants.AlgaeSubsystemConstants.wristDownSpeed));
  }

  public void moveIntakeUp() {
    wristMotor.setControl(wristControl.withVelocity(Constants.AlgaeSubsystemConstants.wristUpSpeed));
  }
}


