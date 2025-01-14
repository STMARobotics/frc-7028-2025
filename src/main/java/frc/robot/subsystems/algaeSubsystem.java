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
  
  //define motor
  public final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLERMOTOR);
  //define how motor is controlled
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  
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
}


