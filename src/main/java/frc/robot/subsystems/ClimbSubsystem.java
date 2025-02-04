package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ClimbConstants.CLIMB_MAGNETIC_OFFSET_BACK;
import static frc.robot.Constants.ClimbConstants.CLIMB_MAGNETIC_OFFSET_FRONT;
import static frc.robot.Constants.ClimbConstants.CLIMB_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.ClimbConstants.CLIMB_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_ENCODER_FRONT;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_MOTOR_BACK;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_MOTOR_FRONT;
import static frc.robot.Constants.ClimbConstants.MAX_CLIMB_VOLTAGE;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  private final TalonFX frontMotor = new TalonFX(DEVICE_ID_CLIMB_MOTOR_FRONT, CANIVORE_BUS_NAME);
  private final TalonFX backMotor = new TalonFX(DEVICE_ID_CLIMB_MOTOR_BACK, CANIVORE_BUS_NAME);
  private final CANdi climbCANdi = new CANdi(DEVICE_ID_CLIMB_ENCODER_FRONT, CANIVORE_BUS_NAME);

  private final VoltageOut climbControlFront = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut climbControlBack = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> climbFrontVelocitySignal = frontMotor.getVelocity();
  private final StatusSignal<AngularVelocity> climbBackVelocitySignal = backMotor.getVelocity();

  public ClimbSubsystem() {
    var climbCANCoderConfig = new CANdiConfiguration();
    climbCANCoderConfig.PWM1.withAbsoluteSensorOffset(CLIMB_MAGNETIC_OFFSET_FRONT)
        .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5))
        .withSensorDirection(true);
    climbCANCoderConfig.PWM2.withAbsoluteSensorOffset(CLIMB_MAGNETIC_OFFSET_BACK)
        .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5))
        .withSensorDirection(true);

    climbCANdi.getConfigurator().apply(climbCANCoderConfig);

    var climbTalonConfig = new TalonFXConfiguration();
    climbTalonConfig.MotorOutput.withNeutralMode(Brake).withInverted(InvertedValue.Clockwise_Positive);
    climbTalonConfig.CurrentLimits.withStatorCurrentLimit(CLIMB_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(CLIMB_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    climbTalonConfig.Feedback.withRotorToSensorRatio(CLIMB_ROTOR_TO_SENSOR_RATIO);
    climbTalonConfig.Feedback.FeedbackRemoteSensorID = climbCANdi.getDeviceID();
    climbTalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM1;

    frontMotor.getConfigurator().apply(climbTalonConfig);
    climbTalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM2;
    backMotor.getConfigurator().apply(climbTalonConfig);
  }

  /**
   * Runs the front climb motor
   * 
   * @param output percentage of full output to run the motor. Range [-1, 1]
   */
  public void runFrontClimb(double output) {
    climbControlFront.withOutput(MAX_CLIMB_VOLTAGE.in(Volts) * MathUtil.clamp(output, -1, 1));
    frontMotor.setControl(climbControlFront);
  }

  /**
   * Runs the back climb motor
   * 
   * @param output percentage of full output to run the motor. Range [-1, 1]
   */
  public void runBackClimb(double output) {
    climbControlBack.withOutput(MAX_CLIMB_VOLTAGE.in(Volts) * MathUtil.clamp(output, -1, 1));
    frontMotor.setControl(climbControlBack);
  }

  public void stopMotors() {
    frontMotor.stopMotor();
    backMotor.stopMotor();
  }

  /**
   * Method to determine if the climb motors are moving
   * 
   * @return true if the motor has reached one rotation per second
   */
  public boolean areClimbMotorsMoving() {
    return (climbFrontVelocitySignal.refresh().getValue().in(RotationsPerSecond) > 1
        && climbBackVelocitySignal.refresh().getValue().in(RotationsPerSecond) > 1);
  }

}
