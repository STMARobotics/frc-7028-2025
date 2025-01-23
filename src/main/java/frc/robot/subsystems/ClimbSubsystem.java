package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.FeedbackSensorSourceValue.FusedCANcoder;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimbConstants.CAGE_DETECTION_THRESHOLD_DISTANCE;
import static frc.robot.Constants.ClimbConstants.CLIMB_LIMIT_FORWARD;
import static frc.robot.Constants.ClimbConstants.CLIMB_LIMIT_REVERSE;
import static frc.robot.Constants.ClimbConstants.CLIMB_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.ClimbConstants.CLIMB_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_DETECTION;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_ENCODER_1;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_MOTOR_1;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_MOTOR_2;
import static frc.robot.Constants.ClimbConstants.LEFT_CLIMB_MAGNETIC_OFFSET;
import static frc.robot.Constants.ClimbConstants.MAX_CLIMB_VOLTAGE;
import static frc.robot.Constants.ClimbConstants.RIGHT_CLIMB_MAGNETIC_OFFSET;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private static final CANBus CANIVORE_BUS_NAME = null;

  private final TalonFX leftMotor = new TalonFX(DEVICE_ID_CLIMB_MOTOR_1, CANIVORE_BUS_NAME);
  private final TalonFX rightMotor = new TalonFX(DEVICE_ID_CLIMB_MOTOR_2, CANIVORE_BUS_NAME);

  private final CANcoder leftClimbEncoder = new CANcoder(DEVICE_ID_CLIMB_ENCODER_1, CANIVORE_BUS_NAME);
  private final CANcoder rightClimbEncoder = new CANcoder(DEVICE_ID_CLIMB_ENCODER_1, CANIVORE_BUS_NAME);

  private final CANrange cageDetection = new CANrange(DEVICE_ID_CLIMB_DETECTION, CANIVORE_BUS_NAME);

  private final VoltageOut climbControl = new VoltageOut(0.0).withEnableFOC(true);
  private final StatusSignal<Distance> cageDistance = cageDetection.getDistance();

  public ClimbSubsystem() {
    var climbTalonConfig = new TalonFXConfiguration();
    climbTalonConfig.MotorOutput.NeutralMode = Brake;
    climbTalonConfig.CurrentLimits.StatorCurrentLimit = CLIMB_STATOR_CURRENT_LIMIT.in(Amps);
    climbTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climbTalonConfig.CurrentLimits.SupplyCurrentLimit = CLIMB_SUPPLY_CURRENT_LIMIT.in(Amps);
    climbTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climbTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbTalonConfig.Feedback.RotorToSensorRatio = CLIMB_ROTOR_TO_SENSOR_RATIO; // 25 rotor turns = 1 shaft turn
    climbTalonConfig.Feedback.FeedbackSensorSource = FusedCANcoder;
    climbTalonConfig.Feedback.FeedbackRemoteSensorID = leftClimbEncoder.getDeviceID();
    climbTalonConfig.Feedback.FeedbackRemoteSensorID = rightClimbEncoder.getDeviceID();
    climbTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climbTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CLIMB_LIMIT_FORWARD.in(Radians);
    climbTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climbTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = CLIMB_LIMIT_REVERSE.in(Radians);

    leftMotor.getConfigurator().apply(climbTalonConfig);
    rightMotor.getConfigurator().apply(climbTalonConfig);

    var climbCANCoderConfig = new CANcoderConfiguration();
    climbCANCoderConfig.MagnetSensor.MagnetOffset = LEFT_CLIMB_MAGNETIC_OFFSET.in(Rotations);
    climbCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    leftClimbEncoder.getConfigurator().apply(climbCANCoderConfig);

    climbCANCoderConfig.MagnetSensor.MagnetOffset = RIGHT_CLIMB_MAGNETIC_OFFSET.in(Rotations);
    rightClimbEncoder.getConfigurator().apply(climbCANCoderConfig);

  };

  public void climb(double speed) {
    var climbSpeed = MathUtil.clamp(speed, -1, 1);
    leftMotor.setControl(climbControl.withOutput(MAX_CLIMB_VOLTAGE.in(Volts) * climbSpeed));
    rightMotor.setControl(climbControl.withOutput(MAX_CLIMB_VOLTAGE.in(Volts) * climbSpeed));

  }

  public boolean cageDetected() {
    return (cageDistance.refresh().getValueAsDouble() < CAGE_DETECTION_THRESHOLD_DISTANCE.in(Millimeters));
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

}
