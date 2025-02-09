package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ClimbConstants.CLIMB_MAGNETIC_OFFSET_BACK;
import static frc.robot.Constants.ClimbConstants.CLIMB_MAGNETIC_OFFSET_FRONT;
import static frc.robot.Constants.ClimbConstants.CLIMB_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.ClimbConstants.CLIMB_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_CANDI;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_MOTOR_BACK;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_MOTOR_FRONT;
import static frc.robot.Constants.ClimbConstants.MAX_CLIMB_VOLTAGE;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ClimbSubsystem extends SubsystemBase {

  private final TalonFX frontMotor = new TalonFX(DEVICE_ID_CLIMB_MOTOR_FRONT, CANIVORE_BUS_NAME);
  private final TalonFX backMotor = new TalonFX(DEVICE_ID_CLIMB_MOTOR_BACK, CANIVORE_BUS_NAME);
  private final CANdi climbCanDi = new CANdi(DEVICE_ID_CLIMB_CANDI, CANIVORE_BUS_NAME);

  private final VoltageOut climbControlFront = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut climbControlBack = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> climbFrontVelocitySignal = frontMotor.getVelocity();
  private final StatusSignal<AngularVelocity> climbBackVelocitySignal = backMotor.getVelocity();
  private final StatusSignal<Angle> climbFrontPositionSignal = frontMotor.getPosition();
  private final StatusSignal<Angle> climbBackPositionSignal = backMotor.getPosition();

  private final Mechanism2d climbMechanisms = new Mechanism2d(12, 5);
  private final MechanismRoot2d frontClimbRoot = climbMechanisms.getRoot("Front Climb", 3, 3);
  private final MechanismLigament2d frontClimbLigament = frontClimbRoot
      .append(new MechanismLigament2d("Front Climb Lever", 2, 0, 3, new Color8Bit(Color.kYellow)));
  private final MechanismRoot2d backClimbRoot = climbMechanisms.getRoot("Back Climb", 8, 3);
  private final MechanismLigament2d backClimbLigament = backClimbRoot
      .append(new MechanismLigament2d("Back Climb Lever", 2, 180, 3, new Color8Bit(Color.kBlue)));

  private final DCMotorSim frontMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, CLIMB_ROTOR_TO_SENSOR_RATIO),
      DCMotor.getKrakenX60Foc(1));
  private final DCMotorSim backMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, CLIMB_ROTOR_TO_SENSOR_RATIO),
      DCMotor.getKrakenX60Foc(1));

  private final TalonFXSimState frontMotorSimState = frontMotor.getSimState();
  private final TalonFXSimState backMotorSimState = backMotor.getSimState();

  public ClimbSubsystem() {
    var climbCANCoderConfig = new CANdiConfiguration();
    climbCANCoderConfig.PWM1.withAbsoluteSensorOffset(CLIMB_MAGNETIC_OFFSET_FRONT)
        .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5))
        .withSensorDirection(true);
    climbCANCoderConfig.PWM2.withAbsoluteSensorOffset(CLIMB_MAGNETIC_OFFSET_BACK)
        .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5))
        .withSensorDirection(true);

    climbCanDi.getConfigurator().apply(climbCANCoderConfig);

    var climbTalonConfig = new TalonFXConfiguration();
    climbTalonConfig.MotorOutput.withNeutralMode(Brake).withInverted(InvertedValue.Clockwise_Positive);
    climbTalonConfig.CurrentLimits.withStatorCurrentLimit(CLIMB_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(CLIMB_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    if (Robot.isReal()) {
      // CANdi doesn't support sim
      climbTalonConfig.Feedback.withRotorToSensorRatio(CLIMB_ROTOR_TO_SENSOR_RATIO).withFusedCANdiPwm1(climbCanDi);
    }

    frontMotor.getConfigurator().apply(climbTalonConfig);
    if (Robot.isReal()) {
      // CANdi doesn't support sim
      climbTalonConfig.Feedback.withFusedCANdiPwm2(climbCanDi);
    }
    backMotor.getConfigurator().apply(climbTalonConfig);

    SmartDashboard.putData("Climb", climbMechanisms);
  }

  /**
   * Runs the front climb motor
   * 
   * @param output percentage of full output to run the motor. Range [-1, 1]
   */
  public void runFrontClimb(double output) {
    frontMotor.setControl(climbControlFront.withOutput(MAX_CLIMB_VOLTAGE.in(Volts) * MathUtil.clamp(output, -1, 1)));
  }

  /**
   * Runs the back climb motor
   * 
   * @param output percentage of full output to run the motor. Range [-1, 1]
   */
  public void runBackClimb(double output) {
    backMotor.setControl(climbControlBack.withOutput(MAX_CLIMB_VOLTAGE.in(Volts) * MathUtil.clamp(output, -1, 1)));
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

  @Override
  public void periodic() {
    frontClimbLigament
        .setAngle(-climbFrontPositionSignal.refresh().getValue().in(Degrees) / CLIMB_ROTOR_TO_SENSOR_RATIO);
    backClimbLigament
        .setAngle(climbBackPositionSignal.refresh().getValue().in(Degrees) / CLIMB_ROTOR_TO_SENSOR_RATIO - 180);
  }

  @Override
  public void simulationPeriodic() {
    getMotorVoltage(frontMotorSimState, frontMotorSim);
    getMotorVoltage(backMotorSimState, backMotorSim);
  }

  private static void getMotorVoltage(TalonFXSimState simState, DCMotorSim motorSim) {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    var motorVoltage = simState.getMotorVoltageMeasure();
    motorSim.setInputVoltage(motorVoltage.in(Volts));
    motorSim.update(0.020);
    simState.setRawRotorPosition(motorSim.getAngularPosition().times(CLIMB_ROTOR_TO_SENSOR_RATIO));
    simState.setRotorVelocity(motorSim.getAngularVelocity().times(CLIMB_ROTOR_TO_SENSOR_RATIO));
  }

}
