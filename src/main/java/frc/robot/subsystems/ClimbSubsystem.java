package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ClimbConstants.CLIMB_MAGNETIC_OFFSET_FRONT;
import static frc.robot.Constants.ClimbConstants.CLIMB_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.ClimbConstants.CLIMB_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_CANDI;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_MOTOR;
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

  private final TalonFX climbMotor = new TalonFX(DEVICE_ID_CLIMB_MOTOR, CANIVORE_BUS_NAME);
  private final CANdi climbCanDi = new CANdi(DEVICE_ID_CLIMB_CANDI, CANIVORE_BUS_NAME);

  private final VoltageOut climbControl = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> climbVelocitySignal = climbMotor.getVelocity();
  private final StatusSignal<Angle> climbPositionSignal = climbMotor.getPosition();

  private final Mechanism2d climbMechanisms = new Mechanism2d(12, 5);
  private final MechanismRoot2d climbRoot = climbMechanisms.getRoot("Climb", 3, 3);
  private final MechanismLigament2d climbLigament = climbRoot
      .append(new MechanismLigament2d("Climb Lever", 2, 0, 3, new Color8Bit(Color.kYellow)));

  private final DCMotorSim climbMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, CLIMB_ROTOR_TO_SENSOR_RATIO),
      DCMotor.getKrakenX60Foc(1));

  private final TalonFXSimState climbMotorSimState = climbMotor.getSimState();

  public ClimbSubsystem() {
    var climbCANCoderConfig = new CANdiConfiguration();
    climbCANCoderConfig.PWM1.withAbsoluteSensorOffset(CLIMB_MAGNETIC_OFFSET_FRONT)
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

    climbMotor.getConfigurator().apply(climbTalonConfig);

    SmartDashboard.putData("Climb", climbMechanisms);
  }

  /**
   * Runs the climb motor
   * 
   * @param output percentage of full output to run the motor. Range [-1, 1]
   */
  public void runClimb(double output) {
    climbMotor.setControl(climbControl.withOutput(MAX_CLIMB_VOLTAGE.in(Volts) * MathUtil.clamp(output, -1, 1)));
  }

  public void stop() {
    climbMotor.stopMotor();
  }

  /**
   * Method to determine if the climb motors are moving
   * 
   * @return true if the motor has reached one rotation per second
   */
  public boolean isClimbMotorMoving() {
    return (climbVelocitySignal.refresh().getValue().in(RotationsPerSecond) > 1);
  }

  @Override
  public void periodic() {
    climbLigament.setAngle(-climbPositionSignal.refresh().getValue().in(Degrees) / CLIMB_ROTOR_TO_SENSOR_RATIO);
  }

  @Override
  public void simulationPeriodic() {
    climbMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    var motorVoltage = climbMotorSimState.getMotorVoltageMeasure();
    climbMotorSim.setInputVoltage(motorVoltage.in(Volts));
    climbMotorSim.update(0.020);
    climbMotorSimState.setRawRotorPosition(climbMotorSim.getAngularPosition().times(CLIMB_ROTOR_TO_SENSOR_RATIO));
    climbMotorSimState.setRotorVelocity(climbMotorSim.getAngularVelocity().times(CLIMB_ROTOR_TO_SENSOR_RATIO));
  }

}
