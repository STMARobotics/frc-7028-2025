package frc.robot;

import static frc.robot.Constants.CANIVORE_BUS_NAME;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainTelemetry {
  // Limit telemetry updates to prevent flooding network and Shuffleboard
  private static final double PUBLISH_FREQUENCY = 0.04;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot speeds for general checking */
  private final NetworkTable driveStats = inst.getTable("Drive");

  private final StructPublisher<Pose2d> odometryPublisher = driveStats.getStructTopic("odometry", Pose2d.struct)
      .publish();
  private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher = driveStats
      .getStructArrayTopic("Module States", SwerveModuleState.struct)
      .publish();
  private final StructArrayPublisher<SwerveModuleState> moduleTargetsPublisher = driveStats
      .getStructArrayTopic("Module Targets", SwerveModuleState.struct)
      .publish();
  private final DoublePublisher periodPublisher = driveStats.getDoubleTopic("Period").publish();

  private final DoublePublisher xSpeedPublisher = driveStats.getDoubleTopic("xSpeed").publish();
  private final DoublePublisher ySpeedPublisher = driveStats.getDoubleTopic("ySpeed").publish();

  private final BooleanPublisher canBusStatusPublisher = driveStats.getBooleanTopic("canivore").publish();

  private final Field2d field2d = new Field2d();

  private final Timer frequencyTimer = new Timer();

  private final CANBus canBus = new CANBus(CANIVORE_BUS_NAME);

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   */
  public DrivetrainTelemetry() {
    frequencyTimer.start();
    SmartDashboard.putData(field2d);
  }

  /* Accept the swerve drive state and telemeterize it to NetworkTables */
  public void telemeterize(SwerveDriveState state) {
    if (frequencyTimer.advanceIfElapsed(PUBLISH_FREQUENCY)) {
      /* Telemeterize the pose */
      odometryPublisher.set(state.Pose, (long) (Timer.getFPGATimestamp() * 1000000));

      // Publish module states and targets
      moduleStatePublisher.set(state.ModuleStates);
      moduleTargetsPublisher.set(state.ModuleTargets);
      periodPublisher.accept(state.OdometryPeriod);
      field2d.setRobotPose(state.Pose);
      xSpeedPublisher.set(state.Speeds.vxMetersPerSecond);
      ySpeedPublisher.set(state.Speeds.vyMetersPerSecond);

      // Canbus status
      canBusStatusPublisher.set(canBus.getStatus().Status == StatusCode.OK);
    }
  }

}
