// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer; 


/** Represents a mecanum drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final CANSparkMax frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax backLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax backRightMotor = new CANSparkMax(4,MotorType.kBrushless);

  private final Encoder frontLeftEncoder = new Encoder(0, 1);
  private final Encoder frontRightEncoder = new Encoder(2, 3);
  private final Encoder backLeftEncoder = new Encoder(4, 5);
  private final Encoder backRightEncoder = new Encoder(6, 7);

  private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  private final PIDController frontLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController frontRightPIDController = new PIDController(1, 0, 0);
  private final PIDController backLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController backRightPIDController = new PIDController(1, 0, 0);

  private final AHRS gyro = new AHRS();

  private final MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  /* Here we use MecanumDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final MecanumDrivePoseEstimator m_poseEstimator =
      new MecanumDrivePoseEstimator(
          m_kinematics,
          gyro.getRotation2d(),
          getCurrentDistances(),
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /** Constructs a MecanumDrive and resets the gyro. */
  public Drivetrain() {
    gyro.reset();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);
  }

  /**
   * Returns the current state of the drivetrain.
   *
   * @return The current state of the drivetrain.
   */
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        frontLeftEncoder.getRate(),
        frontRightEncoder.getRate(),
        backLeftEncoder.getRate(),
        backRightEncoder.getRate());
  }

  /**
   * Returns the current distances measured by the drivetrain.
   *
   * @return The current distances measured by the drivetrain.
   */
  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
        frontLeftEncoder.getDistance(),
        frontRightEncoder.getDistance(),
        backLeftEncoder.getDistance(),
        backRightEncoder.getDistance());
  }

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput =
        frontLeftPIDController.calculate(
            frontLeftEncoder.getRate(), speeds.frontLeftMetersPerSecond);
    final double frontRightOutput =
        frontRightPIDController.calculate(
            frontRightEncoder.getRate(), speeds.frontRightMetersPerSecond);
    final double backLeftOutput =
        backLeftPIDController.calculate(
            backLeftEncoder.getRate(), speeds.rearLeftMetersPerSecond);
    final double backRightOutput =
        backRightPIDController.calculate(
            backRightEncoder.getRate(), speeds.rearRightMetersPerSecond);

    frontLeftMotor.setVoltage(frontLeftOutput + frontLeftFeedforward);
    frontRightMotor.setVoltage(frontRightOutput + frontRightFeedforward);
    backLeftMotor.setVoltage(backLeftOutput + backLeftFeedforward);
    backRightMotor.setVoltage(backRightOutput + backRightFeedforward);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var mecanumDriveWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_poseEstimator.getEstimatedPosition().getRotation())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(gyro.getRotation2d(), getCurrentDistances());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    m_poseEstimator.addVisionMeasurement(
        ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
            m_poseEstimator.getEstimatedPosition()),
        Timer.getFPGATimestamp() - 0.3);
  }
}
