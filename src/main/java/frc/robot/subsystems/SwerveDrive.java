// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import org.littletonrobotics.junction.Logger;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.sensors.Camera;
import frc.robot.sensors.NavX;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends SubsystemBase implements Constants {
  private static SwerveDrive instance = null;

  private final Translation2d[] locations = {
      new Translation2d(botLength, botWidth),
      new Translation2d(botLength, -botWidth),
      new Translation2d(-botLength, botWidth),
      new Translation2d(-botLength, -botWidth)
  };

  SwerveModule[] modules = {
      new SwerveModule("frontRight", 10, 1, 2, -49),
      new SwerveModule("backRight", 11, 5, 6, -40),
      new SwerveModule("frontLeft", 13, 3, 4, 128),
      new SwerveModule("backLeft", 12, 7, 8, 115),
  };

  private ChassisSpeeds botSpeeds = new ChassisSpeeds(0, 0, 0);
  private boolean pathInverted = false;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      locations[0], locations[1], locations[2], locations[3]);

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      NavX.getInstance().getRotation2d(),
      new SwerveModulePosition[] {
          modules[0].getSwerveModulePosition(),
          modules[1].getSwerveModulePosition(),
          modules[2].getSwerveModulePosition(),
          modules[3].getSwerveModulePosition()
      },
      new Pose2d(),

      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  private SwerveDrive() {
    NavX.getInstance().reset();

  }

  // WPILib
  StructArrayPublisher<SwerveModuleState> actualStates = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Actual States", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> setStates = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Set States", SwerveModuleState.struct).publish();
  StructPublisher<Pose2d> odometryStruct = NetworkTableInstance.getDefault()
      .getStructTopic("Odometry", Pose2d.struct).publish();
  SwerveModuleState[] states = new SwerveModuleState[4];

  public void periodic() {
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    updateOdometry();
    actualStates.set(swerveModuleStates);
    setStates.set(states);
    odometryStruct.set(poseEstimator.getEstimatedPosition());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

  /**
   * Drives the swerve drive system with the given speeds and rotation.
   * 
   * @param xSpeed         The speed in the x-axis direction.
   * @param ySpeed         The speed in the y-axis direction.
   * @param rot            The rotation speed.
   * @param fieldRelative  Whether the speeds are field-relative or robot-relative.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    botSpeeds = ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rot), .02);
    swerveModuleStates = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, NavX.getInstance().getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            .02));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

    for (int i = 0; i < 4; i++) {
      modules[i].setStates(swerveModuleStates[i], false);
    }
  }

  /**
   * Drives the robot using arcade drive control.
   * 
   * @param xSpeed the speed at which the robot should move forward or backward
   * @param rot the speed at which the robot should rotate
   */
  public void arcadeDrive(double xSpeed, double rot) {
    drive(xSpeed, 0.0, rot, false);
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(NavX.getInstance().getRotation2d(), getModulePositions(), pose);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  public void setPathInverted(Boolean inverted) {
    pathInverted = inverted;
  }

  public static synchronized SwerveDrive getInstance() {
    if (instance == null) {
      instance = new SwerveDrive();
    }
    return instance;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    poseEstimator.update(
        NavX.getInstance().getRotation2d(),
        new SwerveModulePosition[] {
            modules[0].getSwerveModulePosition(),
            modules[1].getSwerveModulePosition(),
            modules[2].getSwerveModulePosition(),
            modules[3].getSwerveModulePosition()
        });

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    // poseEstimator.addVisionMeasurement(
    // ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
    // poseEstimator.getEstimatedPosition()),
    // Timer.getFPGATimestamp() - 0.3);
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getSwerveModulePosition();
    }
    return positions;
  }

  public boolean shouldFlipPath() {
    return pathInverted;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return botSpeeds;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public double getDistanceFromSpeaker() {
    return Math.hypot(SwerveDrive.getInstance().getPose().getX() - .37,
        SwerveDrive.getInstance().getPose().getY() - (216 * .0254));
  }

  PIDController turnPID = new PIDController(.5, 0.0, 0);

  public double turnToAprilTag(int ID) {
    // turnPID.enableContinuousInput(0, 360);
    double botAngle = getPose().getRotation().getDegrees();
    double offsetAngle = 0;// TODO: camera.getDegToApriltag(ID);
    double setpoint = 0;
    if (botAngle - offsetAngle <= 0)
      setpoint = botAngle + offsetAngle;
    else
      setpoint = botAngle - offsetAngle;

    turnPID.setSetpoint(setpoint);
    return turnPID.calculate(botAngle);
  }
}