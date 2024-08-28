// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.sensors.NavX;

/**
 * The SwerveDrive class represents a subsystem for controlling a swerve drive
 * system.
 * It provides methods for driving the robot, updating odometry, and retrieving
 * the robot's pose.
 * 
 * This class follows the singleton design pattern, ensuring that only one
 * instance of SwerveDrive can exist.
 * The getInstance() method is used to retrieve the instance of SwerveDrive.
 * 
 * The SwerveDrive class relies on the following components:
 * - SwerveModule: Represents a single swerve module with its own motor and
 * encoder.
 * - ChassisSpeeds: Represents the desired speeds and rotation of the robot.
 * - SwerveDriveKinematics: Calculates the module states based on the desired
 * speeds and rotation.
 * - SwerveDrivePoseEstimator: Estimates the robot's pose based on odometry
 * readings and module positions.
 * - NavX: Provides the robot's rotation information.
 * - PIDController: Controls the rotation of the robot to a desired angle.
 * 
 * The SwerveDrive class also interacts with WPILib's NetworkTables to publish
 * actual and set module states, as well as odometry information.
 * 
 * Usage:
 * - Call SwerveDrive.getInstance() to retrieve the instance of SwerveDrive.
 * - Use the drive() method to drive the robot with given speeds and rotation.
 * - Use the arcadeDrive() method to drive the robot using arcade drive control.
 * - Call resetPose() to reset the robot's pose to a given position.
 * - Call updateOdometry() periodically to update the robot's odometry based on
 * sensor readings.
 * - Use the provided getter methods to retrieve information about the robot's
 * pose, module positions, and more.
 * 
 * Note: The numbers used in the constructor and other methods are
 * robot-specific and should be tuned accordingly.
 */
public class SwerveDrive extends SubsystemBase {

    private static SwerveDrive instance = null;

    /**
     * Returns the singleton instance of the SwerveDrive subsystem.
     *
     * @return the singleton instance of the SwerveDrive subsystem
     */
    public static synchronized SwerveDrive getInstance() {
        if (instance == null) {
            instance = new SwerveDrive();
        }
        return instance;
    }

    private final SwerveModule[] modules;

    private final Translation2d[] locations_deleteme = { // positive x is the front of the robot, positive y is the left of the
                                                // robot
            new Translation2d(Constants.wheelsLength / 2.0, Constants.wheelsWidth / 2.0),
            new Translation2d(Constants.wheelsLength / 2.0, -Constants.wheelsWidth / 2.0),
            new Translation2d(-Constants.wheelsLength / 2.0, Constants.wheelsWidth / 2.0),
            new Translation2d(-Constants.wheelsLength / 2.0, -Constants.wheelsWidth / 2.0)
    };

    private final SwerveDriveKinematics kinematics;

    /*
     * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
     * The numbers used
     * below are robot specific, and should be tuned.
     */
    private final SwerveDrivePoseEstimator poseEstimator;

    // WPILib
    StructArrayPublisher<SwerveModuleState> actualStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Actual States", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> setStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Set States", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> odometryStruct = NetworkTableInstance.getDefault()
            .getStructTopic("Odometry", Pose2d.struct).publish();
    SwerveModuleState[] states = new SwerveModuleState[4];

    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

    PIDController turnPID = new PIDController(.5, 0.0, 0);

    /**
     * Constructs a new instance of the SwerveDrive subsystem.
     */
    private SwerveDrive() {

        // fill in from constants
        modules = new SwerveModule[4];

        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModule(
                    Constants.swerve_config.modules[i].label,
                    Constants.swerve_config.modules[i].cancoderId,
                    Constants.swerve_config.modules[i].driveMotorCanId,
                    Constants.swerve_config.modules[i].turnMotorCanId,
                    Constants.swerve_config.modules[i].turnEncoderAngleOffset,
                    Constants.swerve_config.modules[i].xLocationOffset,
                    Constants.swerve_config.modules[i].yLocationOffset);
        }


        kinematics = new SwerveDriveKinematics(
            modules[0].getOffsetLocation(),
            modules[1].getOffsetLocation(),
            modules[2].getOffsetLocation(),
            modules[3].getOffsetLocation());

        poseEstimator = new SwerveDrivePoseEstimator(
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

        NavX.getInstance();
    }

    /**
     * Updates the periodic state of the SwerveDrive subsystem.
     * Retrieves the state of each module and updates the odometry, actual states,
     * set states, and odometry struct.
     */
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
     * Drives the swerve drive system with the given speeds and rotation.
     * 
     * @param xSpeed        The speed in the x-axis direction.
     * @param ySpeed        The speed in the y-axis direction.
     * @param rot           The rotation speed.
     * @param fieldRelative Whether the speeds are field-relative or robot-relative.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds botSpeeds;

        if (fieldRelative) {
            botSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, NavX.getInstance().getRotation2d());
        } else {
            botSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        // ChassisSpeeds.discretize(botSpeeds, 0.2, 0.2, 0.2);

        swerveModuleStates = kinematics.toSwerveModuleStates(botSpeeds);

        // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

        for (int i = 0; i < 4; i++) {
            modules[i].setStates(swerveModuleStates[i], false);
        }
    }

    /**
     * Drives the robot using arcade drive control.
     * 
     * @param xSpeed the speed at which the robot should move forward or backward
     * @param rot    the speed at which the robot should rotate
     */
    public void arcadeDrive(double xSpeed, double rot) {
        drive(xSpeed, 0.0, rot, false);
    }

    /**
     * Resets the pose of the swerve drive to the specified pose.
     * 
     * @param pose The new pose to set for the swerve drive.
     */
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(NavX.getInstance().getRotation2d(), getModulePositions(), pose);
    }

    /**
     * Updates the odometry of the swerve drive system.
     * 
     * This method updates the pose estimator of the swerve drive system using the
     * current rotation
     * from the NavX sensor and the positions of the four swerve modules. It
     * calculates and updates
     * the current position and orientation of the robot on the field.
     */
    public void updateOdometry() {
        poseEstimator.update(
                NavX.getInstance().getRotation2d(),
                new SwerveModulePosition[] {
                        modules[0].getSwerveModulePosition(),
                        modules[1].getSwerveModulePosition(),
                        modules[2].getSwerveModulePosition(),
                        modules[3].getSwerveModulePosition()
                });
    }

    /**
     * Retrieves the positions of all swerve modules in the swerve drive.
     *
     * @return an array of SwerveModulePosition objects representing the positions
     *         of the swerve modules
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getSwerveModulePosition();
        }
        return positions;
    }

    /**
     * Returns the current pose of the robot.
     *
     * @return The current pose of the robot.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

}