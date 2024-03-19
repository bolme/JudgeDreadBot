// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.System.Logger;

//import org.littletonrobotics.junction.LoggedRobot;

//import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDrive;

public class Robot extends TimedRobot implements Constants {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public static XboxController m_controller = RobotContainer.m_DriverController;
  private static SwerveDrive swerve = RobotContainer.m_robotDrive;

  private long loop_counter = 0;
  private double loop_start_time = 0;
  private double loop_end_time = 0;


  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_robotContainer.periodic();
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler inst = CommandScheduler.getInstance();
    loop_counter++;
    // Time the loop using the Logger clock
    loop_start_time = RobotController.getFPGATime() / 1000000.0;
    inst.run();
    loop_end_time = RobotController.getFPGATime() / 1000000.0; // time in seconds
    
    if(loop_counter % 50 == 0){

    
      System.out.printf("Loop count: %8d    Loop time: %10.6fs\n", loop_counter, loop_end_time - loop_start_time);   
    }
    // Find the command that is using the most time    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.err.println("**** START DISABLED INIT ****");

    System.err.println("**** END DISABLED INIT ****");

  // **** START DISABLED INIT ****
  // java.lang.Throwable
	// at frc.robot.Robot.disabledInit(Robot.java:95)
	// at edu.wpi.first.wpilibj.IterativeRobotBase.loopFunc(IterativeRobotBase.java:342)
	// at edu.wpi.first.wpilibj.TimedRobot.startCompetition(TimedRobot.java:131)
	// at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:366)
	// at edu.wpi.first.wpilibj.RobotBase.startRobot(RobotBase.java:458)
	// at frc.robot.Main.main(Main.java:27)
  // **** END DISABLED INIT ***
  }
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {      
                  m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
          m_autonomousCommand.schedule();
        }
        }

  /** This function is called periodically during autonomous. */
@Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
