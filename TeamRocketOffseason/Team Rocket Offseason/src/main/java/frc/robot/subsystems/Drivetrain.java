/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.TeleopDrivetrain;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Drivetrain extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private DifferentialDrive m_myRobot;
  private WPI_TalonSRX m_rightfront = new WPI_TalonSRX(4);
  private WPI_TalonSRX m_rightrear = new WPI_TalonSRX(3);
  private SpeedControllerGroup m_right = new SpeedControllerGroup(m_rightfront,  m_rightrear);
  private WPI_TalonSRX m_leftfront = new WPI_TalonSRX(2);
  private WPI_TalonSRX m_leftrear = new WPI_TalonSRX(1);
  private SpeedControllerGroup m_left = new SpeedControllerGroup(m_leftfront,  m_leftrear);
  public double speed;

  public Drivetrain() {
    super("drivetrain", 2.0, 0.0, 0.0);// The constructor passes a name for the subsystem and the P, I and D constants that are used when computing the motor output
    setAbsoluteTolerance(0.05);
    getPIDController().setContinuous(false);
    
    m_myRobot = new DifferentialDrive(m_right, m_left);

  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new TeleopDrivetrain());
  }

  public void tankDrive(double left, double right){
    m_myRobot.tankDrive(Math.pow(-left, 3),Math.pow(-right, 3)); 
    speed = left;  
  }

  @Override
  protected double returnPIDInput() {
        return 0;
    }

  @Override
  protected void usePIDOutput(double output) {

    }

}


