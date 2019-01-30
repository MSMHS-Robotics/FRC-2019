/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
//Importing Things....



/**
 * This is a demo program showingthe use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  private DifferentialDrive m_myRobot;
  private Joystick gamepad1;
  private Joystick gamepad2;
  DoubleSolenoid exampleDouble = new DoubleSolenoid(0, 1);
  DoubleSolenoid exampleDouble2 = new DoubleSolenoid(2, 3);
  WPI_TalonSRX   collector= new WPI_TalonSRX(7);
  WPI_TalonSRX   roboarm = new WPI_TalonSRX(10);


  

  @Override
  public void robotInit() {
    m_myRobot = new DifferentialDrive(new WPI_TalonSRX(1), new WPI_TalonSRX(4));
    gamepad1 = new Joystick(0);
    gamepad2 = new Joystick(1);
    
		/* Factory default hardware to prevent unexpected behavior */
		roboarm.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		roboarm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											Constants.kPIDLoopIdx, 
											Constants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		roboarm.setSensorPhase(false);
		roboarm.setInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		roboarm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		roboarm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		roboarm.configNominalOutputForward(0, Constants.kTimeoutMs);
		roboarm.configNominalOutputReverse(0, Constants.kTimeoutMs);
		roboarm.configPeakOutputForward(1, Constants.kTimeoutMs);
		roboarm.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		roboarm.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		roboarm.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		roboarm.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		roboarm.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		roboarm.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		roboarm.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		roboarm.configMotionAcceleration(6000, Constants.kTimeoutMs);

		/* Zero the sensor */
		roboarm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(-gamepad1.getRawAxis(1), -gamepad1.getRawAxis(5));
    if (get1a()) {
      exampleDouble.set(DoubleSolenoid.Value.kForward);
    }
    if (get1b()) {
      exampleDouble.set(DoubleSolenoid.Value.kReverse);
    }
    if (!get1a() && !get1b())
    {
      exampleDouble.set(DoubleSolenoid.Value.kOff);
    }
    if (get1x()) {
      exampleDouble.set(DoubleSolenoid.Value.kForward);
    }
    if (get1y()) {
      exampleDouble.set(DoubleSolenoid.Value.kReverse);
    }
    if (!get1x() && !get1y())
    {
      exampleDouble.set(DoubleSolenoid.Value.kOff);
    }
    if (get1leftbumper())
    {
      collector.set(1);
    }
    if (get1rightbumper())
    {
      collector.set(-1);
    }
    if(!get1leftbumper() && !get1rightbumper())
    {
      collector.set(0);
    }
    if(get1lefttrigger())
    {
      roboarm.set(1);
    }
    if(get1righttrigger())
    {
      roboarm.set(-1);
    }
    if(!get1righttrigger() && !get1lefttrigger())
    {
      roboarm.set(0);
    }
    if(get2a()) {
      double targetPos = 4096 *0.50;
			roboarm.set(ControlMode.MotionMagic, targetPos);
    }
    if(get2b()) {
      double targetPos = 4096 * 0;
			roboarm.set(ControlMode.MotionMagic, targetPos);
    }
    double armEncPos = roboarm.getSelectedSensorPosition();
    double collectorPos = roboarm.getSelectedSensorPosition();
    SmartDashboard.putNumber("arm encoder",armEncPos);
    SmartDashboard.putNumber("collector encoder",collectorPos);
   



  }
  
  public double get1LeftY() {
    return (gamepad1.getRawAxis(1));
  }
  public double get1RightY() {
    return (gamepad1.getRawAxis(5));
  }
  public boolean get1rightbumper() {
    return (gamepad1.getRawButton(6)); // right bumper
  }
  public boolean get1leftbumper() {
    return (gamepad1.getRawButton(5)); // left bumper
  }
  public boolean get1lefttrigger() {
    if (gamepad1.getRawAxis(2) > 0.5) { return true; }
    return false;	
  }
  public boolean get1righttrigger() {
    if (gamepad1.getRawAxis(3) > 0.5) { return true; }
    return false;	
  }
  public boolean get1a() {
    return (gamepad1.getRawButton(2));
  }
  public boolean get1b() {
    return (gamepad1.getRawButton(3));
  }
  public boolean get1x() {
    return (gamepad1.getRawButton(1));
  }
  public boolean get1y() {
    return (gamepad1.getRawButton(4));
  }

  public double get2LeftY() {
    return (gamepad2.getRawAxis(1));
  }
  public double get2RightY() {
    return (gamepad2.getRawAxis(5));
  }
  public boolean get2rightbumper() {
    return (gamepad2.getRawButton(6)); // right bumper
  }
  public boolean get2leftbumper() {
    return (gamepad1.getRawButton(5)); // left bumper
  }
  public boolean get2lefttrigger() {
    if (gamepad2.getRawAxis(2) > 0.5) { return true; }
    return false;	
  }
  public boolean get2righttrigger() {
    if (gamepad2.getRawAxis(3) > 0.5) { return true; }
    return false;	
  }
  public boolean get2a() {
    return (gamepad2.getRawButton(2));
  }
  public boolean get2b() {
    return (gamepad2.getRawButton(3));
  }
  public boolean get2x() {
    return (gamepad2.getRawButton(1));
  }
  public boolean get2y() {
    return (gamepad2.getRawButton(4));
  }
}
