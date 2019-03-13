/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;

import edu.wpi.first.wpilibj.DigitalInput;

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
  SpeedController   collector= new Spark(8);
  SpeedController   hatchHook= new Spark(0);
  WPI_TalonSRX   hatcharm= new WPI_TalonSRX(7);
  WPI_TalonSRX   roboarm = new WPI_TalonSRX(10);
  DigitalInput armLimit = new DigitalInput(0);
  DigitalInput hatchLimit = new DigitalInput(1);

  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;  
  //boolean prevTrigger = false;

  boolean forwardIsBackwards = false;

  @Override
  public void robotInit() {
    WPI_TalonSRX m_rightfront = new WPI_TalonSRX(4);
    WPI_TalonSRX m_rightrear = new WPI_TalonSRX(3);
    SpeedControllerGroup m_right = new SpeedControllerGroup(m_rightfront,  m_rightrear);
    WPI_TalonSRX m_leftfront = new WPI_TalonSRX(2);
    WPI_TalonSRX m_leftrear = new WPI_TalonSRX(1);
    SpeedControllerGroup m_left = new SpeedControllerGroup(m_leftfront,  m_leftrear);
    m_myRobot = new DifferentialDrive(m_right, m_left);
    gamepad1 = new Joystick(0);
    gamepad2 = new Joystick(1);
    
		/* Factory default hardware to prevent unexpected behavior */
    roboarm.configFactoryDefault();
    hatcharm.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		roboarm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											Constants.kPIDLoopIdx, 
                      Constants.kTimeoutMs);
    hatcharm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											Constants.kPIDLoopIdx, 
											Constants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		roboarm.setSensorPhase(false);
    roboarm.setInverted(true);
    hatcharm.setSensorPhase(false);
		hatcharm.setInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		roboarm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    roboarm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    hatcharm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		hatcharm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		roboarm.configNominalOutputForward(0, Constants.kTimeoutMs);
		roboarm.configNominalOutputReverse(0, Constants.kTimeoutMs);
		roboarm.configPeakOutputForward(1, Constants.kTimeoutMs);
    roboarm.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    hatcharm.configNominalOutputForward(0, Constants.kTimeoutMs);
		hatcharm.configNominalOutputReverse(0, Constants.kTimeoutMs);
		hatcharm.configPeakOutputForward(1, Constants.kTimeoutMs);
		hatcharm.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		roboarm.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		roboarm.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		roboarm.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		roboarm.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    roboarm.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    hatcharm.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		hatcharm.config_kF(Constants.kSlotIdx, Constants.hatchGains.kF, Constants.kTimeoutMs);
		hatcharm.config_kP(Constants.kSlotIdx, Constants.hatchGains.kP, Constants.kTimeoutMs);
		hatcharm.config_kI(Constants.kSlotIdx, Constants.hatchGains.kI, Constants.kTimeoutMs);
		hatcharm.config_kD(Constants.kSlotIdx, Constants.hatchGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		roboarm.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    roboarm.configMotionAcceleration(6000, Constants.kTimeoutMs);
    hatcharm.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		hatcharm.configMotionAcceleration(6000, Constants.kTimeoutMs);


		/* Zero the sensor */
    roboarm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    hatcharm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
/*
    UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(); //front or back camera?
    camera1.setResolution(320, 240);
    camera1.setFPS(20); //cannot go higher or it uses too much bandwidth!
*/


    camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera1.setResolution(320, 240);
    camera1.setFPS(20); //cannot go higher or it uses too much bandwidth!

    camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    camera2.setResolution(320, 240);
    camera2.setFPS(20); //cannot go higher or it uses too much bandwidth!
 
    server = CameraServer.getInstance().addSwitchedCamera("Switched Camera");
    camera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

  
  }
  @Override
  public void autonomousInit() {
    		/* Zero the sensor */
        roboarm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        hatcharm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  double targetPosArm = 0;
  double targetPosHatch = 0;
  boolean resetArm = false;
  boolean resetHatch =false;

  @Override

  public void autonomousPeriodic() {
    if (forwardIsBackwards) {
      m_myRobot.tankDrive( gamepad1.getRawAxis(5)*gamepad1.getRawAxis(5)*gamepad1.getRawAxis(5), gamepad1.getRawAxis(1)*gamepad1.getRawAxis(1)*gamepad1.getRawAxis(1));
    } else {
      m_myRobot.tankDrive(-gamepad1.getRawAxis(1)*gamepad1.getRawAxis(1)*gamepad1.getRawAxis(1), -gamepad1.getRawAxis(5)*gamepad1.getRawAxis(5)*gamepad1.getRawAxis(5));
    }

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
      hatchHook.set(1);
    }
    if (get1y()) {
     hatchHook.set(-1);
    }
    if (!get1x() && !get1y())
    {
     hatchHook.set(0);
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
      targetPosHatch = 0;
      //roboarm.set(1);
    }
    if(get1righttrigger())
    {
      targetPosHatch = 4096. *0.250;
     // roboarm.set(-1);
    }
    if(!get1righttrigger() && !get1lefttrigger())
    {
      //roboarm.set(0);
    }
    if(get2x()) {
      resetHatch = true;

    }
    if (get2a()) {
      resetHatch = false;
    }
    if(get2y()) {
      // targetPosArm = 4096. * 0;
      //roboarm.set(ControlMode.MotionMagic, targetPos);
      resetArm = true;
    }
    if (get2b()) {
      resetArm = false;
    }
    if(gamepad1.getPOV() == 270) {
      targetPosArm = -4096. *0.0;
    } else if (gamepad1.getPOV() == 0){ 
      targetPosArm = -4096. *0.20;
    }else if (gamepad1.getPOV()==45){
      targetPosArm = -4096. *0.4;
    }else if (gamepad1.getPOV() == 90){ 
      targetPosArm = -4096. * 0.8;
    }  else if (gamepad1.getPOV() == 180){ 
      // NOTE: this is hatch not arm
      targetPosHatch = 3600; //4096. *1.4;
    }
    if (resetArm && armLimit.get()) {
      roboarm.set(ControlMode.PercentOutput, 0.2);
    } else {
      if (!resetArm) {
        roboarm.set(ControlMode.MotionMagic, targetPosArm);
      }
    }
    if ( !armLimit.get()) {
      resetArm = false;
      targetPosArm = 0;
      roboarm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }

    double armEncPos = roboarm.getSelectedSensorPosition();
    double collectorPos = hatcharm.getSelectedSensorPosition();
    SmartDashboard.putNumber("arm encoder",armEncPos);
    SmartDashboard.putNumber("hatch encoder",collectorPos);
    SmartDashboard.putBoolean("reseting hatch", resetHatch);
    SmartDashboard.putBoolean("reseting arm", resetArm);
    SmartDashboard.putBoolean("forward is Backwards",forwardIsBackwards);
    SmartDashboard.putBoolean("hatch limit", !hatchLimit.get());
    SmartDashboard.putBoolean("arm limit", !armLimit.get());
    SmartDashboard.putNumber("DPAD position", gamepad1.getPOV());


    if (resetHatch && hatchLimit.get()) {
      hatcharm.set(ControlMode.PercentOutput, -0.4);
    } else {
      if (!resetHatch) {
        hatcharm.set(ControlMode.MotionMagic, targetPosHatch); 
      }
    }

    if (!hatchLimit.get()) {
      resetHatch = false;
      targetPosHatch = 0;
      hatcharm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }

    // camera switching
    
    if (get2leftbumper()) {
      System.out.println("Setting camera 2");
      server.setSource(camera2);
    }
    if (get2rightbumper()) {
      System.out.println("Setting camera 1");
      server.setSource(camera1);
    }
    // driver direction switching
    if (get2lefttrigger()) {
      forwardIsBackwards = false;
    }
    if (get2righttrigger()) {
      forwardIsBackwards = true;
    }


  }

  public void teleopPeriodic() {
    if (forwardIsBackwards) {
      m_myRobot.tankDrive( gamepad1.getRawAxis(5)*gamepad1.getRawAxis(5)*gamepad1.getRawAxis(5), gamepad1.getRawAxis(1)*gamepad1.getRawAxis(1)*gamepad1.getRawAxis(1));
    } else {
      m_myRobot.tankDrive(-gamepad1.getRawAxis(1)*gamepad1.getRawAxis(1)*gamepad1.getRawAxis(1), -gamepad1.getRawAxis(5)*gamepad1.getRawAxis(5)*gamepad1.getRawAxis(5));
    }

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
      hatchHook.set(1);
    }
    if (get1y()) {
     hatchHook.set(-1);
    }
    if (!get1x() && !get1y())
    {
     hatchHook.set(0);
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
      targetPosHatch = 0;
      //roboarm.set(1);
    }
    if(get1righttrigger())
    {
      targetPosHatch = 4096. *0.250;
     // roboarm.set(-1);
    }
    if(!get1righttrigger() && !get1lefttrigger())
    {
      //roboarm.set(0);
    }
    if(get2x()) {
      resetHatch = true;

    }
    if (get2a()) {
      resetHatch = false;
    }
    if(get2y()) {
      // targetPosArm = 4096. * 0;
      //roboarm.set(ControlMode.MotionMagic, targetPos);
      resetArm = true;
    }
    if (get2b()) {
      resetArm = false;
    }
    if(gamepad1.getPOV() == 270) {
      targetPosArm = -4096. *0.0;
    } else if (gamepad1.getPOV() == 0){ 
      targetPosArm = -4096. *0.20;
    } else if (gamepad1.getPOV() == 90){ 
      targetPosArm = -4096. * 0.8;
    }  else if (gamepad1.getPOV() == 180){ 
      // NOTE: this is hatch not arm
      targetPosHatch = 4000; //4096. *1.4;
    }
    if (resetArm && armLimit.get()) {
      roboarm.set(ControlMode.PercentOutput, 0.2);
    } else {
      if (!resetArm) {
        roboarm.set(ControlMode.MotionMagic, targetPosArm);
      }
    }
    if ( !armLimit.get()) {
      resetArm = false;
      targetPosArm = 0;
      roboarm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }

    double armEncPos = roboarm.getSelectedSensorPosition();
    double collectorPos = hatcharm.getSelectedSensorPosition();
    SmartDashboard.putNumber("arm encoder",armEncPos);
    SmartDashboard.putNumber("hatch encoder",collectorPos);
    SmartDashboard.putBoolean("reseting hatch", resetHatch);
    SmartDashboard.putBoolean("reseting arm", resetArm);
    SmartDashboard.putBoolean("forward is Backwards",forwardIsBackwards);
    SmartDashboard.putBoolean("hatch limit", !hatchLimit.get());
    SmartDashboard.putBoolean("arm limit", !armLimit.get());
    SmartDashboard.putNumber("DPAD position", gamepad1.getPOV());


    if (resetHatch && hatchLimit.get()) {
      hatcharm.set(ControlMode.PercentOutput, -0.4);
    } else {
      if (!resetHatch) {
        hatcharm.set(ControlMode.MotionMagic, targetPosHatch); 
      }
    }

    if (!hatchLimit.get()) {
      resetHatch = false;
      targetPosHatch = 0;
      hatcharm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }


    // camera switching
    
    if (get2leftbumper()) {
      System.out.println("Setting camera 2");
      server.setSource(camera2);
    }
    if (get2rightbumper()) {
      System.out.println("Setting camera 1");
      server.setSource(camera1);
    }
    // driver direction switching
    if (get2lefttrigger()) {
      forwardIsBackwards = false;
    }
    if (get2righttrigger()) {
      forwardIsBackwards = true;
    }

    // Hab 2 override
    double yvalue = get2RightY();
    if (gamepad2.getPOV() == 180 && Math.abs(yvalue) > 0.1) {
      hatcharm.set(ControlMode.PercentOutput, yvalue);
    }

    double yvalue2 = get2RightY();
    if (gamepad2.getPOV() == 0 && Math.abs(yvalue2) > 0.1) {
      roboarm.set(ControlMode.PercentOutput, yvalue2);
    }

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
    return (gamepad1.getRawButton(1));
  }
  public boolean get1b() {
    return (gamepad1.getRawButton(2));
  }
  public boolean get1x() {
    return (gamepad1.getRawButton(3));
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
    return (gamepad2.getRawButton(5)); // left bumper
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
    return (gamepad2.getRawButton(1));
  }
  public boolean get2b() {
    return (gamepad2.getRawButton(2));
  }
  public boolean get2x() {
    return (gamepad2.getRawButton(3));
  }
  public boolean get2y() {
    return (gamepad2.getRawButton(4));
  }
}
