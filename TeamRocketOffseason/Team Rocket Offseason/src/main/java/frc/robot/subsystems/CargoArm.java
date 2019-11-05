/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class CargoArm extends PIDSubsystem {
  WPI_TalonSRX   roboarm = new WPI_TalonSRX(10); 
  
                      if (resetArm && armLimit.get()) {
                        roboarm.set(ControlMode.PercentOutput, 0.4);
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
                      if (gamepad2.getPOV() == 0 && Math.abs(yvalue2) > 0.1) {
                        roboarm.set(ControlMode.PercentOutput, yvalue2);    
                                 //roboarm.set(1);
    // roboarm.set(-1);
     //roboarm.set(0); 
      //roboarm.set(ControlMode.MotionMagic, targetPos);
      resetArm = true;
    }
    if (get2b()) {
      resetArm = false;
   //reset arm
   if (resetArm && armLimit.get()) {
    roboarm.set(ControlMode.PercentOutput, 0.4);
  } else {
    //if not reseting, hold at exact position
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
  //      roboarm.set(ControlMode.PercentOutput, yvalue2);                                                                                         
  public CargoArm(){
    super("CargoArm",1.0,0.0,0.0);

    roboarm.configFactoryDefault();
	roboarm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											Constants.kPIDLoopIdx, 
                      Constants.kTimeoutMs);
                      roboarm.setSensorPhase(false);
                      roboarm.setInverted(true);                   
        
                      roboarm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
                      roboarm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
                      roboarm.configNominalOutputForward(0, Constants.kTimeoutMs);
                      roboarm.configNominalOutputReverse(0, Constants.kTimeoutMs);
                      roboarm.configPeakOutputForward(1, Constants.kTimeoutMs);
                      roboarm.configPeakOutputReverse(-1, Constants.kTimeoutMs);
                      roboarm.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
                      roboarm.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
                      roboarm.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
                      roboarm.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
                      roboarm.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
                      roboarm.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
                      roboarm.configMotionAcceleration(6000, Constants.kTimeoutMs);
                      roboarm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
                      roboarm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  public void initDefaultCommand() {
  }

  public void manualControl(double power){
    roboarm.set(ControlMode.PercentOutput, power);
  }

    
  

  @Override
  protected double returnPIDInput() {
    return 0;
  }

  @Override
  protected void usePIDOutput(double output) {

  }
}

