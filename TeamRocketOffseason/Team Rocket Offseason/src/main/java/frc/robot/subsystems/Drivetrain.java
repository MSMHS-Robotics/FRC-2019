/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.TeleopDrivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  PIDOutput driveOutput;
  PIDOutput turnOutput;
  PIDController turnController;
  AHRS ahrs;
  private WPI_TalonSRX right1 = new WPI_TalonSRX(4);
  private WPI_TalonSRX right2 = new WPI_TalonSRX(3);
  private WPI_TalonSRX left1 = new WPI_TalonSRX(2);
  private WPI_TalonSRX left2 = new WPI_TalonSRX(1);
  private DifferentialDrive drivetrain = new DifferentialDrive(right1, left1);
  public double speed;


  private double turnPower;

  public Drivetrain() {
    right2.follow(right1);
    left2.follow(left1);    

    driveOutput = new PIDOutput(){
    
      @Override
      public void pidWrite(double output) {
        
      }
    };

    turnOutput = new PIDOutput(){
    
      @Override
      public void pidWrite(double output) {
        turnPower = output;
      }
    };

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    turnController = new PIDController(1, 0, 0, 0, ahrs, turnOutput); //change this later
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(2); //change this later
    turnController.setContinuous(true);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new TeleopDrivetrain());
  }

  public void turnToAngle(double angle){

  }

  public void driveDistance(double distance){

  }

  public boolean autoDrive(double distance, double angle){
    turnController.setSetpoint(angle);

    drivetrain.arcadeDrive(0, turnPower);
    return turnController.onTarget();
  }

  public void tankDrive(double left, double right){
    drivetrain.tankDrive(Math.pow(-left, 3),Math.pow(-right, 3)); 
    speed = left;  
  }

 
}


