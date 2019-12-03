package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AutoDriveCommand extends Command {

    private double distance;
    private double angle;
    private boolean onTarget;

    public AutoDriveCommand(double distance, double angle) {
        super("AutoDriveCommand");
        requires(Robot.drivetrain);
        // drivetrain is an instance of our Drivetrain subsystem
        this.distance = distance;
        this.angle = angle;
    }

    protected void initialize() {
    }

    /*
     * execute() - In our execute method we call a tankDrive method we have created in our subsystem. This method takes two speeds as a parameter which we get from methods in the OI class.
     * These methods abstract the joystick objects so that if we want to change how we get the speed later we can do so without modifying our commands
     * (for example, if we want the joysticks to be less sensitive, we can multiply them by .5 in the getLeftSpeed method and leave our command the same).
     */
    protected void execute() {
        onTarget = Robot.drivetrain.autoDrive(distance,angle); 
    }

    protected boolean isFinished() {
        return onTarget;
    }

    protected void end() {
    }

    protected void interrupted() {
    }
}