
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Case9 extends Command {
    
    boolean finished;
    public Case9() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(RoboProgram.autonsubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        RoboProgram.autonreset = false;
        finished = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        RoboProgram.tank.tankDrive(0, 0); //stops movement in the robot
        RoboProgram.arm.set(0);
        finished = true;
        
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
