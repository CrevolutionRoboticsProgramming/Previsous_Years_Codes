
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.*;

public class Testauton extends Command {
    
    boolean finished;
    public Testauton() {
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
        if (!finished)
        {
            RoboProgram.tank.tankDrive(0.25, 0.25);
            Timer.delay(1);
            RoboProgram.tank.tankDrive(0, 0);
            Timer.delay(1);
            RoboProgram.tank.tankDrive(-0.25, -0.25);
            Timer.delay(1);
            RoboProgram.tank.tankDrive(0, 0);
            Timer.delay(1);
            finished = true;
        }
        else
        {
            RoboProgram.tank.tankDrive(0, 0);
            RoboProgram.arm.set(0);
        }
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
