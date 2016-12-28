/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.*;

/**
 *
 * @author Crevolution
 */
public class disc3insideback extends Command {
    
    boolean finished = false;
    
    public disc3insideback() {
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
        RoboProgram.shooter.set(1);
        RoboProgram.LED.set(Relay.Value.kForward);
        RoboProgram.angler.set(DoubleSolenoid.Value.kReverse);
        Timer.delay(0.5);
        RoboProgram.indexer.set(DoubleSolenoid.Value.kReverse);
        Timer.delay(0.5);
        RoboProgram.indexer.set(DoubleSolenoid.Value.kForward);
        Timer.delay(0.25);
        RoboProgram.indexer.set(DoubleSolenoid.Value.kReverse);
        Timer.delay(0.5);
        RoboProgram.indexer.set(DoubleSolenoid.Value.kForward);
        Timer.delay(0.25);
        RoboProgram.indexer.set(DoubleSolenoid.Value.kReverse);
        Timer.delay(0.5);
        RoboProgram.indexer.set(DoubleSolenoid.Value.kForward);
        Timer.delay(0.25);
        RoboProgram.indexer.set(DoubleSolenoid.Value.kReverse);
        RoboProgram.shooter.set(0);
        RoboProgram.LED.set(Relay.Value.kOff);
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
