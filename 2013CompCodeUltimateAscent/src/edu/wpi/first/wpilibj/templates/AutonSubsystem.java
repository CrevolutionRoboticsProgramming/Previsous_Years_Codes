
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.*;

public class AutonSubsystem extends Subsystem {
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new Case9());
    }
}
