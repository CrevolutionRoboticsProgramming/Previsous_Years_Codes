
package org.usfirst.frc.team2851.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static Joystick   drivercontroller, operatorcontroller;
	public static RobotDrive first_person_drive, intake;
	public static CANTalon leftwheel, rightwheel, liftmotor1, liftmotor2, leftintake, rightintake, rollers;
	public static Compressor compressor;
	public static DigitalInput top_switch, bottom_switch;
	public static DoubleSolenoid left_hook, right_hook;;
	public static AnalogPotentiometer pot;
	public static Encoder left_encoder, right_encoder;
	
	double kp = 0.01;
	int autonchooser = 6;
	
	boolean which_way = false;
	boolean override_lift = false;
	
	int autoncounter = 0;
	int autoncounter1 = 0;
	
	// c = container t = tote f = floor
	double t_on_f = 0.0;
	double t_on_t = 0.0;
	double t_on_2t = 0.0;
	double t_on_3t = 0.0;
	double t_on_4t = 0.0;
	
	double c_on_f = 0.0;
	double c_on_t = 0.0;
	double c_on_2t = 0.0;
	double c_on_3t = 0.0;
	double c_on_4t = 0.0;
	
	
	
	
	
    public void robotInit() {
    	
    	leftwheel = new CANTalon(2);
    	rightwheel = new CANTalon(4);
      	liftmotor1 = new CANTalon(5);
      	liftmotor2 = new CANTalon(1);
      	leftintake = new CANTalon(7);
      	rightintake = new CANTalon(6);
      	rollers = new CANTalon(3);
      	
      	left_encoder = new Encoder(0, 1);
 
    	
    	first_person_drive = new RobotDrive(leftwheel, rightwheel);
    	drivercontroller = new Joystick(0);
    	operatorcontroller = new Joystick(1);
    //	intake = new RobotDrive(leftintake, rightintake);
    	compressor = new Compressor(0);
    	
    	left_hook = new DoubleSolenoid(5, 4);
    	right_hook = new DoubleSolenoid(3, 2);
    	
    	top_switch = new DigitalInput(2);
    	bottom_switch  = new DigitalInput(3);
    	
    	pot = new AnalogPotentiometer(1);
    	
    	first_person_drive.setExpiration(0.1);
    	
    	compressor.start();

    }

    public static void liftmotors(double s)
    {
    	liftmotor1.set(s);
    	liftmotor2.set(s);
    }

    public void autonomousInit()
    {
    	left_encoder.reset();
    }
    public void autonomousPeriodic()
    {
    	while(isAutonomous() && isEnabled())
    	{
    		double pote = pot.get();
        	
        	SmartDashboard.putNumber("Pot Value", pote);
          	SmartDashboard.putBoolean("Override State", override_lift);
        	SmartDashboard.putBoolean("Robot Direction State", which_way);
        	SmartDashboard.putNumber("Left Encoder", left_encoder.get());
    	switch(autonchooser)
    	{

    	case 1:
    	{
    		while(left_encoder.get() < 700)
    		{
    			first_person_drive.setLeftRightMotorOutputs(0.5, 0.5);
    		}
    		first_person_drive.setLeftRightMotorOutputs(0, 0);
    		break;
    		
    	}
    	case 2:	
   	{
   		liftmotors(0.5);
   		Timer.delay(0.8);
   		liftmotors(0);
   		Timer.delay(0.8);
   		liftmotors(-0.75);
   		Timer.delay(1.5);
   		liftmotors(0);
   		while(left_encoder.get() < 655)
   		{
   			first_person_drive.setLeftRightMotorOutputs(0.5, 0.5);
   		}
        first_person_drive.setLeftRightMotorOutputs(0, 0);
        Timer.delay(1);
        while(autoncounter < 38)
        {
        	first_person_drive.setLeftRightMotorOutputs(0.4, -0.4);
        	autoncounter++;
        }
        first_person_drive.setLeftRightMotorOutputs(0, 0);
        autonchooser = 9;
        break;
   		
   	}
   	case 3:
   	{
   	 while(autoncounter < 38)
     {
     	first_person_drive.setLeftRightMotorOutputs(0.4, -0.4);
     	autoncounter++;
     }
     first_person_drive.setLeftRightMotorOutputs(0, 0);
     autonchooser = 9;
     break;
   	}
   	case 5:
   	{
   		right_hook.set(DoubleSolenoid.Value.kForward);
   		left_hook.set(DoubleSolenoid.Value.kForward);
   		Timer.delay(4);
   		while(left_encoder.get() < -1300)
   		{
   			first_person_drive.setLeftRightMotorOutputs(-0.6, -0.6);
   		}
   		first_person_drive.setLeftRightMotorOutputs(0, 0);
   		autonchooser = 9;
   		break;
   	}
   	case 6: 
   	{
   		right_hook.set(DoubleSolenoid.Value.kForward);
   		left_hook.set(DoubleSolenoid.Value.kForward);
   		Timer.delay(4);
   		while(autoncounter < 700)
   		{
   			first_person_drive.setLeftRightMotorOutputs(-0.6, -0.6);
   			autoncounter++;
   		}
   		first_person_drive.setLeftRightMotorOutputs(0, 0);
   		autonchooser = 9;
   		break;
   	}
   	case 9:
   	{
   		liftmotors(0);
   		rollers.set(0);
		leftintake.set(0);
		rightintake.set(0);
   		first_person_drive.setLeftRightMotorOutputs(0, 0);
   		break;
   	}
    	}
   
    }
    }
    
    
    

    /**
     * 
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	while(isOperatorControl() && isEnabled())
    	{
    		double pote = pot.get();
    		double half_motor_speed = (operatorcontroller.getRawAxis(1) * 0.7);
        	SmartDashboard.putNumber("Pot Value", pote);
        	SmartDashboard.putNumber("Left Encoder", left_encoder.get());
        	SmartDashboard.putBoolean("Override State", override_lift);
        	SmartDashboard.putBoolean("Robot Direction State", which_way);
        	
    		//code to drive robot
    		if(drivercontroller.getRawButton(2))
    		{
    			which_way =true;
    		}
    		else if(drivercontroller.getRawButton(4))
    		{
    			which_way = false;
    		}
    		
    		
    		if(which_way)
    		{	
    			first_person_drive.setLeftRightMotorOutputs(-drivercontroller.getRawAxis(1) - drivercontroller.getRawAxis(2), -drivercontroller.getRawAxis(1) + drivercontroller.getRawAxis(2));    			
    		}
    		else if(!which_way)
    		{
    			first_person_drive.setLeftRightMotorOutputs(drivercontroller.getRawAxis(1) - drivercontroller.getRawAxis(2), drivercontroller.getRawAxis(1) + drivercontroller.getRawAxis(2));
    		}
    		
    		
    
    	    if(operatorcontroller.getRawAxis(1) > 0)
    		{
    			liftmotors(half_motor_speed);
    		}
    		else if(operatorcontroller.getRawAxis(1) < 0)
    		{
    			liftmotors(operatorcontroller.getRawAxis(1));
    		}
			else
    		{
    			liftmotors(0);
    		}	
    		
    		
    		// This code is for drivercontroller which controls the intake
    		if(drivercontroller.getRawButton(6))
    		{
    			rollers.set(1);
    			leftintake.set(-1);
    			rightintake.set(1);
    		}
    		else if(drivercontroller.getRawButton(8))
    		{
    			rollers.set(-1);
    			leftintake.set(1);
    			rightintake.set(-1);
    		}
    		else
    		{
    			rollers.set(0);
    			leftintake.set(0);
    			rightintake.set(0);
    		}
    		
    		if(operatorcontroller.getRawButton(5))
    		{

    			right_hook.set(DoubleSolenoid.Value.kForward);
    		}
    		else 
    		{
    			right_hook.set(DoubleSolenoid.Value.kReverse);
    		}
    		
    		if(operatorcontroller.getRawButton(6))
    		{
    			left_hook.set(DoubleSolenoid.Value.kForward);
    		}
    		else
    		{
    			left_hook.set(DoubleSolenoid.Value.kReverse);
    		}
    		
    
    	}
    		
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}

