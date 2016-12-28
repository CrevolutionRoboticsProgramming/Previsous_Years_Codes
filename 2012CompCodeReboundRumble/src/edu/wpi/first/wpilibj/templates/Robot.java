// Tartarus encoder ratio is 2000 counts for 4 feet
// Iris encoder ratio is 2950 counts for 4 feet.
// Tested/Working ratio between Tartarus to Iris is 1:1.4

package edu.wpi.first.wpilibj.templates; 
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
//creates variables for all objects
    private Joystick driverControl, operatorControl;
    private RobotDrive tank;
    private Victor shooter1, shooter2, intake_front, intake_back, lifter, lifter2, turret;
    private Compressor compressor;
    private DoubleSolenoid hopper, ramp_arm;
    private Encoder leftEncoder, rightEncoder;
    private Servo vexmotor1;
    
    private AnalogChannel vexpot;
    private Relay LED2; //lifter and shooter lights
    
    public static final double dunk = 0.33;  //constants for shooting preset
    public static final int dunkAngle = 770;
    public static final double key = 0.85;
    public static final int keyAngle = 640;
    public static final double full = 1;
    public static final int fullAngle = 650;
    
    //Constant for Auton
//    public static final double encoderRatio = (1); // practice robot
    public static final double encoderRatio = (1.4);  //competition robot 
    
    //Choose your Auton
    public static final int autonSelected = 3;  //The auton that runs (change number to change auton)
    /*
     *   1 = <QuickDraw> Shoot stationary from front of key
     *          Starts: front of Key on center
     * 
     *   2 = <AutonomousPrime> Dunker
     *          Starts: front of Key on sides
     * 
     *   3 = <COLD> Back to bridge, collect balls, drive to front of key and shoot
     *          Starts: Front wheels on Arch, Center
     * 
     *   4 = <Drop it like it's HOT> Shoot 2 balls, back to collect from bridge, drives to FRONT OF KEY and shoots again
     *           Starts: BACK wheels on Arch, Center
     * 
     *   5 = <SunDrop> Shoot 2 balls, back to collect from bridge, returns to START and shoots again
     *            Starts: BACK wheels on Arch, Center
     * 
     *   6 = <BOOMER> Spits out balls as it rolls back and forward slow(NG)
     *            Starts: Front wheels on Arch, Center
     *       
     *   7 = <A Cold War Carnival> Shoot 2 balls, back to collect from bridge, returns to START and shoots again
     *            Starts: Like COLD
     *
     *   13 = <USSR> Back to bridge at FULL speed, collect balls, drive to front of key and shoot(test)
     *          Starts: Front wheels on Arch, Center
     *
     *   14 = <Stone COLD> Back to bridge, collect balls, and shoot, but NO Driving forward(test)
     *          Starts: Front wheels on Arch, Center
     *
     *   9 = <HerpDerp> Does nothing! (continues last action)
     */      
    private boolean autonReset = false; 
    private int autonChoice = autonSelected; //used to run auton once til expiration
    private int shooterRev = 1;
    
    public void robotInit(){
    //Runs when robot turns on
        driverControl = new Joystick(RobotMap.gamepad1);    //instantiates the Joysticks 
        operatorControl = new Joystick(RobotMap.gamepad2);
    
        tank = new RobotDrive(RobotMap.leftMotor,RobotMap.rightMotor);  //instantiates the motors
        intake_front = new Victor(RobotMap.intake_front);
        intake_back = new Victor (RobotMap.intake_back);
        lifter = new Victor(RobotMap.lifter);
         lifter2 = new Victor(RobotMap.lifter2);
        
        vexmotor1 = new Servo(RobotMap.vexmotor1);
        
        turret = new Victor(RobotMap.turret);
        shooter1 = new Victor(RobotMap.shooter1);
        shooter2 = new Victor(RobotMap.shooter2); 
              
        compressor = new Compressor(RobotMap.compressorSwitch, RobotMap.compressorRelay);   //instantiates the Compressor
        hopper = new DoubleSolenoid(RobotMap.solenoid1A, RobotMap.solenoid1B);   //instantiates the solenoids
        ramp_arm = new DoubleSolenoid(RobotMap.solenoid2A, RobotMap.solenoid2B);

        leftEncoder = new Encoder(RobotMap.leftEncoderA, RobotMap.leftEncoderB);    //instantiates the encoders
        rightEncoder = new Encoder(RobotMap.rightEncoderA,RobotMap.rightEncoderB);
                
        vexpot = new AnalogChannel(RobotMap.vexpot);    //instantiates the potentimeter
        LED2 = new Relay(RobotMap.LEDRelay);  //instantiates the relay for LEDs
       
        leftEncoder.start();    //Starts what needs the be started when robot turms on
        rightEncoder.start(); 
        compressor.start(); 
    }
    
    public void autonomousPeriodic()
    {
        //reset the encoders to zero //?
        leftEncoder.reset();
        rightEncoder.reset(); 
        tank.setSafetyEnabled(false);
        
        while (isAutonomous() && isEnabled()) 
        {
            switch(autonChoice)
            { 
              case 1:   //shoot up to 4+ balls from top of key and hope that other teams give us balls
              {  
                    shooter1.set(-key); //puts shooting motors at 70% for key shots
                    shooter2.set(key);
                    
                  //hood adjust 
                do
                {
                    if(vexpot.getValue()>(keyAngle+7))  //sets hood to proper angle for key shot
                    {
                        vexmotor1.set(0);      //Hood down?  
                    }else if(vexpot.getValue()<(keyAngle-7))
                    {
                        vexmotor1.set(1);      //Hood up?
                    }else
                    {
                        vexmotor1.set(0.5); //if it gets to the right place, stop (likely will not reach this in the while
                    }
                }while(((vexpot.getValue())>(keyAngle+7) || (vexpot.getValue())<(keyAngle-7)) && isAutonomous());    
                 vexmotor1.set(0.5); //make sure it's stopped

                    LED2.set(Relay.Value.kForward);   //activates LEDs for shooting
                    hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting

                    Timer.delay(2);     //CHANGE TO 5 FOR gUERILLAS
 
                    lifter.set(1);  lifter2.set(1);
                    intake_back.set(1);
                    intake_front.set(1);

                    Timer.delay(3);


                    lifter.set(0);  lifter2.set(0);
                    intake_back.set(0);
                    intake_front.set(0);

                    Timer.delay(1); //end shot 1


                    lifter.set(1);  lifter2.set(1);
                    intake_back.set(1);
                    intake_front.set(1);

                    Timer.delay(2);


                    lifter.set(0);  lifter2.set(0);
                    intake_back.set(0);
                    intake_front.set(0);
                    
                    Timer.delay(1); //end shot 2

                    lifter.set(1);  lifter2.set(1);
                    intake_back.set(1);
                    intake_front.set(1);
////Keep shooting
//                    Timer.delay(3);
//
//                    lifter.set(0);  lifter2.set(0);
//                    intake_back.set(0);
//                    intake_front.set(0);
//
//                    Timer.delay(2); //end shot 3
//
//                    lifter.set(1);  lifter2.set(1);
//                    intake_back.set(1);
//                    intake_front.set(1);

                    autonChoice = 9;
                    break;
              }
              case 2:                   //goes straight and dunks
              {
                  
                 LED2.set(Relay.Value.kForward);   //activates LEDs for shooting
                 hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                 //prime the shooter
//                 shooter1.set(-(.5)); // 50% to get momentum
//                 shooter2.set(.5);
//                 Timer.delay(1);
                 shooter1.set(-(dunk)); // from fender
                 shooter2.set(dunk); 
                 
                 intake_back.set(1); //start rollers early to pick up anything that may fall in front
                 intake_front.set(1);
                
                 //hood adjust 
                do
                {
                    if(vexpot.getValue()>(dunkAngle+5))  //sets hood to proper angle for dunk was at 800
                    {
                        vexmotor1.set(0);      //Hood down?  
                    }else if(vexpot.getValue()<(dunkAngle-5))
                    {
                        vexmotor1.set(1);      //Hood up?
                    }else
                    {
                        vexmotor1.set(0.5); //if it gets to the right place, stop (likely will not reach this in the while
                    }
                }while(((vexpot.getValue())>(dunkAngle+5) || (vexpot.getValue())<(dunkAngle-5)) && isAutonomous());    
                 vexmotor1.set(0.5); //make sure it's stopped
                
                 do 
                 {
                    tank.tankDrive(.75, .75);   //goes straight  
                 }while((rightEncoder.get()) < (3300*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                 
                 tank.tankDrive(0, 0);  //stops

//                 Timer.delay(2);
 
                 lifter.set(1);  lifter2.set(1);

                 Timer.delay(2);

                 lifter.set(0);  lifter2.set(0);
                 intake_back.set(0);
                 intake_front.set(0);

                 //Timer.delay(2.5); //end shot 1

                 lifter.set(1);  lifter2.set(1);
                 intake_back.set(1);
                 intake_front.set(1);

                 Timer.delay(2);
                 autonChoice = 9;
                 break;
              }
                  
              case 3:                 
                  //goes back to bridge, lowers it, goes back to front of key and shoots until end
              {
                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //make sure the wheelie bar is up
                 LED2.set(Relay.Value.kForward);   //activates LEDs for shooting
                 hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                 shooter1.set(-key); //puts shooting motors at key%                key shots
                 shooter2.set(key);
                 
                 do
                 {
                    tank.tankDrive(-0.75, -0.75);   //drives backwards
                 }while(rightEncoder.get() > (-2300*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                    
                 tank.tankDrive(0, 0);  //stops motor
                 ramp_arm.set(DoubleSolenoid.Value.kReverse);   //puts down wheelie bar      
                 
                 intake_back.set(1); //turn on rollers
                 intake_front.set(1);
                 
                 Timer.delay(4.5);    //wait for balls to drop
                 
                 
                 do
                 {
                    tank.tankDrive(.75, .75);   //drive straight
                 }while(rightEncoder.get() < (2300*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                 
                 tank.tankDrive(0, 0);  //stops motor

                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //puts up wheelie bar 
                 
                     //hood adjust 
                do
                {
                    if(vexpot.getValue()>(keyAngle+7))  //sets hood to proper angle for key shot
                    {
                        vexmotor1.set(0);      //Hood down?  
                    }else if(vexpot.getValue()<(keyAngle-7))
                    {
                        vexmotor1.set(1);      //Hood up?
                    }else
                    {
                        vexmotor1.set(0.5); //if it gets to the right place, stop (likely will not reach this in the while
                    }
                }while(((vexpot.getValue())>(keyAngle+7) || (vexpot.getValue())<(keyAngle-7)) && isAutonomous());    
                 vexmotor1.set(0.5); //make sure it's stopped
                 
                 lifter.set(1);  lifter2.set(1); //first shot
                 intake_back.set(1);
                 intake_front.set(1);
                 autonChoice = 9;
                 break;
              }
              case 4:                 
                  //Shoots 2, THEN goes back to bridge, lowers it, goes back to FRONT OF KEY and shoots utill end
              {
                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //make sure the wheelie bar is up
                 LED2.set(Relay.Value.kForward);   //activates LEDs for shooting
                 hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                 shooter1.set(-full);    //shoot from back of key
                 shooter2.set(full);
                 
                 Timer.delay(1.5);
                 
                //hood adjust 
                do
                {
                    if(vexpot.getValue()>(keyAngle+7))  //sets hood to proper angle for key shot
                    {
                        vexmotor1.set(0);      //Hood down?  
                    }else if(vexpot.getValue()<(keyAngle-7))
                    {
                        vexmotor1.set(1);      //Hood up?
                    }else
                    {
                        vexmotor1.set(0.5); //if it gets to the right place, stop (likely will not reach this in the while
                    }
                }while(((vexpot.getValue())>(keyAngle+7) || (vexpot.getValue())<(keyAngle-7)) && isAutonomous());    
                 vexmotor1.set(0.5); //make sure it's stopped
                 
                 lifter.set(1);  lifter2.set(1); //get things lifting and sucking
                 intake_back.set(1);
                 intake_front.set(1);
                 
                 Timer.delay(2); //wait enough time to shoot two balls
                 
                 hopper.set(DoubleSolenoid.Value.kForward);  //deactivate hopper
                 
                 do
                 {
                    tank.tankDrive(-0.75, -0.75);   //drives backwards to bridge (STARTS with backwheels on back of key)
                 }while(rightEncoder.get() > (-3000*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                    
                 tank.tankDrive(0, 0);  //stops motor
                 ramp_arm.set(DoubleSolenoid.Value.kReverse);   //puts down wheelie bar      
                 
                 Timer.delay(4.5);    //wait for balls to drop
                 
                 shooter1.set(-key); //shooter motors at speed for front of key shots
                 shooter2.set(key);
                 
                 do
                 {
                    tank.tankDrive(.75, .75);   //drive back to Key
                 }while(rightEncoder.get() < (-500*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                 
                 hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                 
                 do
                 {
                    tank.tankDrive(.75, .75);   //drive straight to front of key
                 }while(rightEncoder.get() < (1600*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                 
                 tank.tankDrive(0, 0);  //stops motor

                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //puts up wheelie bar 
                 
                 autonChoice = 9;
                 break;
              }
              case 5:                 
                  //Shoots 2, THEN goes back to bridge, lowers it, returns to START and shoots utill end (STARTS with backwheels on back of key)
              {
                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //make sure the wheelie bar is up
                 LED2.set(Relay.Value.kForward);   //activates LEDs for shooting
                 hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                 shooter1.set(-full);    //shoot from back of key
                 shooter2.set(full);
                 
                 Timer.delay(1.5);
                 
                //hood adjust 
                do
                {
                    if(vexpot.getValue()>(keyAngle+7))  //sets hood to proper angle for key shot
                    {
                        vexmotor1.set(0);      //Hood down?  
                    }else if(vexpot.getValue()<(keyAngle-7))
                    {
                        vexmotor1.set(1);      //Hood up?
                    }else
                    {
                        vexmotor1.set(0.5); //if it gets to the right place, stop (likely will not reach this in the while
                    }
                }while(((vexpot.getValue())>(keyAngle+7) || (vexpot.getValue())<(keyAngle-7)) && isAutonomous());    
                 vexmotor1.set(0.5); //make sure it's stopped
                 
                 lifter.set(1);  lifter2.set(1); //get things lifting and sucking
                 intake_back.set(1);
                 intake_front.set(1);
                 
                 Timer.delay(2); //wait enough time to shoot two balls
                 
                 do
                 {
                    tank.tankDrive(-0.75, -0.75);   //drives backwards to bridge
                 }while(rightEncoder.get() > (-3000*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                    
                 tank.tankDrive(0, 0);  //stops motor
                 ramp_arm.set(DoubleSolenoid.Value.kReverse);   //puts down wheelie bar      
                 
                 Timer.delay(4.5);    //wait for balls to drop
                 
                 do
                 {
                    tank.tankDrive(.75, .75);   //drives straight back to near starting location
                 }while(rightEncoder.get() < (-450*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                 
                 tank.tankDrive(0, 0);  //stops motor

                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //puts up wheelie bar 
                 
                 autonChoice = 9;
                 break;
              }
              case 6: //Delays, then feeds bot in front while driving back and forward
                        //DOES NOT SHOOT
                        //Front wheels on back of key
              {
                 Timer.delay(2); //change for for longer wait
                 
                 lifter.set(-1); lifter2.set(-1); //get things lifting and sucking
                 intake_back.set(1);
                 intake_front.set(-1);
                 
                 do
                 {
                    tank.tankDrive(-0.5, -0.5);   //drives backwards a short way
                 }while(rightEncoder.get() > (-1100*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                 
                 lifter.set(0);  lifter2.set(0); //stops lifter
                 Timer.delay(1);
                 
                 do
                 {
                    tank.tankDrive(0.5, 0.5);   //drives forward
                 }while(rightEncoder.get() < (-100*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                 
                 tank.tankDrive(0, 0);  //stops motor
//                 ramp_arm.set(DoubleSolenoid.Value.kReverse);   //puts down wheelie bar      
                 
                 autonChoice = 9;  
                 break;
              }
              case 7:                 
                  //Shoots 2, THEN goes back to bridge, lowers it, returns to START and shoots utill end (STARTS like cold)
              {
                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //make sure the wheelie bar is up
                 LED2.set(Relay.Value.kForward);   //activates LEDs for shooting
                 hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                 shooter1.set(-full);    //shoot from back of key
                 shooter2.set(full);
                 
                 Timer.delay(2);
                 
                //hood adjust 
                do
                {
                    if(vexpot.getValue()>(keyAngle+7))  //sets hood to proper angle for key shot
                    {
                        vexmotor1.set(0);      //Hood down?  
                    }else if(vexpot.getValue()<(keyAngle-7))
                    {
                        vexmotor1.set(1);      //Hood up?
                    }else
                    {
                        vexmotor1.set(0.5); //if it gets to the right place, stop (likely will not reach this in the while
                    }
                }while(((vexpot.getValue())>(keyAngle+7) || (vexpot.getValue())<(keyAngle-7)) && isAutonomous());    
                 vexmotor1.set(0.5); //make sure it's stopped
                 
                 lifter.set(1);  lifter2.set(1); //get things lifting and sucking
                 intake_back.set(1);
                 intake_front.set(1);
                 
                 Timer.delay(3); //wait enough time to shoot two balls
                 
                 do
                 {
                    tank.tankDrive(-0.75, -0.75);   //drives backwards to bridge
                 }while(rightEncoder.get() > (-2300*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                    
                 tank.tankDrive(0, 0);  //stops motor
                 ramp_arm.set(DoubleSolenoid.Value.kReverse);   //puts down wheelie bar      
                 
                 Timer.delay(4.5);    //wait for balls to drop
                 
                 do
                 {
                    tank.tankDrive(.75, .75);   //drives straight back to near starting location
                 }while(rightEncoder.get() < (-450*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                 
                 tank.tankDrive(0, 0);  //stops motor

                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //puts up wheelie bar 
                 
                 autonChoice = 9;
                 break;
              }
              case 13:                 
                  //goes back to bridge at FULL speed, lowers it for 5secs, goes back to front of key and shoots until end
              {
                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //make sure the wheelie bar is up
                 LED2.set(Relay.Value.kForward);   //activates LEDs for shooting
                 hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                 shooter1.set(-key); //puts shooting motors at key%                key shots
                 shooter2.set(key);
                 
                 do
                 {
                    tank.tankDrive(-1, -1);   //drives backwards
                 }while(rightEncoder.get() > (-2100*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                    
                 tank.tankDrive(0, 0);  //stops motor
                 ramp_arm.set(DoubleSolenoid.Value.kReverse);   //puts down wheelie bar      
                 
                 intake_back.set(1); //turn on rollers
                 intake_front.set(1);
                 
                 Timer.delay(5);    //wait for balls to drop
                                  
                 do
                 {
                    tank.tankDrive(.75, .75);   //drive straight
                 }while(rightEncoder.get() < (2300*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                 
                 tank.tankDrive(0, 0);  //stops motor

                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //puts up wheelie bar 
                 
                     //hood adjust 
                do
                {
                    if(vexpot.getValue()>(keyAngle+7))  //sets hood to proper angle for key shot
                    {
                        vexmotor1.set(0);      //Hood down?  
                    }else if(vexpot.getValue()<(keyAngle-7))
                    {
                        vexmotor1.set(1);      //Hood up?
                    }else
                    {
                        vexmotor1.set(0.5); //if it gets to the right place, stop (likely will not reach this in the while
                    }
                }while(((vexpot.getValue())>(keyAngle+7) || (vexpot.getValue())<(keyAngle-7)) && isAutonomous());    
                 vexmotor1.set(0.5); //make sure it's stopped
                 
                 lifter.set(1);  lifter2.set(1); //first shot
                 intake_back.set(1);
                 intake_front.set(1);
                 autonChoice = 9;
                 break;
              }
              case 14:                 
                  //goes back to bridge, lowers it, nd shoots until end(NO driving forward)
              {
                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //make sure the wheelie bar is up
                 LED2.set(Relay.Value.kForward);   //activates LEDs for shooting
                 hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                 shooter1.set(-key); //puts shooting motors at key%                key shots
                 shooter2.set(key);
                 
                 do
                 {
                    tank.tankDrive(-1, -1);   //drives backwards
                 }while(rightEncoder.get() > (-2100*(encoderRatio)) && isAutonomous());  //makes sure that it attains a certain distance
                    
                 tank.tankDrive(0, 0);  //stops motor
                 ramp_arm.set(DoubleSolenoid.Value.kReverse);   //puts down wheelie bar      
                 
                 intake_back.set(1); //turn on rollers
                 intake_front.set(1);
                 
//                 Timer.delay(7);    //wait for balls to drop
//
//                 ramp_arm.set(DoubleSolenoid.Value.kForward);   //puts up wheelie bar 
                 
                     //hood adjust 
                do
                {
                    if(vexpot.getValue()>(keyAngle+7))  //sets hood to proper angle for key shot
                    {
                        vexmotor1.set(0);      //Hood down?  
                    }else if(vexpot.getValue()<(keyAngle-7))
                    {
                        vexmotor1.set(1);      //Hood up?
                    }else
                    {
                        vexmotor1.set(0.5); //if it gets to the right place, stop (likely will not reach this in the while
                    }
                }while(((vexpot.getValue())>(keyAngle+7) || (vexpot.getValue())<(keyAngle-7)) && isAutonomous());    
                 vexmotor1.set(0.5); //make sure it's stopped
                 
                 lifter.set(1);  lifter2.set(1); //first shot
                 intake_back.set(1);
                 intake_front.set(1);
                 autonChoice = 9;
                 break;
              }
              case 9:
              {
                 autonReset = false;    //make sure it resets in teleop
                 break;
              }
              default:  //what happens when nothing is selected
              {
                     tank.tankDrive(0, 0);  //drive motors stop
                     autonReset = false;    //make sure it resets in teleop
                     break;
              }
          }
        }
    }
    
    public void teleopPeriodic()
     {
    //runs during teleop
        while (true && isOperatorControl() && isEnabled())
        {    
//<--RESET AUTON--> 
            
            if(autonReset == false)
            {
                leftEncoder.reset();    //reset auton condition
                rightEncoder.reset();
                autonChoice = autonSelected;
                autonReset = true;
            }
//</--RESET AUTON-->
//</--SMARTDASHBOARD-->
            SmartDashboard.putDouble("Right Encoder", rightEncoder.get());         //displays rightencoder value
            SmartDashboard.putDouble("Left Encoder", leftEncoder.get());         //displays rightencoder value
            SmartDashboard.putInt("pot value", vexpot.getValue());                    //displays the poteniameter value for the hood
//</--SMARTDASHBOARD--> 
//<--DRIVER CONTROL-->    

               tank.tankDrive(-driverControl.getRawAxis(4),-driverControl.getRawAxis(2));   //regular speed for drive wheelz
//</--DRIVER CONTROL-->

//<--WHEELIE BAR-->
            if(driverControl.getRawButton(5))              //bar ges up
            {
                ramp_arm.set(DoubleSolenoid.Value.kForward);
            }else if(driverControl.getRawButton(6))   //when button 6 is pressed, bar goes down
            {
                ramp_arm.set(DoubleSolenoid.Value.kReverse);             
            }
//</--WHEELIE BAR-->
             
//<--OPERATOR CONTROL-->    

    //<--LIFTER INTAKE SYSTEM-->
            // 1 = IN and -1 = OUT
            if(operatorControl.getRawButton(1)) //when button 1 is pressed ball goes up the tower
            {
                lifter.set(1);  lifter2.set(1);  
            }
            else if(operatorControl.getRawButton(2))  //when button 2 is pressed ball goes down
            {
                lifter.set(-1); lifter2.set(-1); 
            }
            else if(operatorControl.getRawButton(3)) //button 3 is pressed rollers spin outward
            {   
                intake_front.set(-1);
                intake_back.set(-1);
            }
            else if(operatorControl.getRawButton(4)) //button 4 is pressed rollers spin inward and tower goes up
            {   
                intake_front.set(1);
                intake_back.set(1);
                lifter.set(1);  lifter2.set(1);
            }
            else if(operatorControl.getRawButton(5)) //button 5 is pressed rollers spin such that the ball goes to the rear
            {
                intake_front.set(1);
                intake_back.set(-1);
            }
            else if(operatorControl.getRawButton(6)) //button 6 is pressed rollers spin such that the ball goes to the front
            {   
                intake_front.set(-1);
                intake_back.set(1);  
            }
            else                           //if no buttons are pressed, intake rollers don't move (is this needed or is it extra code?)
            {
                intake_front.set(0);
                intake_back.set(0);
                lifter.set(0);  lifter2.set(0); 
            }
    //</--LIFTER INTAKE SYSTEM-->
            
    //<--SHOOTING CONTROL/HOOD CONTROL-->
            //exponential turret control through the right joystick with 50% max
            double turretSpin = -operatorControl.getRawAxis(3);
            if(Math.abs((turretSpin)) < .45 && Math.abs((turretSpin)) > 0.05)
            {
            turret.set(.1*((turretSpin)/(Math.abs((turretSpin)))));
            }else{
            turret.set(0.50 * (turretSpin)*Math.abs((turretSpin)));  
            }
            
            if(operatorControl.getRawButton(9)) //when button 9 on operator is pressed, shooters shoot at 100% and 45 degrees 
            {
                LED2.set(Relay.Value.kForward);   //activates LEDs for shooting
                hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                shooter1.set(-full); //puts shooting motors at 100%    long range shots
                shooter2.set(full);
                                
//                if(vexpot.getValue()>(fullAngle+5))  //sets hood to 45 degrees
                if(vexpot.getValue()>(690+5))  //sets hood to world champ end game
                {
                    vexmotor1.set(0);       //Hood down?
//                }else if(vexpot.getValue()<(fullAngle-5))
                }else if(vexpot.getValue()<(690-5)) //end game
                {
                    vexmotor1.set(1);       //Hood up?
                }
                else
                {
                    vexmotor1.set(0.5);
                }
            }else if(operatorControl.getRawButton(8)) //when button 8 on operator is pressed shooters shot at 80%
            {
                LED2.set(Relay.Value.kForward);   //activates LEDs for shooting
                hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                shooter1.set(-key); //puts shooting motors from key                key shots
                shooter2.set(key);
                
                if(vexpot.getValue()>(keyAngle+7))  //sets hood to proper angle
                {
                    vexmotor1.set(0);       //Hood down?
                }else if(vexpot.getValue()<(keyAngle-7))
                {
                    vexmotor1.set(1);       //Hood up?
                }else
                {
                   vexmotor1.set(0.5);
                }
            }else if(operatorControl.getRawButton(7)) //when button 7 on operator is pressed
            {
                LED2.set(Relay.Value.kForward);
                hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                
                if (shooterRev <= 5) //rev shooter on first shot
                {
                    shooter1.set(-0.5); //motor speed
                    shooter2.set( 0.5);
                    shooterRev = shooterRev + 1; //step the timer
                } else {
                shooter1.set(-dunk);                         //puts shooting motors from fender
                shooter2.set(dunk);
                }
                
                if(vexpot.getValue()> (dunkAngle+5))  //sets hood to proper angle
                {
                    vexmotor1.set(0);      //Hood down?  
                }else if(vexpot.getValue()<(dunkAngle-5))
                {
                    vexmotor1.set(1);      //Hood up?
                }
                else
                {
                    vexmotor1.set(0.5);
                }    
            }else if(operatorControl.getRawButton(10))   //activates shooting from far (full power) NO hood adjustment
            {
                LED2.set(Relay.Value.kForward);
                hopper.set(DoubleSolenoid.Value.kReverse);  //activates hopper for shooting
                shooter1.set(-full); //puts shooting motors at 100%    long range shots
                shooter2.set(full);
            }else if(driverControl.getRawButton(4))   //when buttons 5 and 4 are pressed, hood goes up
            {
                vexmotor1.set(1); //UP
            } //when buttons 5 and 4 are pressed, hood goes down //not working?
            else if(driverControl.getRawButton(3)) 
            {
                vexmotor1.set(0); //Down
            }else                                     //when none of the buttons are pressed, hopper returns to nonshooting mode and shooter stops
            {
                hopper.set(DoubleSolenoid.Value.kForward);
                shooter1.set(0);
                shooter2.set(0);
                vexmotor1.set(.5);
                LED2.set(Relay.Value.kOff); //lights be gone!
                
                shooterRev = 1; //reset the dunk shooter rev up
            }           
        } 
    //</--SHOOTING/HOOD CONTROL CONTROL--> 
//</--OPERATOR CONTROL-->
    }
    
    public void Disable()
    { 
        while(isDisabled())
        {
            tank.stopMotor();   //Stops everything
            intake_front.stopMotor();  
            intake_back.stopMotor();
            lifter.stopMotor();
            lifter2.stopMotor();
            turret.stopMotor();
            shooter1.stopMotor();
            shooter2.stopMotor();
            vexmotor1.set(0.5);
            leftEncoder.reset();
            rightEncoder.reset();
        } 
    } 
}
