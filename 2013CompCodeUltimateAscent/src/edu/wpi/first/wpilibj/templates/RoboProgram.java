/*----------------------------------------------------------------------------
 Copyright (c) FIRST 2008. All Rights Reserved.                             
 Open Source Software - may be modified and shared by FRC teams. The code   
 must be accompanied by the FIRST BSD license file in the root directory of 
 the project.                                                               
----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;

public class RoboProgram extends IterativeRobot {
// <editor-fold>
    //<editor-fold defaultstate="collapsed" desc="Robotic Components Variables">
    public static AutonSubsystem autonsubsystem;
    public static Relay LED;
    public Joystick drivercontroller, operatorcontroller;
    public static RobotDrive tank; //RobotDrive controls all of the Jaguars: leftmotors, rightmotors 
    public static Victor intake, winch;
    public static DoubleSolenoid lifter, indexer, angler; 
    public static Encoder leftencoders, rightencoders, shooterencoder;
    public Compressor compressor;
    public static AnalogChannel potentiometer;     //for arm
    public static Servo indexerservo1;
    public static Servo indexerservo2;
    public static Talon shooter, arm;
    public static DigitalInput limitswitchmax, limitswitchmin;
//    public static AxisCamera camera;
    private double DriveSpeed = Robomap.Drivespeed;
    //Auton Stuff
    public static int autonselected = Robomap.autonselected;
    Command autonchoice;
    SendableChooser autonchooser;
    //</editor-fold>
    //<editor-fold defaultstate="collapsed" desc="Auton Descriptions">
    //Choose your auton warrior!
        //(starting positions are relative to facing the goals)
        /*
            7 Disc-
                Starting Position: middle back of pyramid, arm up
	
                Shoot the 3 discs starting position 30 degree angle
                Get arm to drop to ground (back up)
                Drive forward
                Pickup two discs under pyrimid (intake on)
                Arm to forward position for loading
                Arm to ground for discs in front of pyramid
                Pick up 2 discs, load (arm forward)
                Shooter to 36 degrees 
                Drive back to shooting position (contact front of pyramid)
                Shoot four discs at 36 degree angle

            5 Disc-
                Starting Position: middle back of pyramid, arm up
                
                Shoot the 3 discs starting position 30 degree angle
                Get arm to drop to ground (back up)
                Drive forward
                Pickup two discs under pyrimid (intake on)
                Arm to forward position for loading
                Arm to up position, shooter to 36 degrees 
                Drive back to shooting position (contact front of pyramid)
                Shoot two discs at 36 degree angle

            5 Disc Midfield-
                Starting Position: middle back of pyramid, arm up

        	Shoot 3 discs at 30 degrees
                Turn around
                Drive backwards and pickup 2 discs at mid field
                Drive back to the pyrimid
                Shoot 2 dics at 30% angle

            4 Disc Front-
                Starting Position: middle front of pyramid, arm up
                
                Put arm to ground position
                Go forward and gather discs
                Put arm to forward position and feed in discs
                Drive back to shooting position for 36%
                Shoot 4 discs

            3 Disc Inside Back-
                Starting Position: middle back pyramid, arm up
                
                Shoot three discs at the middle goal at 30 degrees

            3 Disc Outside Back-
                Starting Position: back right side of pyramid
                
                Start with 3 discs
                Shoot them at the 2 point goal

            3 Disc Outside 2 Disc Dunk-
                Starting Position: back left side of pyramid, arm at ground
                
                Shoot 3 discs at the 2 point goal
                Drive forward, turn
                Gather discs in front of pyramid
                Dunk discs

            2 Disc Front-
                Starting Position: middle front of pyramid, arm up
                
            Herp Derp-
                Starting Position: anywhere
                
                Do nothing

            2 Disc Dunk-
                Starting Position: front right corner of the pyramid, facing low dunk goal (1 point goal), arm starts slightly raised
                start with 2 discs in the arm
                go forward to the dunk goal
                set intake to shoot out discs
        */
    //</editor-fold>
    //<editor-fold defaultstate="collapsed" desc="Assorted Variables">
    public static boolean autonreset = true;               //tells if auton encoders and the such have been reset
    public static boolean armmanualoverride = false;        //tells if manual override for the arm control is on
    public static boolean shootermanualoverride = true;    //tells if manual override for the shooter control is on
    public String armoverridestatus = "Disabled";           //displayed to smartdashboard
    public String shooteroverridestatus = "Disabled";       //displayed to smartdashboard
    public double driveoutleft = 0;                         //the next 4 are used to control driving full forward and then full backward or vice versa
    public double driveoutright = 0;                            //^used to control tankdrive
    public double driveoutleftpre = 0;                      //next 2 are previous values for driveoutleft and driveoutright, respectively
    public double driveoutrightpre = 0;
    public double shooterdelay = Robomap.ShooterDelay;
    public double servodelay = Robomap.ServoDelay;
    public boolean aftertap = true;
    public double armspeed = Robomap.ArmSpeed;  //defined the arm rotational speed
    public double shooterspeed = Robomap.ShooterSpeed;
    public boolean lastup = false;
    //</editor-fold>
    //<editor-fold defaultstate="collapsed" desc="PID Control Variables">
        //Shooter
            //Error
                public static double et = 0;        //current error from latest measurement to setpoint
                public static double eT = 0;        //total error over time (when integrating, assumed bounds are t=0 and t=t)
            //I/O
                public double PV = 0;        //system input via the sensor on robot (system input)
                public static double MV = 0;        //Manipulated Variable (system output)
                public double Pout = 0;         //output for just Kp operation
                public double Iout = 0;         //output for just Ki operation
                public double previous = 0;
                public double current = 0;
                public static double rate = 0;
                //when activating the indexer, RPM drops by about 400-500 RPM
        //Arm
            //Error
                public double armeT = 0;            //base variable
                public double armeTback = 0;
                public double armeTground = 0;
                public double armeTforward = 0;
                public double armeTup = 0;
                public double armSP = 0;
            //I/O
                public double armPV = 0;            //base variable
                public double armMV = 0;            //base variable
                public double armMVback = 0;
                public double armMVground = 0;
                public double armMVforward = 0;
                public double armMVup = 0;
                public double armPout = 0;          //base variable
                public double armIout = 0;          //base variable
    //</editor-fold>
    //<editor-fold defaultstate="collapsed" desc="Auton Encoder Variables">
        
        //All
            public boolean useright = Robomap.Useright;
        //7 Disc
            public int c1move1 = Robomap.d7move1;
            public int c1move2 = Robomap.d7move2;
            public int c1move3 = Robomap.d7move3;
        //5 Disc
            public int c2move1 = Robomap.d5move1;
            public int c2move2 = Robomap.d5move2;
            public int c2move3 = Robomap.d5move3;
        //5 Disc Midfield
            public int c3move1 = Robomap.dmmove1;
            public int c3move2 = Robomap.dmmove2;
            public int c3move3 = Robomap.dmmove3;
            public int c3move4 = Robomap.dmmove4;
            public int c3move5 = Robomap.dmmove5;
        //4 Disc Front
            public int c4move1 = Robomap.d4move1;
            public int c4move2 = Robomap.d4move2;
        //3 Disc Inside Back
            //NONE
        //3 Disc Outside Back
            //NONE
        //3 Disc Outside 2 Disc Dunk
            public int c7move1 = Robomap.odmove1;
            public int c7move2 = Robomap.odmove2;
            public int c7move3 = Robomap.odmove3;
            public int c7move4 = Robomap.odmove4;
            public int c7move5 = Robomap.odmove5;
        //2 Disc Front
            //NONE
        //Herp Derp
            //NONE
        //2 Disc Dunk
            public int c10move1 = Robomap.ddmove1;
    //</editor-fold>
// </editor-fold>    

    public void robotInit() {
//<editor-fold defaultstate="collapsed" desc="Main Variables">
        //Subsystem Stuff
            autonsubsystem = new AutonSubsystem();
        //Joystick Controllers
            drivercontroller = new Joystick(Robomap.drivergamepad);
            operatorcontroller = new Joystick(Robomap.operatorgamepad);
        //Wheels n' Stuff
            tank = new RobotDrive(Robomap.leftmotors, Robomap.rightmotors);
                //sets tank as the controlling variable for all the jaguar motors, for a 2 motor system
        //Encoders
            leftencoders = new Encoder(Robomap.leftencoderA, Robomap.leftencoderB);
            rightencoders = new Encoder(Robomap.rightencoderA, Robomap.rightencoderB);
                //locates the encoders and sets them to left and right sides
            shooterencoder = new Encoder(Robomap.shooterencoderA, Robomap.shooterencoderB);
        //Victors
            intake = new Victor(Robomap.intakevic);
            winch = new Victor(Robomap.winchvic);
        //Talons
            shooter = new Talon(Robomap.shootermotortal);
            arm = new Talon(Robomap.ARMtal);
        //Camera
//            camera = AxisCamera.getInstance();
        //Servo
            indexerservo1 = new Servo (Robomap.servoindexer1);
            indexerservo2 = new Servo (Robomap.servoindexer2);
        //Air Controllers
            compressor = new Compressor(Robomap.compressorswitch, Robomap.compressorrelay);
            lifter = new DoubleSolenoid(Robomap.solenoid1A, Robomap.solenoid1B);
            indexer = new DoubleSolenoid(Robomap.solenoid2A, Robomap.solenoid2B);
            angler = new DoubleSolenoid(Robomap.solenoid3A, Robomap.solenoid3B);
        //Limit Switches & Potentiometer
            potentiometer = new AnalogChannel (Robomap.potmet);
            limitswitchmax = new DigitalInput (Robomap.limitswitch1);
            limitswitchmin = new DigitalInput (Robomap.limitswitch2);
//</editor-fold>
//<editor-fold defaultstate="collapsed" desc="SmartDashboard Auton Buttons">
        //SmartDashBoard
            autonchooser = new SendableChooser();
            autonchooser.addDefault("Herp Derp", new Case9());
            autonchooser.addObject("Test Auton", new Testauton());
            autonchooser.addObject("7 Disc", new Disc7());
            autonchooser.addObject("5 Disc", new Disc5());
            autonchooser.addObject("5 Disc Midfield", new Disc5MidField());
            autonchooser.addObject("4 Disc Front", new disc4front());
            autonchooser.addObject("3 Disc Inside Back", new disc3insideback());
            autonchooser.addObject("3 Disc Outside Back", new disc3outsideback());
            autonchooser.addObject("3 Disc Outside 2 Disc Dunk", new disc3outside2discdunk());
            autonchooser.addObject("2 Disc Front", new disc2front());
            autonchooser.addObject("2 Disc Dunk", new disc2dunk());
            SmartDashboard.putData("Autonomous chooser", autonchooser);
//</editor-fold>
//<editor-fold defaultstate="collapsed" desc="Other Stuff">
            LED = new Relay(Robomap.LEDrelay);
            leftencoders.start();
            rightencoders.start(); //tells the encoders to start counting
            shooterencoder.start();
            compressor.start();
//</editor-fold>
    }

    public void autonomousPeriodic() {
        //<editor-fold defaultstate="collapsed" desc="Pre-Auton Setup">
        autonreset = false;
        autonselected = Robomap.autonselected;
        leftencoders.reset();
        rightencoders.reset();
        shooterencoder.reset();
            //resets the encoders' count
        tank.setSafetyEnabled(false);
        //</editor-fold>
        while (isAutonomous() && isEnabled())
        {
        //<editor-fold defaultstate="collapsed" desc="Command Based Auton Version">
            if (Robomap.usecommand)
            {
            autonchoice = (Command) autonchooser.getSelected();
            autonchoice.start();
            Scheduler.getInstance().run();
            }
        //</editor-fold>
        //<editor-fold defaultstate="collapsed" desc="Switch Based Auton Version">
            else
            {
            switch(autonselected)
            {
                case 1:
                    //<editor-fold defaultstate="collapsed" desc="7 Disc">
                {
                    //angle starts at 30 degrees
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    shooter.set(shooterspeed);
                    Timer.delay(5);
                    LED.set(Relay.Value.kForward);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    LED.set(Relay.Value.kOff);
                    if (useright)
                    {
                        while (rightencoders.get() > c1move1)
                        {
                            tank.tankDrive(-0.5, -0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() > c1move1)
                        {
                            tank.tankDrive(-0.5, -0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    while (potentiometer.getValue()/Robomap.potratio > Robomap.armgroundangle + Robomap.armtol && limitswitchmin.get() != true)
                    {
                        arm.set(armspeed);
                    }
                    arm.set(0);
                    intake.set(1);
                    LED.set(Relay.Value.kReverse);
                    if (useright)
                    {
                        while (rightencoders.get() < c1move2)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c1move2)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    intake.set(0);
                    LED.set(Relay.Value.kOff);
                    while (potentiometer.getValue()/Robomap.potratio < Robomap.armforwardangle - Robomap.armtol)
                    {
                        if (potentiometer.getValue()/Robomap.potratio > Robomap.armforwardangle - Robomap.ArmSlowTol)
                        {
                            arm.set(-armspeed * 0.3);
                        }
                        else
                        {
                            arm.set(-armspeed);
                        }
                    }
                    arm.set(0.05);
                    intake.set(1);
                    LED.set(Relay.Value.kReverse);
                    Timer.delay(1);
                    intake.set(0);
                    LED.set(Relay.Value.kOff);
                    while (potentiometer.getValue()/Robomap.potratio > Robomap.armgroundangle + Robomap.armtol && limitswitchmin.get() != true)
                    {
                        arm.set(armspeed);
                    }
                    arm.set(0);
                    intake.set(1);
                    LED.set(Relay.Value.kReverse);
                    if (useright)
                    {
                        while (rightencoders.get() < c1move3)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c1move3)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    intake.set(0);
                    LED.set(Relay.Value.kOff);
                    while (potentiometer.getValue()/Robomap.potratio < Robomap.armforwardangle - Robomap.armtol)
                    {
                        if (potentiometer.getValue()/Robomap.potratio > Robomap.armforwardangle - Robomap.ArmSlowTol)
                        {
                            arm.set(-armspeed * 0.3);
                        }
                        else
                        {
                            arm.set(-armspeed);
                        }
                    }
                    arm.set(0.05);
                    intake.set(1);
                    LED.set(Relay.Value.kReverse);
                    Timer.delay(1);
                    intake.set(0);
                    LED.set(Relay.Value.kOff);
                    angler.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    LED.set(Relay.Value.kForward);
                    Timer.delay(1);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    Timer.delay(1);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    LED.set(Relay.Value.kOff);
                    autonselected = 9;
                    break;
                }
                    //</editor-fold>
                case 2:
                    //<editor-fold defaultstate="collapsed" desc="5 Disc">
                {
                    //angler set to 30 degrees to start
                    shooter.set(shooterspeed);
                    Timer.delay(5);
                    LED.set(Relay.Value.kForward);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    LED.set(Relay.Value.kOff);
                    if (useright)
                    {
                        while (rightencoders.get() > c2move1)
                        {
                            tank.tankDrive(-0.5, -0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() > c2move1)
                        {
                            tank.tankDrive(-0.5, -0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    while (potentiometer.getValue()/Robomap.potratio > Robomap.armgroundangle + Robomap.armtol && limitswitchmin.get() != true)
                    {
                        arm.set(armspeed);
                    }
                    arm.set(0);
                    intake.set(1);
                    LED.set(Relay.Value.kReverse);
                    if (useright)
                    {
                        while (rightencoders.get() < c2move2)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c2move2)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    intake.set(0);
                    LED.set(Relay.Value.kOff);
                    while (potentiometer.getValue()/Robomap.potratio < Robomap.armforwardangle - Robomap.armtol)
                    {
                        if (potentiometer.getValue()/Robomap.potratio > Robomap.armforwardangle - Robomap.ArmSlowTol)
                        {
                            arm.set(-armspeed * 0.3);
                        }
                        else
                        {
                            arm.set(-armspeed);
                        }
                    }
                    arm.set(0.05);
                    intake.set(1);
                    LED.set(Relay.Value.kReverse);
                    Timer.delay(1);
                    intake.set(0);
                    LED.set(Relay.Value.kOff);
                    while (potentiometer.getValue()/Robomap.potratio > Robomap.armgroundangle + Robomap.armtol && limitswitchmin.get() != true)
                    {
                        arm.set(armspeed);
                    }
                    arm.set(0);
                    if (useright)
                    {
                        while (rightencoders.get() < c2move3)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c2move3)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    angler.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.5);
                    LED.set(Relay.Value.kForward);
                    Timer.delay(0.5);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    LED.set(Relay.Value.kOff);
                    autonselected = 9;
                    break;
                }
                    //</editor-fold>
                case 3:
                    //<editor-fold defaultstate="collapsed" desc="5 Disc Midfield">
                {
                    //angler set to 30 degrees to start
                    shooter.set(shooterspeed);
                    Timer.delay(5);
                    LED.set(Relay.Value.kForward);
                    Timer.delay(0.5);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    LED.set(Relay.Value.kOff);
                    if (useright)
                    {
                        while (rightencoders.get() > c3move1)
                        {
                           tank.tankDrive(-0.5, -0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() > c3move1)
                        {
                            tank.tankDrive(-0.5, -0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    if (useright)
                    {
                        while (rightencoders.get() < c3move2)
                        {
                            tank.tankDrive(-0.75, 0.75);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c3move2)
                        {
                            tank.tankDrive(0.75, -0.75);
                        }
                    }
                    tank.tankDrive(0, 0);
                    while (potentiometer.getValue()/Robomap.potratio > Robomap.armgroundangle + Robomap.armtol && limitswitchmin.get() != true)
                    {
                        arm.set(armspeed);
                    }
                    arm.set(0);
                    intake.set(1);
                    LED.set(Relay.Value.kReverse);
                    if (useright)
                    {
                        while (rightencoders.get() < c3move3)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c3move3)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    intake.set(0);
                    LED.set(Relay.Value.kOff);
                    while (potentiometer.getValue()/Robomap.potratio < Robomap.armforwardangle - Robomap.armtol)
                    {
                        if (potentiometer.getValue()/Robomap.potratio > Robomap.armforwardangle - Robomap.ArmSlowTol)
                        {
                            arm.set(-armspeed * 0.3);
                        }
                        else
                        {
                            arm.set(-armspeed);
                        }
                    }
                    arm.set(0.05);
                    intake.set(1);
                    LED.set(Relay.Value.kReverse);
                    Timer.delay(1);
                    intake.set(0);
                    LED.set(Relay.Value.kOff);
                    arm.set(0);
                    if (useright)
                    {
                        while (rightencoders.get() > c3move4)
                        {
                            tank.tankDrive(0.75, -0.75);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() > c3move4)
                        {
                            tank.tankDrive(-0.75, 0.75);
                        }
                    }
                    tank.tankDrive(0, 0);
                    if (useright)
                    {
                        while (rightencoders.get() < c3move5)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c3move5)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    LED.set(Relay.Value.kForward);
                    Timer.delay(0.5);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    Timer.delay(1);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    LED.set(Relay.Value.kOff);
                    autonselected = 9;
                    break;
                }
                    //</editor-fold>
                case 4:
                    //<editor-fold defaultstate="collapsed" desc="4 Disc Front">
                {
                    //angler starts at 30 degrees
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    shooter.set(shooterspeed);
                    Timer.delay(5);
                    LED.set(Relay.Value.kForward);
                    while (potentiometer.getValue()/Robomap.potratio > Robomap.armgroundangle + Robomap.armtol)
                    {
                        arm.set(armspeed);
                    }
                    arm.set(0);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    LED.set(Relay.Value.kOff);
                    intake.set(1);
                    if (useright)
                    {
                        while (rightencoders.get() < c4move1)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c4move1)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    intake.set(0);
                    if (useright)
                    {
                        while (rightencoders.get() > c4move2)
                        {
                            tank.tankDrive(-0.5, -0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() > c4move2)
                        {
                            tank.tankDrive(-0.5, -0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    LED.set(Relay.Value.kForward);
                    angler.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(2);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    LED.set(Relay.Value.kOff);
                    autonselected = 9;
                    break;
                }
                    //</editor-fold>
                case 5:
                    //<editor-fold defaultstate="collapsed" desc="3 Disc Inside Back">
                {
                    //angler starts at 30 degrees
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    shooter.set(shooterspeed);
                    Timer.delay(5);
                    LED.set(Relay.Value.kForward);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    LED.set(Relay.Value.kOff);
                    autonselected = 9;
                    break;
                }
                    //</editor-fold>
                case 6:
                    //<editor-fold defaultstate="collapsed" desc="3 Disc Outside Back">
                {
                    //angler starts at 30 degrees
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    shooter.set(shooterspeed);
                    Timer.delay(5);
                    LED.set(Relay.Value.kForward);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    LED.set(Relay.Value.kOff);
                    autonselected = 9;
                    break;
                }
                    //</editor-fold>
                case 7:
                    //<editor-fold defaultstate="collapsed" desc="3 Disc Outside 2 Disc Dunk">
                {
                    //angler starts at 30 degrees
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    shooter.set(shooterspeed);
                    Timer.delay(5);
                    LED.set(Relay.Value.kForward);
                    angler.set(DoubleSolenoid.Value.kReverse);
                    Timer.delay(0.5);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    Timer.delay(0.5);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    LED.set(Relay.Value.kReverse);
                    intake.set(1);
                    if (useright)
                    {
                        while (rightencoders.get() < c7move1)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c7move1)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    intake.set(0);
                    LED.set(Relay.Value.kOff);
                    if (useright)
                    {
                        while (rightencoders.get() > c7move2)
                        {
                            tank.tankDrive(0.75, -0.75);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() > c7move2)
                        {
                            tank.tankDrive(-0.75, 0.75);
                        }
                    }
                    tank.tankDrive(0, 0);
                    if (useright)
                    {
                        while (rightencoders.get() < c7move3)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c7move3)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    if (useright)
                    {
                        while (rightencoders.get() < c7move4)
                        {
                            tank.tankDrive(-0.75, 0.75);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c7move4)
                        {
                            tank.tankDrive(0.75, -0.75);
                        }
                    }
                    tank.tankDrive(0, 0);
                    if (useright)
                    {
                        while (rightencoders.get() < c7move5)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c7move5)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    while (potentiometer.getValue()/Robomap.potratio < Robomap.armforwardangle - Robomap.armtol)
                    {
                        if (potentiometer.getValue()/Robomap.potratio > Robomap.armforwardangle - Robomap.ArmSlowTol)
                        {
                            arm.set(-armspeed * 0.3);
                        }
                        else
                        {
                            arm.set(-armspeed);
                        }
                    }
                    arm.set(0.05);
                    intake.set(-1);
                    LED.set(Relay.Value.kReverse);
                    Timer.delay(1);
                    intake.set(0);
                    LED.set(Relay.Value.kOff);
                    autonselected = 9;
                    break;
                }
                    //</editor-fold>
                case 8:
                    //<editor-fold defaultstate="collapsed" desc="2 Disc Front">
                {
                    //angler starts at 36 degrees
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    shooter.set(shooterspeed);
                    Timer.delay(5);
                    LED.set(Relay.Value.kForward);
                    Timer.delay(1);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    Timer.delay(0.5);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    Timer.delay(0.75);
                    indexer.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.25);
                    indexer.set(DoubleSolenoid.Value.kReverse);
                    indexerservo1.setAngle(110);
                    indexerservo2.setAngle(65);
                    Timer.delay(0.25);
                    indexerservo1.setAngle(0);
                    indexerservo2.setAngle(0);
                    LED.set(Relay.Value.kOff);
                    autonselected = 9;
                    break;
                }
                    //</editor-fold>
                case 9:
                    //<editor-fold defaultstate="collapsed" desc="Herp Derp">
                    {
                    tank.tankDrive(0, 0);
                    arm.set(0);
                    autonreset = false;
                    break;
                    }
                    //</editor-fold>
                case 10:
                    //<editor-fold defaultstate="collapsed" desc="2 Disc Dunk">
                {
                    if (useright)
                    {
                        while (rightencoders.get() < c10move1)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    else
                    {
                        while (-leftencoders.get() < c10move1)
                        {
                            tank.tankDrive(0.5, 0.5);
                        }
                    }
                    tank.tankDrive(0, 0);
                    intake.set(-1);
                    LED.set(Relay.Value.kReverse);
                    Timer.delay(1);
                    intake.set(0);
                    LED.set(Relay.Value.kOff);
                    autonselected = 9;
                    break;
                }
                    //</editor-fold>
                default:
                    //<editor-fold defaultstate="collapsed" desc="Default (no auton selected)">
                {
                    tank.tankDrive(0, 0); //stops movement in the robot
                    arm.set(0);
                    autonreset = false;
                    break;
                }
                    //</editor-fold>
            }
            }
            
        //</editor-fold>
        }
    }

    public void teleopPeriodic() {
        
//<editor-fold defaultstate="collapsed" desc="<---RESET AUTON--->">
        //only do this when first starting teleoperation
                if(autonreset == false)
                {
                    leftencoders.reset();
                    rightencoders.reset();
                    shooterencoder.reset();
                    autonselected = Robomap.autonselected;
                    autonreset = true;
                }

//</editor-fold>
        //as long as in teleop, do this stuff below
        while(isOperatorControl() && isEnabled())
        {
//<editor-fold defaultstate="collapsed" desc="<---SMARTDASHBOARD--->">
            SmartDashboard.putNumber("Shooter PID Kp", Robomap.Kp);
            SmartDashboard.putNumber("Shooter PID Ki", Robomap.Ki);
            SmartDashboard.putNumber("Arm PID Kp", Robomap.armKp);
            SmartDashboard.putNumber("Arm PID Ki", Robomap.armKi);
            SmartDashboard.putNumber("Right Encoder", rightencoders.get());
            SmartDashboard.putNumber("Left Encoder", -leftencoders.get());
            SmartDashboard.putNumber("Potentiometer", potentiometer.getValue()/Robomap.potratio);
            SmartDashboard.putNumber("Shooter Encoder Rate", rate);
            SmartDashboard.putNumber("Shooter PID Total Error", eT);
            SmartDashboard.putNumber("Shooter PID Error", et);
            SmartDashboard.putNumber("Shooter Output", MV);
            SmartDashboard.putNumber("Pout", Pout);
            SmartDashboard.putNumber("Iout", Iout);
            SmartDashboard.putNumber("Shooter Encoder Total", shooterencoder.get());
            SmartDashboard.putBoolean("Auton Reset?", autonreset);
            SmartDashboard.putString("Arm Manual Override", armoverridestatus);
            SmartDashboard.putString("Shooter Manual Override", shooteroverridestatus);
            /*Robomap.Kp = SmartDashboard.getNumber("Shooter PID Kp", Robomap.Kp);
            Robomap.Ki = SmartDashboard.getNumber("Shooter PID Ki", Robomap.Ki);
            Robomap.armKp = SmartDashboard.getNumber("Arm PID Kp", Robomap.armKp);
            Robomap.armKi = SmartDashboard.getNumber("Arm PID Ki", Robomap.armKi);*/
            //the above was used to edit variables in smartdashboard while robot is active, may not 
            //work because it was telling it to both read and set the same variable (creates conflict maybe?)
            //</editor-fold>

//<editor-fold defaultstate="collapsed" desc="<---DRIVER CONTROLLER--->">
            
            
//<editor-fold defaultstate="collapsed" desc="<---ARM MANUAL OVERRIDE--->">
                    /*if (drivercontroller.getRawButton(9))
                    {
                        armmanualoverride = true;
                        armoverridestatus = "Enabled";
                    }
                    else if (drivercontroller.getRawButton(10))
                    {
                        armmanualoverride = false;
                        armoverridestatus = "Disabled";
                    }*/
                //</editor-fold>
                
//<editor-fold defaultstate="collapsed" desc="<---DRIVE CONTROL--->">
                    
            //lower drive speed for percision
                    if (drivercontroller.getRawButton(7))
                    {
                        DriveSpeed = 0.75;
                    }
                    else if (drivercontroller.getRawButton(8))
                    {
                        DriveSpeed = 0.5;
                    }
                    else
                    {
                        DriveSpeed = Robomap.Drivespeed;
                    }
                    
                    //Prevent overcurrent when switching from max speed in one direction to the other
                    driveoutleft = -drivercontroller.getRawAxis(2)*DriveSpeed;
                    driveoutright = -drivercontroller.getRawAxis(4)*DriveSpeed;
                    //left drive
                    if (driveoutleft > 0.9 && driveoutleftpre < -0.9)
                    {
                        driveoutleft = 0.75;
                    }
                    else if (driveoutleft < -0.9 && driveoutleftpre > 0.9)
                    {
                        driveoutleft = -0.75;
                    }
                    
                    //right drive
                    if (driveoutright > 0.9 && driveoutrightpre < -0.9)
                    {
                        driveoutright = 0.75;
                    }
                    else if (driveoutright < -0.9 && driveoutrightpre > 0.9)
                    {
                        driveoutright = -0.75;
                    }
                    tank.tankDrive((driveoutleft), (driveoutright));
                    driveoutleftpre = (driveoutleft);
                    driveoutrightpre = (driveoutright);
                    //tells which directions on the joystick do what and calibrates based on the drivespeed
                //</editor-fold>
                
//<editor-fold defaultstate="collapsed" desc="<---LIFTER--->">
                    if (drivercontroller.getRawButton(2))
                    {
                        lifter.set(DoubleSolenoid.Value.kReverse);
                        //retracts the pneumatics for the climber
                    }
                    else if (drivercontroller.getRawButton(4))
                    {
                        lifter.set(DoubleSolenoid.Value.kForward);
                        //extends the pneumatics for the climber
                    }
                //</editor-fold>
                    
//<editor-fold defaultstate="collapsed" desc="<---WINCH--->">
                    if (drivercontroller.getRawButton(6))
                    {
                        winch.set(1);
                    }
                    else
                    {
                        winch.set(0);
                    }
                //</editor-fold>
                    
//</editor-fold>
                
//<editor-fold defaultstate="collapsed" desc="<--- OPERATOR CONTROLLER--->">
                
           
//<editor-fold defaultstate="collapsed" desc="<---SHOOTER MANUAL OVERRIDE--->">
            if (operatorcontroller.getRawButton(9))
            {
                shootermanualoverride = true;
            }
            else if (operatorcontroller.getRawButton(10))
            {
                shootermanualoverride = false;
            }
            if (shootermanualoverride)
            {
                shooteroverridestatus = "Enabled";
            }
            else
            {
                shooteroverridestatus = "Disabled";
            }
//</editor-fold>
                    
//<editor-fold defaultstate="collapsed" desc="<---ARM INTAKES--->">
               
                    //setting intake rollers for arm to pull in a disc
                    if (operatorcontroller.getRawButton(7) /*&& ((potentiometer.getValue()/Robomap.potratio < 5) || 
                            (potentiometer.getValue()/Robomap.potratio > Robomap.armforwardangle - Robomap.armtol && potentiometer.getValue()/Robomap.potratio < Robomap.armforwardangle + Robomap.armtol) ||
                            (potentiometer.getValue()/Robomap.potratio > Robomap.armbackangle - Robomap.armtol && potentiometer.getValue()/Robomap.potratio < Robomap.armbackangle + Robomap.armtol) ||
                            (armmanualoverride))*/)
                    {
                        intake.set(1);
                        LED.set(Relay.Value.kReverse);
                    }
                    //set intake rollers to push disc out
                    else if(operatorcontroller.getRawButton(8))
                    {
                        intake.set(-1);
                        LED.set(Relay.Value.kReverse);
                    }
                    // if neither button is being pushed turn off intake rollers
                    else 
                    {
                        intake.set(0);
                        LED.set(Relay.Value.kOff);
                    }
                //</editor-fold>
                
//<editor-fold defaultstate="collapsed" desc="<---ARM--->">
                    if (armmanualoverride)
                        {   //when manual override is enabled, this completely dissabled the potentiometer readings. 
                            if (Math.abs(operatorcontroller.getRawAxis(4)) >= 0.05) //only do this if the joystick is being toggled
                            {
                                if (limitswitchmax.get()) //arm is at the max limit, prevent it from going further
                                {
                                    if (-operatorcontroller.getRawAxis(4) < 0)
                                    {
                                        arm.set(0);
                                    }
                                    else
                                    {
                                        arm.set(-operatorcontroller.getRawAxis(4)*armspeed); 
                                    }
                                }
                                else if (limitswitchmin.get()) //arm is at the min limit, prevent it from going further
                                {
                                    if (-operatorcontroller.getRawAxis(4) > 0)
                                    {
                                        arm.set(0);
                                    }
                                    else
                                    {
                                        arm.set(-operatorcontroller.getRawAxis(4)*armspeed);
                                    }
                                }
                                else //arm is free to move either way
                                {
                                    arm.set(-operatorcontroller.getRawAxis(4)*armspeed);
                                }
                            }
                            else //joystick is not being toggled, set arm to zero
                            {
                                arm.set(0);
                            }
                        }
                    else
                    {
                    //(1=back, 2=ground, 3=forward,4=up)
                    //arm movement via straight potentiometer value
                    //if the potentiometer is reading less than the desired angle, arm goes up
                    //if potentiometer is greater, arm goes down. A tolerance is added to each angle
                    //to provide a sweetspot; if this is met, arm stops moving.
                    if (operatorcontroller.getRawButton(1)) //back position
                    {
                        lastup = false;
                        if ((potentiometer.getValue()/Robomap.potratio) < (Robomap.armbackangle - Robomap.armtol))
                        {
                            if (potentiometer.getValue()/Robomap.potratio > Robomap.armbackangle - Robomap.ArmSlowTol)
                            {
                                arm.set(-armspeed*0.3);
                            }
                            else
                            {
                                arm.set(-armspeed);
                            }
                        }
                        else if ((potentiometer.getValue()/Robomap.potratio) > (Robomap.armbackangle + Robomap.armtol))
                        {
                            if (potentiometer.getValue()/Robomap.potratio < Robomap.armbackangle + Robomap.ArmSlowTol)
                            {
                                arm.set(armspeed*0.3);
                            }
                            else
                            {
                                arm.set(armspeed);
                            }
                        }
                        else
                        {
                            arm.set(-0.05);
                        }
                    }
                    else if (operatorcontroller.getRawButton(2)) //ground position
                    {
                        lastup = false;
                        if ((potentiometer.getValue()/Robomap.potratio) > (Robomap.armgroundangle + Robomap.armtol))
                        {
                            arm.set(armspeed);
                        }
                        else
                        {
                            arm.set(0);
                        }
                    }
                    else if (operatorcontroller.getRawButton(3)) //forward position
                    {
                        lastup = false;
                        if ((potentiometer.getValue()/Robomap.potratio) < (Robomap.armforwardangle - Robomap.armtol))
                        {
                            if (potentiometer.getValue()/Robomap.potratio > Robomap.armforwardangle - Robomap.ArmSlowTol)
                            {
                                arm.set(-armspeed*0.3);
                            }
                            else
                            {
                                arm.set(-armspeed);
                            }
                        }
                        else if ((potentiometer.getValue()/Robomap.potratio) > (Robomap.armforwardangle + Robomap.armtol))
                        {
                            if (potentiometer.getValue()/Robomap.potratio < Robomap.armforwardangle + Robomap.ArmSlowTol)
                            {
                                arm.set(armspeed*0.3);
                            }
                            else
                            {
                                arm.set(armspeed);
                            }
                        }
                        else
                        {
                            arm.set(0.05);
                        }
                    }
                    else if (operatorcontroller.getRawButton(4)) //up position
                    {
                        lastup = true;
                        if ((potentiometer.getValue()/Robomap.potratio) < (Robomap.armupangle - Robomap.armtol))
                        {
                            if (potentiometer.getValue()/Robomap.potratio > Robomap.armupangle - Robomap.ArmSlowTol)
                            {
                                arm.set(-armspeed*0.3);
                            }
                            else
                            {
                                arm.set(-armspeed);
                            }
                        }
                        else if ((potentiometer.getValue()/Robomap.potratio) > (Robomap.armupangle + Robomap.armtol))
                        {
                            if (potentiometer.getValue()/Robomap.potratio < Robomap.armupangle + Robomap.ArmSlowTol)
                            {
                                arm.set(armspeed*0.3);
                            }
                            else
                            {
                                arm.set(armspeed);
                            }
                        }
                        else
                        {
                            arm.set(0);
                        }
                    }
                    else if (lastup)
                    {
                        if ((potentiometer.getValue()/Robomap.potratio) > (Robomap.armupangle + Robomap.armtol))
                        {
                            arm.set(0.05);
                        }
                        else
                        {
                            arm.set(-0.05);
                        }
                    }
                    else //if no input, turn off the arm
                    {
                        arm.set(0);
                    }
                }
                //</editor-fold>
                    
//<editor-fold defaultstate="collapsed" desc="<---SHOOTER--->">
                    if (shootermanualoverride)
                    {
                        if (operatorcontroller.getRawButton(5) && operatorcontroller.getRawButton(6))
                        {
                            shooter.set(shooterspeed);
                            indexer.set(DoubleSolenoid.Value.kForward);
                            if (aftertap) //delay shooter trigger from returning until full stroke
                            {
                                shooterdelay = 0;
                                servodelay = 0;
                            }
                            aftertap = false;
                            if (shooterdelay < Robomap.ShooterDelay)
                            {
                                shooterdelay = shooterdelay + 1;
                            }
                            LED.set(Relay.Value.kForward);
                        }
                        else if (operatorcontroller.getRawButton(6)) //manually prime the servo
                        {
                            if (aftertap) 
                            {
                                servodelay = 0;
                            }
                            aftertap = false;
                            
                        }
                        else if (operatorcontroller.getRawButton(5))
                        {
                            shooter.set(shooterspeed);
                            if (shooterdelay < Robomap.ShooterDelay)
                            {
                                shooterdelay = shooterdelay + 1;
                            }

                            if (shooterdelay >= Robomap.ShooterDelay)
                            {
                                indexer.set(DoubleSolenoid.Value.kReverse);
                                
                                if (servodelay < Robomap.ServoDelay)
                                {
                                    indexerservo1.setAngle(110);
                                    indexerservo2.setAngle(65);
                                    servodelay = servodelay + 1;
                                }
                                else if(servodelay >= Robomap.ServoDelay)
                                {
                                    indexerservo1.setAngle(0);
                                    indexerservo2.setAngle(0);
                                }
                            }
                            aftertap = true;
                            LED.set(Relay.Value.kForward);
                        }
                        else
                        {
                            shooter.set(0);
                            if (shooterdelay < Robomap.ShooterDelay)
                            {
                                shooterdelay = shooterdelay + 1;
                            }

                            if (shooterdelay >= Robomap.ShooterDelay)
                            {
                                indexer.set(DoubleSolenoid.Value.kReverse);
                                
                                if (servodelay < Robomap.ServoDelay)
                                {
                                    indexerservo1.setAngle(110);
                                    indexerservo2.setAngle(65);
                                    servodelay = servodelay + 1;
                                }
                                else if(servodelay >= Robomap.ServoDelay)
                                {
                                    indexerservo1.setAngle(0);
                                    indexerservo2.setAngle(0);
                                }
                            }
                            aftertap = true;
                            LED.set(Relay.Value.kOff);
                        }
                    }
                    else
                    {
                        if (operatorcontroller.getRawButton(5) && operatorcontroller.getRawButton(6))
                        {
                        //Wheel spin
                            //next if statements will set shooter output to the limits of motor capacity if it exceeds it
                            if (MV > 1)
                            {
                                MV = 1;
                            }
                            else if (MV < -1)
                            {
                                MV = -1;
                            }
                            shooter.set (MV);                       //sets the shooter to the output value
                            current = shooterencoder.get();
                            rate = current - previous;
                            previous = current;
                            PV = rate;                              //gets the rate that the encoder is spinning at
                                et = (Robomap.SP - PV);             //calculates new error based on setpoint and measured input
                                eT = (et + eT);                     //calculates total error based on current error and previous error
                                Pout = (Robomap.Kp * et);           //multiplies the proportional gain by current error
                                if (PV > 60 && PV < 62)
                                {
                                Iout = (Robomap.Ki * eT);           //Ki * integral from 0 to t (a to b)~~(b-a)*((f(a)+f(b))/2)(f(x)=e(T)=eT)
                                }
                                else
                                {
                                    Iout = 0;
                                }
                                MV = (Pout + Iout);                 //adds outputs from proportional and integral calculations together to get new output
                            
                        //shoot disk and turn on light    
                            indexer.set(DoubleSolenoid.Value.kForward);
                            if (aftertap)
                            {
                                shooterdelay = 0;
                                servodelay = 0;
                            }
                            aftertap = false;
                            if (shooterdelay < Robomap.ShooterDelay)
                            {
                                shooterdelay = shooterdelay + 1;
                            }

                            LED.set(Relay.Value.kForward);
                            
                        }
                        else if(operatorcontroller.getRawButton(5))
                        {
                            //next if statements will set shooter output to the limits of motor capacity if it exceeds it
                            if (MV > 1)
                            {
                                MV = 1;
                            }
                            else if (MV < -1)
                            {
                                MV = -1;
                            }
                            shooter.set (MV);                       //sets the shooter to the output value
                            current = shooterencoder.get();
                            rate = current - previous;
                            previous = current;
                            PV = rate;                              //gets the rate that the encoder is spinning at
                                et = (Robomap.SP - PV);             //calculates new error based on setpoint and measured input
                                eT = (et + eT);                           //calculates total error based on current error and previous error
                                Pout = (Robomap.Kp * et);           //multiplies the proportional gain by current error
                                if (PV > 60 && PV < 62)
                                {
                                Iout = (Robomap.Ki * eT);           //Ki * integral from 0 to t (a to b)~~(b-a)*((f(a)+f(b))/2)(f(x)=e(T)=eT)
                                }
                                else
                                {
                                    Iout = 0;
                                }
                                MV = (Pout + Iout);          //adds outputs from proportional and integral calculations together to get new output
                            
                            //primes wheel for shooting, resets pneumatic for reloading
                            if (shooterdelay < Robomap.ShooterDelay)
                            {
                                shooterdelay = shooterdelay + 1;
                            }

                            if (shooterdelay >= Robomap.ShooterDelay)
                            {
                                indexer.set(DoubleSolenoid.Value.kReverse);
                                
                                if (servodelay < Robomap.ServoDelay)
                                {
                                    indexerservo1.setAngle(110);
                                    indexerservo2.setAngle(65);
                                    servodelay = servodelay + 1;
                                }
                                else if(servodelay >= Robomap.ServoDelay)
                                {
                                    indexerservo1.setAngle(0);
                                    indexerservo2.setAngle(0);
                                }
                            }
                            LED.set(Relay.Value.kForward);
                            aftertap = true;
                            
                        }
                        else
                        {
                            shooter.set(0);
                            if (shooterdelay < Robomap.ShooterDelay)
                            {
                                shooterdelay = shooterdelay + 1;
                            }

                            if (shooterdelay >= Robomap.ShooterDelay)
                            {
                                indexer.set(DoubleSolenoid.Value.kReverse);
                                
                                if (servodelay < Robomap.ServoDelay)
                                {
                                    indexerservo1.setAngle(110);
                                    indexerservo2.setAngle(65);
                                    servodelay = servodelay + 1;
                                }
                                else if(servodelay >= Robomap.ServoDelay)
                                {
                                    indexerservo1.setAngle(0);
                                    indexerservo2.setAngle(0);
                                }
                            }
                            LED.set(Relay.Value.kOff);
                            aftertap = true;
                            eT = 0;
                            et = 0;
                            MV = 0;
                            Pout = 0;
                            Iout = 0;
                        }
                    }
                    
                    
//</editor-fold>
                    
//<editor-fold defaultstate="collapsed" desc="<---SHOOTER ANGLER--->">
                    //changes the angle of the shooter
                    //kForward sets it to high-shots (close distance)
                    //kReverse sets it to low-shots (far away)
                    if (operatorcontroller.getRawAxis(2) < -0.9)
                    {
                        angler.set(DoubleSolenoid.Value.kForward);
                    }
                    else if (operatorcontroller.getRawAxis(2) > 0.9)
                    {
                        angler.set(DoubleSolenoid.Value.kReverse);
                    }

//</editor-fold>
                    
//</editor-fold>
        }
    }

    public void Disabled() {
  
        while(isDisabled())
        {
            //makes everything stop
                tank.stopMotor();
                intake.stopMotor();
                winch.stopMotor();
                shooter.stopMotor();
                arm.stopMotor();
                //indexerservo1.set(0);
               // indexerservo2.set(0);
                leftencoders.reset();
                rightencoders.reset();
                shooterencoder.reset();
                autonselected = Robomap.autonselected;
                autonreset = true;
        }
    }

    public void testPeriodic() {
//        SmartDashboard.putNumber("Shooter PID Kp", Robomap.Kp);
//        SmartDashboard.putNumber("Shooter PID Ki", Robomap.Ki);
//        SmartDashboard.putNumber("Arm PID Kp", Robomap.armKp);
//        SmartDashboard.putNumber("Arm PID Ki", Robomap.armKi);
//        SmartDashboard.putNumber("Right Encoder", rightencoders.get());
//        SmartDashboard.putNumber("Left Encoder", -leftencoders.get());
//        SmartDashboard.putNumber("Potentiometer", potentiometer.getValue());
//        SmartDashboard.putNumber("Shooter Encoder Rate", shooterencoder.getRate());
//        SmartDashboard.putNumber("Shooter Encoder Rate", shooterencoder.getRate());
//        SmartDashboard.putBoolean("Auton Reset?", autonreset);
//        SmartDashboard.putString("Arm Manual Override", armoverridestatus);
        
        if(operatorcontroller.getRawButton(1))
        {
            indexerservo1.setAngle(110);
            indexerservo2.setAngle(65);
        }
        else if(operatorcontroller.getRawButton(2))
        {
            indexerservo1.setAngle(110);
            indexerservo2.setAngle(65);
        }
        else if(operatorcontroller.getRawButton(3))
        {
            indexerservo1.setAngle(100);
            indexerservo2.setAngle(100);
        }
        else if(operatorcontroller.getRawButton(4))
        {
            indexerservo1.setAngle(0);
            indexerservo2.setAngle(0);
        }
        else if(operatorcontroller.getRawButton(5))
        {
            indexerservo1.setAngle(120);
            indexerservo2.setAngle(120);
        }
        else if(operatorcontroller.getRawButton(6))
        {
            indexerservo1.setAngle(180);
            indexerservo2.setAngle(180);
        }
        else
        {
            indexerservo1.setAngle(110);
            indexerservo2.setAngle(65); 
        }
    }

}