

package edu.wpi.first.wpilibj.templates;

public class Robomap {
    
    //Quick Variables
    public static final int autonselected = 5;
    public static boolean Useright = true;
    public static final boolean usecommand = false;
    
    //<editor-fold defaultstate="collapsed" desc="Joystick Constants">
    public static final int drivergamepad = 1;
    public static final int operatorgamepad = 2;
    //</editor-fold>
    
    //<editor-fold defaultstate="collapsed" desc="Motor Constants (PWM)">
    public static final int leftmotors = 1;             //jaguar
    public static final int rightmotors = 2;            //jaguar
    public static final int shootermotortal = 3;        //talon
    //public static final int shooterintakemotortal = 4;  //talon
        //slot 4 in PWM reserved in case 2nd minicim motor for shooter intake is needed
    public static final int intakevic = 5;         //victor
    //public static final int intakebackvic = 6;          //victor (no longer in use)
    public static final int ARMtal = 7;                 //talon
    public static final int winchvic = 8;               //victor
    public static final int servoindexer1 = 9;
    public static final int servoindexer2 = 10;
    //</editor-fold>
    
    //<editor-fold defaultstate="collapsed" desc="Digital Constants">
    public static final int leftencoderA = 1;
    public static final int leftencoderB = 2;
    public static final int rightencoderA = 3;
    public static final int rightencoderB = 4;
    public static final int shooterencoderA = 5;
    public static final int shooterencoderB = 6;
    public static final int compressorswitch = 7;
    public static final int limitswitch1 = 8;
    public static final int limitswitch2 = 9;
    //Relay Constants
    public static final int compressorrelay = 1;
    public static final int LEDrelay = 2;
    //</editor-fold>
    
    //<editor-fold defaultstate="collapsed" desc="Solenoid Constants">
    public static final int solenoid1A = 1;
    public static final int solenoid1B = 2;
    public static final int solenoid2A = 3;
    public static final int solenoid2B = 4;
    public static final int solenoid3A = 5;
    public static final int solenoid3B = 6;
    //</editor-fold>
    
    //<editor-fold defaultstate="collapsed" desc="Analog Constants">
    public static final int potmet = 1;
    //</editor-fold>
    
    //<editor-fold defaultstate="collapsed" desc="Arm and Shooter Constants">
    //<editor-fold defaultstate="collapsed" desc="Shooter">
    public static final double SP = 83;   //motor needs to be at 65% power, encoder needs to be calibrated properly to match this rate, 0.00008 s/t or 12500 t/s
    //3000RPM/60seconds*250ticks per second/1000*5 (converts to ticks per 5 miliseconds)
    public static double Kp = 1;            //should be a number between 1 and 100 (start low)
    public static double Ki = 0.001;        //should be a very small fraction ie. 0.001
    public static double ShooterSpeed = 0.85;
    //</editor-fold>
    //<editor-fold defaultstate="collapsed" desc="Arm">
    public static double armKp = 1;
    public static double armKi = 0.001;
    public static final double armtol = 2;
    public static final double armgroundangle = 0;
    public static final double armforwardangle = 75;
    public static final double armupangle = 95;
    public static final double armbackangle = 160;
    //Voltage version (maybe?) 1.08=50 degrees, 2.07=95 degrees, 3.04=140 degrees, 4.05=185 degrees, 0.22=10 degrees, max=250 degrees
    
    public static final double ArmSpeed = 1; //arm rotational speed governer
    public static final double ArmSlowTol = 10;
    
    //</editor-fold>
    //</editor-fold>
    
    //<editor-fold defaultstate="collapsed" desc="Auton Encoder Constants">
    
        //7 Disc
            public static int d7move1 = -155;
            public static int d7move2 = 620;
            public static int d7move3 = 1783;
        //5 Disc
            public static int d5move1 = -155;
            public static int d5move2 = 620;
            public static int d5move3 = 1220;
        //5 Disc Midfield
            public static int dmmove1 = -155;
            public static int dmmove2 = 332;
            public static int dmmove3 = 1727;
            public static int dmmove4 = 1240;
            public static int dmmove5 = 2635;
        //4 Disc Front
            public static int d4move1 = 568;
            public static int d4move2 = 0;
        //3 Disc Inside Back
            //NONE
        //3 Disc Outside Back
            //NONE
        //3 Disc Outside 2 Disc Dunk
            public static int odmove1 = 1783;
            public static int odmove2 = 1539;
            public static int odmove3 = 3244;
            public static int odmove4 = 3366;
            public static int odmove5 = 3676;
        //2 Disc Front
            //NONE
        //Herp Derp
            //NONE
        //2 Disc Dunk
            public static int ddmove1 = 1073;
    //</editor-fold>
    
    //<editor-fold defaultstate="collapsed" desc="Others">
    public static final double Drivespeed = 1;  //drive speed governer

    public static final double axiscamera = 1;
    public static final double potratio = 3.864; //966/250=3.864, 966/360=2.683
    public static final double ShooterDelay = 50;   //delay the shooter indexer from returning to 
                                                        //kReverse for this many iterations of the code. 100 = 0.5 seconds
    public static final double ServoDelay = 50;     //delay for the servo motors on the indexer/hopper array
    //1 foot of travel is 155 encoder clicks
    //</editor-fold>
    
}
