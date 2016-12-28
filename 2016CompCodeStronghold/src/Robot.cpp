#include "RobotDrive.h"
#include "Encoder.h"
#include "CANTalon.h"
#include "Analogpotentiometer.h"
#include "Joystick.h"
#include "IterativeRobot.h"
#include "Timer.h"
#include "CameraServer.h"
#include "DigitalInput.h"
#include "WPILib.h"


class Robot: public IterativeRobot
{

public:
	LiveWindow *lw = LiveWindow::GetInstance();
	CANTalon *front_right_motor, *rear_right_motor, *front_left_motor, *rear_left_motor, *arm, *intake_roller, *shooter;
	Joystick *driver_controller, *operator_controller;
//	AnalogPotentiometer *potcrack;
//	Encoder *left_encoder;
	RobotDrive *first_person_drive;
	Timer *timer;

	DigitalInput *_left_turn, *_right_turn, *_speed_control_1, *_speed_control_2;

    SendableChooser *chooser;

	int m1_a1 = 1400;

	bool completed_run = true;

	int SPEED_Multiplier = 0.25;

	void RobotInit()
	{
		SmartDashboard::PutData("NAH SUn", chooser);
//	    CameraServer::GetInstance()->SetQuality(50);
//
//	    CameraServer::GetInstance()->StartAutomaticCapture("cam0");
		front_left_motor = new CANTalon(1);
		rear_left_motor = new CANTalon(2);
		front_right_motor = new CANTalon(3);
		rear_right_motor = new CANTalon(4);

		intake_roller = new CANTalon(5);
		shooter = new CANTalon(6);
		arm = new CANTalon(7);

//		potcrack = new AnalogPotentiometer(0);
//
//		left_encoder = new Encoder(8, 9);

		driver_controller = new Joystick(0);
		operator_controller = new Joystick(1);

		first_person_drive = new RobotDrive(front_left_motor, rear_left_motor, front_right_motor, rear_right_motor);

		first_person_drive->SetSafetyEnabled(false);

		_left_turn = new DigitalInput(0);
		_right_turn = new DigitalInput(1);// True when cords found. Value found from Rassberry Pi port 27
		_speed_control_1 = new DigitalInput(2);
		_speed_control_2 = new DigitalInput(3);

	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit()
	{
		//left_encoder->Reset();

	}

	void AutonomousPeriodic()
	{
		int autochooser = 4; 				//cheese
		int autocounter = 0;
		while(IsAutonomous() && IsEnabled())
		{

//			double pote = potcrack->Get();
//			int left_encoder_value = left_encoder->Get();
			//SmartDashboard::PutNumber("pote value", pote);
			//SmartDashboard::PutNumber("left encoder value", left_encoder_value);
			switch(autochooser)
			{
//				case 1:
//				{
//					while(autocounter < 900 )
//					{
//						arm->Set(0.65);
//						autocounter++;
//					}
//					arm->Set(0);
//					while(autocounter >= 900 && autocounter <= 1050)
//					{
//						first_person_drive->SetLeftRightMotorOutputs(0, 0);
//					autocounter++;
//					}
//					while(autocounter <= 1650 && autocounter >= 1050)
//					{
//						first_person_drive->SetLeftRightMotorOutputs(0.515, 0.515);
//						autocounter++;
//					}
//					first_person_drive->SetLeftRightMotorOutputs(0, 0);
//					while(autocounter >= 1650 && _right_turn)
//					{
//						first_person_drive->SetLeftRightMotorOutputs(0.45, -0.45);
//						autocounter++;
//					}
//					first_person_drive->SetLeftRightMotorOutputs(0, 0);
//					while(autocounter >= 2050 &&  autocounter <= 2400)
//					{
//						first_person_drive->SetLeftRightMotorOutputs(0.515, 0.515);
//					}
//					first_person_drive->SetLeftRightMotorOutputs(0, 0);
//					while(autocounter >= 1650 && autocounter <= 2250)
//					{
//						intake_roller->Set(-1);
//						autocounter++;
//					}										//cheese
//					intake_roller->Set(0);
//					break;
//
//				}
				//Vision Processing
				case 2:
					//Vision Processing w/ RasPi
				{
					//Arm down for two seconds
					while(autocounter < 200 )
					{
						arm->Set(0.75);
						autocounter++;
					}
					arm->Set(0);
					//robot pause for 2.5 seconds
					while(autocounter >= 200 && autocounter < 450)
					{
						first_person_drive->SetLeftRightMotorOutputs(0, 0);
						autocounter++;
					}
					//Robot forward for 4.5 seconds
					while(autocounter >= 450 && autocounter < 900)
					{
						first_person_drive->SetLeftRightMotorOutputs(0.515, 0.515);
						autocounter++;
					}
					first_person_drive->SetLeftRightMotorOutputs(0, 0);
					//Robot turn for 3 seconds at most
					while(autocounter >= 900 && _right_turn && autocounter < 1250) // Right turn is false when cords not found. Debug: Runs for 3 seconds(change !_right_turn to true)
					{
						first_person_drive->SetLeftRightMotorOutputs(0.45, -0.45);
						autocounter++;
					}
					first_person_drive->SetLeftRightMotorOutputs(0, 0);
//we stopped here
					//Makes sure that autocounter increased to 1250 if not reached in turn loop
//					while(autocounter >= 900 && autocounter < 1250 && _right_turn)
//					{
//						first_person_drive->SetLeftRightMotorOutputs(0, 0);
//						autocounter++;
//					}

					//Robot forward for 2.5 seconds
					while(autocounter >= 1250 && autocounter < 1650)
					{
						first_person_drive->SetLeftRightMotorOutputs(0.515, 0.515);
						autocounter++;
					}
					first_person_drive->SetLeftRightMotorOutputs(0, 0);

					while(autocounter >= 1650 && autocounter <= 2050)
					{
						arm->Set(-.75);
						autocounter++;
					}
					arm->Set(0);
					//Out take for 1 second
					while(autocounter >= 1750 && autocounter < 2250)
					{
						intake_roller->Set(-1);
						autocounter++;
					}										//cheese
					intake_roller->Set(0);
					break;
				}
				//Drive forward, arm down
				case 3:
				{
					SmartDashboard::PutBoolean("Pi", _right_turn);
					break;
				}
				case 4:
				{
					//BACK
					// auton for going over most defense that only require drivetrain
					while(autocounter < 100)
					{
						first_person_drive->SetLeftRightMotorOutputs(-0.5, -0.5);
						autocounter++;
					}
					first_person_drive->SetLeftRightMotorOutputs(0,0);
					while(autocounter <= 300 && autocounter >= 100)
					{
						arm->Set(0.75);
						autocounter++;
					}
					arm->Set(0);
					while(autocounter >= 300 && autocounter <= 650)
					{
						first_person_drive->SetLeftRightMotorOutputs(0, 0);
						autocounter++;
					}
					while(autocounter < 1100 && autocounter >= 650) // -3 seconds Jason:3-25-16
					{
						first_person_drive->SetLeftRightMotorOutputs(-0.650, -0.650);
						autocounter++;
					}
					first_person_drive->SetLeftRightMotorOutputs(0, 0);
					break;
				}
			}
		}
	}
	void TeleopPeriodic()
	{

		while(IsOperatorControl() && IsEnabled())
		{
			//int left_encoder_value = left_encoder->Get();

			//SmartDashboard::PutNumber("left encoder value", left_encoder_value);

			first_person_drive->SetLeftRightMotorOutputs(-driver_controller->GetRawAxis(1) + driver_controller->GetRawAxis(2), -driver_controller->GetRawAxis(1) - driver_controller->GetRawAxis(2));

			if(operator_controller->GetRawButton(7))
			{
				intake_roller->Set(1);
			}
			else if(operator_controller->GetRawButton(8))
			{
				intake_roller->Set(-1);
			}
			else
		    {
	 			intake_roller->Set(0);
			}


	      	// PreSetsArm();
	       	// intake_controls();
			arm->Set(.75*operator_controller->GetRawAxis(1));
	        Wait(0.005);
		 }
	}
double Driver_RightJoy_Y(void)
	{
		//return(D_CONTROLLER->GetRawAxis(1));
	}
};
START_ROBOT_CLASS(Robot)
