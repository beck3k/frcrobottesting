#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>

#include <XboxJoystickMap.h>
#include <Robot.h>
#include <WPILib.h>

class Robot: public frc::IterativeRobot {
public:

	Victor* frontMotorL;
	Victor* frontMotorR;
	Victor* backMotorL;
	Victor* backMotorR;

	Solenoid* fire0;
	Solenoid* fire1;
	Solenoid* fire2;
	Solenoid* fire3;

	Spark* rollerIntake;

	ADXRS450_Gyro gyro;

	enum Drive {
		frontMotorLPWM = 0,
		frontMotorRPWM = 2,
		backMotorLPWM = 1,
		backMotorRPWM = 3
	};

	Joystick* joystick1;

	RobotDrive* driveTrain;

	DoubleSolenoid* arm;

	double gyroInit;

	int intakeRollerPWM = 4;

	enum ArmSolenoids {
		Sol0 = 6,
		Sol1 = 7
	};

	enum ArmDirection{
		Down,
		Up
	};

	Ultrasonic* ultrasonic;

	void RobotInit() {
		joystick1 = new Joystick(0);

		frontMotorL = new Victor(Drive::frontMotorLPWM);
		frontMotorR = new Victor(Drive::frontMotorRPWM);
		backMotorL = new Victor(Drive::backMotorLPWM);
		backMotorR = new Victor(Drive::backMotorRPWM);

		driveTrain = new RobotDrive(frontMotorL, backMotorL, frontMotorR, backMotorR);

		arm = new DoubleSolenoid(ArmSolenoids::Sol0, ArmSolenoids::Sol1);

		rollerIntake = new Spark(intakeRollerPWM);

		fire0 = new Solenoid(FIRE0_PWM);
		fire1 = new Solenoid(FIRE1_PWM);
		fire2 = new Solenoid(FIRE2_PWM);
		fire3 = new Solenoid(FIRE3_PWM);

		ultrasonic = new Ultrasonic(SONIC0_P0, SONIC0_P1);
		ultrasonic->SetAutomaticMode(true);

		gyroInit = gyro.GetAngle();
	}

	void AutonomousInit() override {
//		frontMotorL = new Victor(Drive::frontMotorLPWM);
//		frontMotorR = new Victor(Drive::frontMotorRPWM);
//		backMotorL = new Victor(Drive::backMotorLPWM);
//		backMotorR = new Victor(Drive::backMotorRPWM);

		//driveTrain = new RobotDrive(frontMotorL, backMotorL, frontMotorR, backMotorR);
	}

	void UltrasonicPeriodic(){
		bool canDrive = true;
		bool distance = ultrasonic->GetRangeInches();
		if(ultrasonic->GetRangeInches() <= 85){
			canDrive = false;
		}else{
			canDrive = true;
		}

		if(canDrive == true){
			driveTrain->TankDrive(-0.8, -0.8);
		}
	}

	void AutonomousPeriodic() {
		UltrasonicPeriodic();

		SmartDashboard::PutNumber("Ultrasonic", ultrasonic->GetRangeInches());
	}

	void TeleopInit() {

	}

	double JoystickAxis(Joystick* j, int a){
		return j->GetRawAxis(a);
	}

	void FireSet(double f){
		fire0->Set(f);
		fire1->Set(f);
		fire2->Set(f);
		fire3->Set(f);
	}

	DoubleSolenoid::Value returnArmDirection(ArmDirection d){
		DoubleSolenoid::Value action;
		switch(d){
		case ArmDirection::Down:
			action = DoubleSolenoid::kReverse;
			break;
		case ArmDirection::Up:
			action = DoubleSolenoid::kForward;
			break;
		default:
			action = DoubleSolenoid::kForward;
			break;
		}
		return action;
	}

	void DriveTrainPeriodic(){
		Joystick* j = joystick1;
		driveTrain->ArcadeDrive(JoystickAxis(j, XboxAxisLeftStickY), JoystickAxis(j, XboxAxisRightStickY));
	}

	void IntakePeriodic(){
		if(joystick1->GetRawButton(XboxButtonY)){
			arm->Set(returnArmDirection(ArmDirection::Down));
			rollerIntake->Set(ROLLER_MAX);
		}else{
			arm->Set(returnArmDirection(ArmDirection::Up));
			rollerIntake->Set(ROLLER_NONE);
		}
	}

	void FirePeriodic(){
		FireSet(joystick1->GetRawButton(XboxButtonRightBumper));
	}

	void TeleopPeriodic() {
		DriveTrainPeriodic();
		IntakePeriodic();
		FirePeriodic();

		SmartDashboard::PutNumber("Ultrasonic", ultrasonic->GetRangeInches());
		SmartDashboard::PutNumber("Gyro", (gyro.GetAngle() - gyroInit) % (double) 360);
	}

	void TestPeriodic() {
	}


};

START_ROBOT_CLASS(Robot)
