/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// FRC TEAM 4855 ROBOT CODE
// 2019 GAME DESTINATION: DEEP SPACE

// Swerve-bot code: Robot

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.christmas.2012.ben10watch.used;
// import com.nintendo.gameboy.pikachu;

public class Robot extends TimedRobot {
			
	// CONTROL CONSTANTS

	final double CONTROL_SPEEDREDUCTION = 2; 	  			// teleop drivetrain inputs are divided by this number when turbo is NOT engaged
	final double CONTROL_DEADZONE = 0.21;       			// minimum value before joystick inputs will be considered
	final double CONTROL_LIFT_DEADZONE = 0.3;   			// minimum value before joystick inputs will be considered on the lift
	final double CONTROL_LIFT_PRECISION_FACTOR = 2; 		// lift input is divided by this value when in precision mode
	final double CONTROL_INTAKE_DEADZONE = .15;				// minimum value before trigger inputs will be considered on intake wheels
	final double CONTROL_PIVOT_DEADZONE = .06;				// minimum value before joystick inputs will be considered on the pivot arm
	final double CONTROL_FOOTWHEEL_DEADZONE = .12;			// minimum value before trigger inputs will be considered on foot wheels
	final static double CONTROL_LIFTLEVEL1 = 590;			// level 1 encoder value
	final static double CONTROL_LIFTLEVEL2 = 8150;			// level 2 encoder value
	final static double CONTROL_LIFTLEVEL3 = 14500;			// level 3 encoder value

	final double CONTROL_CAM_MOE = 5.2;	          			// margin of lateral error for that alignment process for the limelight
	final double CONTROL_CAM_ANGLETHRESHOLD = 4;			// the limelight allows the robot to be +- this value off from its target angle and still call it good
	final boolean CONTROL_CAM_INTERRUPTRECOVERY = true;		// whether or not interruption recovery should be enabled or not
	final static int CONTROL_CAM_INTERRUPTRECOVERYTIME = 30;// the maximum amount of time to give the camera to recover its reading on the target
	final double CONTROL_CAM_ANGLEPIDANGLE = .01;			// the proportional value for adjusting the angle during the angular phase
	final double CONTROL_CAM_STRAFEPIDANGLE = .002;			// the proportional value for adjusting the angle during the strafing phase
	final double CONTROL_CAM_FWDPIDANGLE = .003;			// the proportional value for adjusting the angle during the strafing phase
	final double CONTROL_CAM_STRAFEPIDSTRAFE = .0412;		// the proportional value for strafing during the strafing phase
	final double CONTROL_CAM_FWDANGLECORRECT = 0;			// the robot naturally drifts in this direction when approaching a target, this value corrects that drift
	final double CONTROL_CAM_FWDPIDSTRAFE = .012;			// the proportional value for strafing during the forward phase (originally .017)
	final double CONTROL_CAM_FWDPIDFWD = .1;				// the proportional value for forward motion during the forward phase
	final double CONTROL_CAM_FWDAREATHRESHOLD = 5.8;		// the goal area during the forward phase, used in calculating forward speed
	final double CONTROL_CAM_AREACLEARANCE = 5.1;			// the area that should be read from the limelight to safely say that we've reached the target
	final boolean CONTROL_CAM_VALIDATION = true;			// whether to enable or disable limelight validating its outputs
	final static int CONTROL_CAM_VALIDATIONTIME = 2;		// every x number of steps limelight will validate limelight outputs to ensure that they're sane
	final double CONTROL_CAM_VALIDXSCORE = .1;				// the coefficient used to determine the validation score of limelightX
	final double CONTROL_CAM_VALIDYSCORE = .2;				// the coefficient used to determine the validation score of limelightY
	final double CONTROL_CAM_VALIDASCORE = .6;				// the coefficient used to determine the validation score of limelightArea
	final double CONTROL_CAM_VALIDPSCORE = 2; 				// the coefficient used to determine the validation score of limelightProp
	final static int CONTROL_CAM_VALIDTIMEOUT = 20;			// amount of time the robot will substitute invalid outputs for until it gives up and sticks with the outputs it sees
	final double CONTROL_CAM_VALIDSCORETHRESHOLD = 36;		// maximum acceptable validation score

	final boolean INTERFACE_SINGLEDRIVER = false;  			// whether or not to enable or disable single driver input (press START to switch between controllers)
	//=======================================
	
	// OTHER CONSTANTS

	final static double ROBOT_WIDTH = 29;
	final static double ROBOT_LENGTH = 29;
	final static double ROBOT_R = Math.sqrt(Math.pow(ROBOT_LENGTH,2)+Math.pow(ROBOT_WIDTH,2));
	final static double ENC_TO_DEG = 1.158333;
	final static double ABS_TO_DEG = 11.244444;
	final static double ENC_360 = 417;
	final static double IN_TO_ENC = 10.394;
	// buttons
	final static int BUTTON_A = 1;
	final static int BUTTON_B = 2;
	final static int BUTTON_X = 3;
	final static int BUTTON_Y = 4;
	final static int BUTTON_LB = 5;
	final static int BUTTON_RB = 6;
	final static int BUTTON_SELECT = 7;
	final static int BUTTON_START = 8;
	final static int BUTTON_LSTICK = 9;
	final static int BUTTON_RSTICK = 10;

	// BEGINNING VARIABLES
	
	int wheelTune = 0; 								// Remembers what wheel we are tweaking in test mode
	int singleDriverController = 0; 				// port number of controller to operate
	boolean emergencyTank = false; 					// True if the robot is in emergency tank drive mode
	boolean reverseRotate = false; 					// ?????
	boolean driverOriented = true; 					// true = driver oriented, false = robot oriented
	static double matchTime = 0;					// the calculated match time from the driver station
	// All for calculating wheel speed/angle, if you need to read from a motor don't pull from these
	static double a, b, c, d, max, temp, rads; 
	static double encoderSetpointA, encoderSetpointB, encoderSetpointC, encoderSetpointD;
	static double jStr, jFwd, jRcw;
	static double wheelSpeed1, wheelSpeed2, wheelSpeed3, wheelSpeed4;
	// Gradual starts/stops in teleop
	static double wheelSpeedActual1 = 0, wheelSpeedActual2 = 0, wheelSpeedActual3 = 0, wheelSpeedActual4 = 0;
	static Timer wheelSpeedTimer = new Timer();
	// For limelight
	static boolean limelightActive = true;																	// true if the robot is collecting limelight data and tracking targets
	static boolean limelightSeeking = false;																// true if the limelight is currently seeking a target
	static int limelightPhase = 0;																			// phase for limelight correction, 0=off 1=angular 2=lateral 3=proceed
	static int limelightInterTimer = CONTROL_CAM_INTERRUPTRECOVERYTIME;										// the amount of remaining time for the camera to continue to move if it losts its target
	static double limelightInterX = 0, limelightInterY = 0, limelightInterArea = 0;							// last read values from limelight before an interrupt occurred
	static double limelightValidX = 0, limelightValidY = 0, limelightValidArea = 0, limelightValidProp = 0;	// the last validated values obtained from the limelight
	static int limelightValidTimer = CONTROL_CAM_VALIDATIONTIME;											// every x steps limelight values will be validated
	static int limelightValidTimerTotal = CONTROL_CAM_VALIDTIMEOUT;											// robot will stop trying to validate values after this much time has passed since a rejection
	static boolean limelightInvalidValues = false;															// whether invalid values were found from the limelight
	static double limelightForceAngle = -1;																	// if specific input is given to attempt to guide to a certain angle
	// Operator
	static double pivotSetpoint = 0;						// setpoint for the pivot
	static double liftSetpoint = 0;							// setpoint for the lift
	static boolean liftSetpointControl = false;				// whether the lift is operating on setpoint control or not
	static boolean pivotSetpointControl = true;				// whether the pivot is operating on setpoint control or not
	//=======================================
	
	// LIMELIGHT + DATA TABLES

	NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");							// creates the limelight table
	NetworkTableEntry tx = limelightTable.getEntry("tx");															// the x offset from the crosshairs
	NetworkTableEntry ty = limelightTable.getEntry("ty");															// the y offset from the crosshairs
	NetworkTableEntry ta = limelightTable.getEntry("ta");															// the area (0-100) of the object
	NetworkTableEntry tv = limelightTable.getEntry("tv");															// 1 if object is tracking, 0 if not
	NetworkTableEntry thor = limelightTable.getEntry("thor");														// 0-320 width of the box
	NetworkTableEntry tvert = limelightTable.getEntry("tvert");														// 0-320 height of the box

	NetworkTableEntry tshort = limelightTable.getEntry("tshort");													// 0-320 value of shortest side
	NetworkTableEntry tlong = limelightTable.getEntry("tlong");														// 0-320 value of longest side

	NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");													// 0 for on, 1 for off
	NetworkTableEntry camMode = limelightTable.getEntry("camMode");													// 0 for main, 1 for driver view

	double limelightX, limelightY, limelightArea, limelightWidth, limelightHeight;									// defined in limelightGather
	double limelightEstAngle, limelightGoalAngle, limelightPIDAngle, limelightInitX, limelightROC, limelightProp;	// all used for auto calculations
	static boolean limelightTargetFound = false;																	// set to true if a target is being tracked
	static double limelightInputTimer = -1;																			// this timer acts as a buffer between phases and counts down, -1 when not active
	static int limelightGuideMode;																					// there are 2 guidance modes, the second one is fatally broken so this is usually set to 0
	//=======================================

	// DEFINING HARDWARE

	// Magnetic encoders
	static Encoder encoderAngle[] = {
		new Encoder(0,1),
		new Encoder(2,3),
		new Encoder(6,7),
		new Encoder(4,5)
		
	};

	// Define swerve wheel classes
	static Wheel wheel[] = {
		new Wheel(encoderAngle[0]),
		new Wheel(encoderAngle[1]),
		new Wheel(encoderAngle[2]),
		new Wheel(encoderAngle[3])
	};
	
	// The one CIMcoder on the bot, measures distance traveled
	static Encoder encoderDistance = new Encoder(8,9);	// TODO wire this up and update the ports
	
	// Xbox controllers
	Joystick controlDriver = new Joystick(0);			// the joystick responsible for driving
	Joystick controlOperator = new Joystick(1);			// the joystick responsible for operator controls
	Joystick controlWorking;  							// the controller currently being read from, usually used just for one-driver control
	
	// NavX Constructor
	static AHRS ahrs = new AHRS(SPI.Port.kMXP);
	
	// All motors
	static Spark motorAngle[] = { // Directional motors
		new Spark(3),
		new Spark(7),
		new Spark(6),
		new Spark(2)
	};
	
	static Spark motorDrive[] = { // Movement motors
		new Spark(1),
		new Spark(5),
		new Spark(4),
		new Spark(0)
	};

	// Pneumatic hatch intake
	static DoubleSolenoid solenoidHatchIntake = new DoubleSolenoid(0,1);
	
	// Blinkin LED Driver

	// option 1
	Spark sparkLeds = new Spark(10);
	Blinkin leds = new Blinkin(sparkLeds);
	
	// option 2 (don't even bother this is stupid)
	// Blinkin leds = new Blinkin(new Servo(0));

	// Lift/Climber
	static TalonSRX motorPivot = new TalonSRX(1);
	static TalonSRX motorFootWheels = new TalonSRX(2);
	static TalonSRX motorLift = new TalonSRX(3);
	static TalonSRX motorClimb = new TalonSRX(4);
	static TalonSRX motorIntake = new TalonSRX(5);
	//=======================================
	
	// PID LOOPS AND ROUTINES

	// These control the steering motors using the mers (?? idk what mers is)
	static PIDController PIDdrive[] = {
		new PIDController(0.035,0,0.01,encoderAngle[0],motorAngle[0]),
		new PIDController(0.035,0,0.01,encoderAngle[1],motorAngle[1]),
		new PIDController(0.035,0,0.01,encoderAngle[2],motorAngle[2]),
		new PIDController(0.035,0,0.01,encoderAngle[3],motorAngle[3])
	};
	
	// Left behind in old program, unsure if they are necessary yet
	
		// These are used for autonomous in the turning to a specific angle
		static PIDController PIDautoAngle[] = {
			new PIDController(0.06,0,0.013,ahrs,motorDrive[0]),
			new PIDController(0.06,0,0.013,ahrs,motorDrive[1]),
			new PIDController(0.06,0,0.013,ahrs,motorDrive[2]),
			new PIDController(0.06,0,0.013,ahrs,motorDrive[3])
		};
		
		// These are used for autonomous in moving to a certain distance
		static PIDController PIDautoDistance[] = {
			new PIDController(0.025,0,0.01,encoderDistance,motorDrive[0]),
			new PIDController(0.025,0,0.01,encoderDistance,motorDrive[1]),
			new PIDController(0.025,0,0.01,encoderDistance,motorDrive[2]),
			new PIDController(0.025,0,0.01,encoderDistance,motorDrive[3])
		};
	
	// Action queues
	ActionQueue actionQueues[] = {
		new ActionQueue(),
		new ActionQueue(),
		new ActionQueue(),
		new ActionQueue(),
		new ActionQueue()
	};

	// Reference IDs for action queues
	final int QUEUE_TEST = 0;
	final int QUEUE_PLACEHATCH = 1;
	final int QUEUE_CLIMB = 2;
	final int QUEUE_GRABHATCH = 3;
	final int QUEUE_HABDESCENT = 4;
	//=======================================

	// End of variable definitions
	// <--- ROBOT INITIALIZATION --->
	
	/**
	 * This function is called when the robot is turned on
	 */
	@Override
	public void robotInit() {
		// Configure swerve wheel PID loops
		PIDdrive[0].setOutputRange(-1, 1);
		PIDdrive[1].setOutputRange(-1, 1);
		PIDdrive[2].setOutputRange(-1, 1);
		PIDdrive[3].setOutputRange(-1, 1);
		
		// Autonomous wheel speed loop settings
		for (int i=0;i<=3;i++) {
			PIDautoAngle[i].setOutputRange(-0.5, 0.5);
			PIDautoAngle[i].setInputRange(-180, 180);
			PIDautoAngle[i].setContinuous();
			PIDautoDistance[i].setOutputRange(-0.2, 0.2);
		}

		// Reset lift/climb encoders
		motorLift.setSelectedSensorPosition(0);
		motorLift.setInverted(true);
		motorPivot.setSelectedSensorPosition(0);
		motorPivot.setInverted(true);
		motorClimb.setSelectedSensorPosition(0);

		// Feed action queues, they hunger for your command
		actionQueues[QUEUE_TEST].queueFeed(ActionQueue.Command.PIVOT,1,50,false,.2,0,0);
		actionQueues[QUEUE_TEST].queueFeed(ActionQueue.Command.SWERVE,50,100,false,.4,0,0);
		actionQueues[QUEUE_TEST].queueFeed(ActionQueue.Command.PIVOT,100,150,false,-.2,0,0);

		// Place hatch (almost functional)
		actionQueues[QUEUE_PLACEHATCH].queueFeed(ActionQueue.Command.SWERVE,1,24,false,.22,.01,0);
		actionQueues[QUEUE_PLACEHATCH].queueFeed(ActionQueue.Command.PIVOT,1,84,false,-135,1,0);		// make sure pivot is parallel with ground
		actionQueues[QUEUE_PLACEHATCH].queueFeed(ActionQueue.Command.SWERVE,25,75,false,.48,.01,0);		// move towards target assuming we're lined up
		actionQueues[QUEUE_PLACEHATCH].queueFeed(ActionQueue.Command.HATCH_INTAKE,95,96,false,0,0,0);	// release the hatch
		actionQueues[QUEUE_PLACEHATCH].queueFeed(ActionQueue.Command.SWERVE,140,200,false,-.38,0,0);	// back up
		actionQueues[QUEUE_PLACEHATCH].queueFeed(ActionQueue.Command.PIVOT,150,200,false,-60,1,0);		// pivot down just a touch
		
		// Grab hatch (not working)
		actionQueues[QUEUE_GRABHATCH].queueFeed(ActionQueue.Command.HATCH_INTAKE,1,2,false,0,0,0);		// make sure intake is open
		actionQueues[QUEUE_GRABHATCH].queueFeed(ActionQueue.Command.PIVOT,1,41,false,-100,1,0);			// make sure pivot is parallel with ground
		actionQueues[QUEUE_GRABHATCH].queueFeed(ActionQueue.Command.SWERVE,30,60,false,.48,0,0);		// push forward a bit
		actionQueues[QUEUE_GRABHATCH].queueFeed(ActionQueue.Command.HATCH_INTAKE,70,71,false,1,0,0);	// assuming we've speared already, grab the hatch
		actionQueues[QUEUE_GRABHATCH].queueFeed(ActionQueue.Command.PIVOT,95,115,false,810,1,0);		// pull hatch out
		actionQueues[QUEUE_GRABHATCH].queueFeed(ActionQueue.Command.LIFT,110,140,false,.12,0,0);		// lift up just a bit
		actionQueues[QUEUE_GRABHATCH].queueFeed(ActionQueue.Command.SWERVE,145,175,false,-.48,0,0);		// back up

		// Descend the hab
		actionQueues[QUEUE_HABDESCENT].queueFeed(ActionQueue.Command.PIVOT,1,150,false,880,1,0);
		actionQueues[QUEUE_HABDESCENT].queueFeed(ActionQueue.Command.SWERVE,75,250,false,.7,0,0);
	}
	
	/**
	 * This function is called immediately when the robot is disabled
	 */
	public void disabledInit() {
		setAllPIDControllers(PIDdrive, false);
		setAllPIDControllers(PIDautoAngle, false);
		setAllPIDControllers(PIDautoDistance, false);
		
		setAllPIDSetpoints(PIDdrive, 0);

		limelightKillSeeking();
		killQueues(actionQueues);
		motorLift.setSelectedSensorPosition(0);
		motorClimb.setSelectedSensorPosition(0);
	}

	/**
	 * This function is called periodically while disabled
	 */
	public void disabledPeriodic() {
		if (matchTime == 0) leds.setLEDs(Blinkin.C1_AND_C2_GRADIENT);
		if (1 < matchTime && matchTime <= 130) leds.setLEDs(Blinkin.STROBE_RED);
		if (matchTime > 130) leds.setLEDs(Blinkin.RAINBOW_RAINBOWPALETTE);
		SmartDashboard.putNumber("LEDnumber",leds.getLEDs());
	}
	
	/**
	 * This function is called immediately when autonomous begins
	 */
	@Override
	public void autonomousInit() {
		wheelSpeedTimer.start();
		wheelSpeedTimer.reset();
		encoderDistance.reset();
		ahrs.reset();

		init();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		//SmartDashboard.putNumber("Gyro", ahrs.getYaw());
		//SmartDashboard.putNumber("CIMCODER",encoderDistance.get());
		matchTime = DriverStation.getInstance().getMatchTime();
		periodic();
	}
	
	/**
	 * This function is called when teleop begins
	 */
	public void teleopInit() {
		init();
	}
	
	/**
	 * This function is called periodically during teleop mode
	 */
	@Override
	public void teleopPeriodic() {
		periodic();
	}

	/**
	 * This should be run whenever the robot is being enabled
	 */
	public void init() {
		if (PIDautoAngle[0].isEnabled()) {
			setAllPIDControllers(PIDautoAngle, false);
		}
		if (PIDautoDistance[0].isEnabled()) {
			setAllPIDControllers(PIDautoDistance, false);
		}
		setAllPIDControllers(PIDdrive, true);
		
		encoderDistance.reset();
		
		wheelSpeedTimer.start();
		wheelSpeedTimer.reset();
		
		ahrs.reset();
		encoderAngle[0].reset();encoderAngle[1].reset();encoderAngle[2].reset();encoderAngle[3].reset();
		resetAllWheels();

		limelightGuideMode = 0;
		driverOriented = true;

		pivotSetpoint = 0;motorPivot.setSelectedSensorPosition(0);
		liftSetpoint = 0;motorLift.setSelectedSensorPosition(0);
		liftSetpointControl = false;pivotSetpointControl = false;
	}

	/**
	 * Drives the robot. Run this whenever you want to drive the robot. Everything is here. This is how to drive the robot.
	 */
	public void periodic() {
		// Begin DRIVER CONTROL
		if (INTERFACE_SINGLEDRIVER == false || (INTERFACE_SINGLEDRIVER == true && singleDriverController == 0)) {
			controlWorking = controlDriver;
			
			// Drive the robot
			if (!emergencyTank) {
				if (!controlWorking.getRawButton(1) && !limelightSeeking) {
					//driverOriented = true;
					jFwd = -controlWorking.getRawAxis(1);if (Math.abs(jFwd) < CONTROL_DEADZONE) jFwd = 0;
					if (!controlWorking.getRawButton(BUTTON_RB)) jFwd /= CONTROL_SPEEDREDUCTION;
					jStr = controlWorking.getRawAxis(0);if (Math.abs(jStr) < CONTROL_DEADZONE) jStr = 0;
					if (!controlWorking.getRawButton(BUTTON_RB)) jStr /= CONTROL_SPEEDREDUCTION;
					jRcw = controlWorking.getRawAxis(4);if (Math.abs(jRcw) < CONTROL_DEADZONE) jRcw = 0;
					if (!controlWorking.getRawButton(BUTTON_RB)) jRcw /= CONTROL_SPEEDREDUCTION;
					if (reverseRotate) {jRcw=-jRcw;}
					swerve(jFwd,jStr,jRcw,driverOriented);
				} else {
					/*
					driverOriented = false;
						Auto tilt correct program from 2018, deprecated for now
					if (ahrs.getRoll() >= 2) {
						swerve(0,0.5,0,false);
					} else if (ahrs.getRoll() <= -2) {
						swerve(0,-0.5,0,false);
					} else if (ahrs.getPitch() >= 2) {
						swerve(0.5,0,0,false);
					} else if (ahrs.getPitch() <= -2) {
						swerve(-0.5,0,0,false);
					}*/
				}
			} else {
				setAllPIDSetpoints(PIDdrive, 0);
				resetAllWheels();
				motorDrive[0].set(controlWorking.getRawAxis(5));motorDrive[3].set(controlWorking.getRawAxis(5));
				motorDrive[2].set(controlWorking.getRawAxis(1));motorDrive[1].set(controlWorking.getRawAxis(1));
			}
			
			// Reset the gyroscope
			if (controlWorking.getRawButton(BUTTON_Y)) ahrs.reset();
			
			// Reset the wheels
			if (controlWorking.getRawButton(BUTTON_X)) {
				resetAllWheels();
				setAllPIDSetpoints(PIDdrive, 0);
			}

			// Foot wheel control
			if (controlWorking.getRawAxis(2) >= CONTROL_FOOTWHEEL_DEADZONE) {
				motorFootWheels.set(ControlMode.PercentOutput,controlWorking.getRawAxis(2));
			} else if (controlWorking.getRawAxis(3) >= CONTROL_FOOTWHEEL_DEADZONE) {
				motorFootWheels.set(ControlMode.PercentOutput,-controlWorking.getRawAxis(3));
			} else motorFootWheels.set(ControlMode.PercentOutput,0);

			// Toggle operator control
			if (controlWorking.getRawButtonPressed(BUTTON_A)) {
				if (driverOriented == true) driverOriented = false; else driverOriented = true;
			}

			// LIMELIGHT SEEKING CODE
			// Toggle Limelight activity
			if (controlWorking.getRawButtonPressed(BUTTON_LB)) {
				if (limelightActive) limelightActive = false; else limelightActive = true;
			}
			// Kill seeking if joystick input is given
			if (limelightSeeking == true && ((controlWorking.getRawAxis(0) >= .3 || controlWorking.getRawAxis(0) <= -.3) || (controlWorking.getRawAxis(1) >= .3 || controlWorking.getRawAxis(1) <= -.3) || (controlWorking.getRawAxis(4) >= .3 || controlWorking.getRawAxis(4) <= -.3))) {
				limelightKillSeeking();
			}
			// Run Limelight searching if active
			if (limelightActive == true) {
				ledMode.setNumber(0);	// turn leds on
				camMode.setNumber(0);	// set cam to low-contrast view
				limelightGather();
				if (controlWorking.getRawButtonPressed(BUTTON_B) && limelightTargetFound == true && limelightSeeking == false) {
					// Set seeking to GOD'S WILL, an angle will be chosen by Limelight
					limelightStart(-1);
				}
				if (controlWorking.getPOV() == 0 && limelightTargetFound == true && limelightSeeking == false) {
					// Set seeking to FAR TARGET, the angle shall be 145 degrees
					limelightStart(150);
				}
				if (controlWorking.getPOV() == 180 && limelightTargetFound == true && limelightSeeking == false) {
					limelightStart(20);
				}
				/*if (controlWorking.getRawButtonPressed(BUTTON_START) && limelightTargetFound == true && limelightSeeking == false) {
					// Set seeking on
					limelightSeeking = true;
					limelightInputTimer = 25;
					limelightGuideMode = 1;
					limelightInitX = limelightX;	// used to find a rate of change
					limelightGoalAngle = ahrs.getYaw();	// keep yaw at yaw for whole motion
					limelightPhase = 1;
					System.out.println("seeking initiated: mode 1 - forward correction guidance");
				}*/
				// Track to a target if the target is still present
				if (limelightSeeking == true && limelightTargetFound == true) {
					switch (limelightGuideMode) {
						// Multiphase Guidance
						// In this mode, limelightPhase defines what action is currently taking place. Each phase will pass when certain conditions are met.
						case 0:
							// Phase 1: angle to the forced angle

							if (limelightPhase == 1) {
								if (limelightInputTimer > 0) {
									limelightInputTimer --;	// elapse time to make sure the wheels are turned by the 
									swerve(0,0,.05,false);
									// Find goal angle
									if (limelightForceAngle == -1) limelightGoalAngle = 90 * Math.round(ahrs.getYaw() / 90);	else limelightGoalAngle = limelightForceAngle; // the goal angle is always going to be at some 45 degree angle so round yaw to the nearest 45
									if (Math.signum(ahrs.getYaw()) != Math.signum(limelightGoalAngle)) limelightGoalAngle *= Math.signum(ahrs.getYaw());	// if the angle I want is negative and I'm positive then change the target to my angle
								} else {
									if (Math.signum(ahrs.getYaw()) != Math.signum(limelightGoalAngle)) limelightGoalAngle *= Math.signum(ahrs.getYaw());	// if the angle I want is negative and I'm positive then change the target to my angle
									limelightPIDAngle = -proportionalLoop(CONTROL_CAM_ANGLEPIDANGLE,ahrs.getYaw(),limelightGoalAngle);	// find the motor speed required to reach my target angle
									if (-CONTROL_CAM_ANGLETHRESHOLD < limelightGoalAngle && limelightGoalAngle < CONTROL_CAM_ANGLETHRESHOLD) limelightPIDAngle = 0;	// if I'm close enough to the target angle then don't bother adjusting it
									swerve(0,0,limelightPIDAngle,false);	// actual move function
								}
							}

							// Phase 2: line up laterally with the target

							if (limelightPhase == 2) {

								if (limelightInputTimer > 0) {
									limelightInputTimer --;	// elapse time to make sure the wheels are turned by the time I'm moving
									swerve(0,.1,0,false);	// turn wheels to lateral movement
									// Find goal angle
									if (limelightForceAngle == -1) limelightGoalAngle = 90 * Math.round(ahrs.getYaw() / 90);	else limelightGoalAngle = limelightForceAngle; // the goal angle is always going to be at some 45 degree angle so round yaw to the nearest 45
									if (Math.signum(ahrs.getYaw()) != Math.signum(limelightGoalAngle)) limelightGoalAngle *= Math.signum(ahrs.getYaw());	// if the angle I want is negative and I'm positive then change the target to my angle
								} else {
									if (Math.signum(ahrs.getYaw()) != Math.signum(limelightGoalAngle)) limelightGoalAngle *= Math.signum(ahrs.getYaw());	// if the angle I want is negative and I'm positive then change the target to my angle
									limelightPIDAngle = -proportionalLoop(CONTROL_CAM_STRAFEPIDANGLE,ahrs.getYaw(),limelightGoalAngle);	// find the motor speed required to reach my target angle
									if (-CONTROL_CAM_ANGLETHRESHOLD < limelightGoalAngle && limelightGoalAngle < CONTROL_CAM_ANGLETHRESHOLD) limelightPIDAngle = 0;	// if I'm close enough to the target angle then don't bother adjusting it
									swerve(0,proportionalLoop(CONTROL_CAM_STRAFEPIDSTRAFE,limelightX,0),limelightPIDAngle,false);	// actual move function
								}
								// Proceed to next step
								if ((Math.abs(limelightX) < CONTROL_CAM_MOE)/* && (-CONTROL_CAM_ANGLETHRESHOLD + limelightGoalAngle < ahrs.getYaw() && ahrs.getYaw() < CONTROL_CAM_ANGLETHRESHOLD + limelightGoalAngle)*/) {	// if I'm laterally within margin of error and my angle is within threshold
									limelightPhase = 3;
									limelightInputTimer = 20;
								}
							}

							// Phase 3: proceed towards target

							if (limelightPhase == 3) {
								
								if (limelightInputTimer > 0) {
									limelightInputTimer --; 
									swerve(.2,.09,0,false);
									if (limelightForceAngle == -1) limelightGoalAngle = 90 * Math.round(ahrs.getYaw() / 90); else limelightGoalAngle = limelightForceAngle; // the goal angle is always going to be at some 45 degree angle so round yaw to the nearest 45
								} else {
									limelightPIDAngle = -proportionalLoop(CONTROL_CAM_FWDPIDANGLE,ahrs.getYaw(),limelightGoalAngle);	// find the motor speed required to reach my target angle
									swerve(-proportionalLoop(CONTROL_CAM_FWDPIDFWD,limelightArea,CONTROL_CAM_FWDAREATHRESHOLD),proportionalLoop(CONTROL_CAM_FWDPIDSTRAFE,limelightX,0),limelightPIDAngle,false);
								}
								// Proceed to next step
								if (limelightArea >= CONTROL_CAM_AREACLEARANCE) {
									limelightKillSeeking();
								}
							}
							break;

						// Forward Correction Guidance
						// In this mode, limelightPhase defines what action is taking place, however the robot continues to move forward during each step of the process.
						// This mode has largely been phased out and it is highly recommended that you never run this guidance mode.
						case 1:
							// Move towards the target without strafe or rotation to collect data
							// Rate of change of the target will be recorded here and used for phase 2
							if (limelightPhase == 1) {
								limelightInputTimer --;
								if (limelightArea < 1.1 || limelightInputTimer > 0) {	// if .5 sec hasn't passed OR area is bigger than 1.1
									// Move forward
									limelightGoalAngle *= Math.signum(ahrs.getYaw());
									swerve(proportionalLoop(.22, limelightArea, 2.2), 0, proportionalLoop(.001, ahrs.getYaw() / 7, limelightGoalAngle / 7),false);
								} else {
									// Passage

									limelightPhase = 2;
									limelightGoalAngle = 45 * Math.round(ahrs.getYaw() / 45);	// the goal angle is always going to be at some 45 degree angle so round yaw to the nearest 45
									if (Math.signum(ahrs.getYaw()) != Math.signum(limelightGoalAngle)) limelightGoalAngle *= Math.signum(ahrs.getYaw());	// if the angle I want is negative and I'm positive then change the target to my angle
									if (limelightX > limelightInitX) {	// if I drifted left
										limelightROC = -Math.abs(limelightX - limelightInitX);
									} else limelightROC = Math.abs(limelightX - limelightInitX);	// if I drifted right
									
									// If ROC barely changed, then we're probably straight on. We can just skip phase 2
									/*if (-.3 < limelightROC && limelightROC < .3)  {
										limelightPhase = 3;
									}*/
									limelightInputTimer = 20;
								}
							}
							// Slightly approach target, correct angle based on gyro and limelightROC
							if (limelightPhase == 2) {
								if (limelightInputTimer > 0) {
									limelightInputTimer --;
									swerve(0,.1,0,false);
								} else {
									limelightGoalAngle *= Math.signum(ahrs.getYaw());
									swerve(.1, -proportionalLoop(.038,limelightX,0),proportionalLoop(.0024,ahrs.getYaw() / 2,limelightGoalAngle / 2),false);
									if ((-CONTROL_CAM_MOE < limelightX && limelightX < CONTROL_CAM_MOE) && (-CONTROL_CAM_ANGLETHRESHOLD + limelightGoalAngle < ahrs.getYaw() && ahrs.getYaw() < CONTROL_CAM_ANGLETHRESHOLD + limelightGoalAngle)) {	// if I'm laterally within margin of error and my angle is within threshold
										limelightPhase = 3;
										limelightInputTimer = 20;
									}
								}
							}
							// Approach
							if (limelightPhase == 3) {
								if (limelightInputTimer > 0) {
									limelightInputTimer --;
								} else {
									swerve(proportionalLoop(.042,limelightArea,15),-proportionalLoop(.014,limelightX,0),0,false);
								}
							}
							break;
					}
					if (CONTROL_CAM_INTERRUPTRECOVERY) {
						// Store values for later if we lose sight of the target
						limelightInterX = limelightX;
						limelightInterY = limelightY;
						limelightInterArea = limelightArea;
						limelightInterTimer = CONTROL_CAM_INTERRUPTRECOVERYTIME;
					}
				} else {
					// Interrupt recovery - if the target has been lost, continue to move in the general direction that the robot last remembers the target being in, until CONTROL_CAM_INTERRUPTRECOVERYTIME time has past
					if (CONTROL_CAM_INTERRUPTRECOVERY == false) limelightKillSeeking(); else if (limelightSeeking == true && limelightTargetFound == false) {
						switch (limelightPhase) {
							case 1:
								// Strafe to target, don't bother with the angular piece
								swerve(0,proportionalLoop(CONTROL_CAM_STRAFEPIDSTRAFE * .70,limelightInterX,0),0,false);	// Strafing speed is reduced to 70% in case we're overshooting the target during this process on accident
								break;
							case 2:
								// Move forward towards target, strafe just a little bit but not much
								swerve(proportionalLoop(CONTROL_CAM_FWDPIDFWD * .70,limelightInterArea,CONTROL_CAM_FWDAREATHRESHOLD),proportionalLoop(CONTROL_CAM_FWDPIDSTRAFE * .60,limelightInterX,0),CONTROL_CAM_FWDANGLECORRECT,false);	// Forward speed is reduced to 70%
								break;
						}
						// Experimental - switch to phase 2 if we deem that we're ready
						// Specifically, this checks if the x val is already within a slightly broader MOE, OR if a certain amount of time has passed and we're within an even broader MOE (assuming we've moved far enough to compensate)
						if (limelightPhase == 1 && ((Math.abs(limelightInterX) > CONTROL_CAM_MOE + 1.5 && Math.abs(limelightInterX) < CONTROL_CAM_MOE + 4 && limelightInterTimer / CONTROL_CAM_INTERRUPTRECOVERYTIME <= .5) || Math.abs(limelightInterX) <= CONTROL_CAM_MOE + 1.5)) {
							limelightPhase = 2;
						}
						// Kill seeking if we're close enough to the target in phase 2 anyway
						if (limelightPhase == 2 && limelightInterArea >= CONTROL_CAM_AREACLEARANCE) {
							limelightKillSeeking();
							System.out.println("seeking ended in recovery mode - target was presumably reached");
						}

						// If enough time has passed then abandon seeking
						limelightInterTimer --;
						if (limelightInterTimer <= 0) {
							limelightKillSeeking();
							System.out.println("seeking ended in recovery mode - target timed out");
						}
					} else limelightKillSeeking();
				}
				SmartDashboard.putNumber("limelightPhase:", limelightPhase);
			} else {
				// Limelight is not active, turn lights off
				ledMode.setNumber(1);	// turn leds off
				camMode.setNumber(1);	// set cam to driver view
			}
		}
		// END OF LIMELIGHT SEEKING CODE
		
		// End DRIVER CONTROL
		// Begin OPERATOR DRIVING

		if (INTERFACE_SINGLEDRIVER == false || (INTERFACE_SINGLEDRIVER == true && singleDriverController == 1)) {
			if (INTERFACE_SINGLEDRIVER == false) controlWorking = controlOperator; else controlWorking = controlDriver;

			// Run the lift
			if (Math.abs(controlWorking.getRawAxis(5)) >= CONTROL_LIFT_DEADZONE) {
				if (controlWorking.getRawButton(4)) motorLift.set(ControlMode.PercentOutput,controlWorking.getRawAxis(5) / CONTROL_LIFT_PRECISION_FACTOR); else motorLift.set(ControlMode.PercentOutput,controlWorking.getRawAxis(5));
				liftSetpointControl = false;
			} else if (liftSetpointControl == false) motorLift.set(ControlMode.PercentOutput,0);
			// Reset lift encoder
			if (controlWorking.getRawButton(BUTTON_RSTICK) && controlWorking.getRawButton(BUTTON_B)) motorLift.setSelectedSensorPosition(0);

			// Auto level control
			if (controlWorking.getPOV() == 270) liftLevel(1);
			if (controlWorking.getPOV() == 0) liftLevel(2);
			if (controlWorking.getPOV() == 90 || controlWorking.getPOV() == 45) liftLevel(3);

			if (liftSetpointControl == true) setLift(liftSetpoint);

			// Run the climber
			if (controlWorking.getRawButton(1)) {
				motorClimb.set(ControlMode.PercentOutput,-.5);
			} else if (controlWorking.getRawButton(3)) {
				motorClimb.set(ControlMode.PercentOutput,.5);
			} else motorClimb.set(ControlMode.PercentOutput,0);

			// Run the hatch intake
			if (controlWorking.getRawButton(BUTTON_LB)) {
				// Open solenoid
				solenoidHatchIntake.set(DoubleSolenoid.Value.kReverse);
			}
			if (controlWorking.getRawButton(BUTTON_RB)) {
				// Close solenoid
				solenoidHatchIntake.set(DoubleSolenoid.Value.kForward);
			}

			// Intake wheel control
			if (controlWorking.getRawAxis(2) >= CONTROL_INTAKE_DEADZONE) {
				motorIntake.set(ControlMode.PercentOutput,-controlWorking.getRawAxis(2));
			} else if (controlWorking.getRawAxis(3) >= CONTROL_INTAKE_DEADZONE) {
				motorIntake.set(ControlMode.PercentOutput,controlWorking.getRawAxis(3));
			} else motorIntake.set(ControlMode.PercentOutput,0);

			// Pivot control - uses a setpoint by default, push in the joystick to change
			if (!pivotSetpointControl) {
				if (Math.abs(controlWorking.getRawAxis(1)) >= CONTROL_PIVOT_DEADZONE) {
					motorPivot.set(ControlMode.PercentOutput,-controlWorking.getRawAxis(1) / 2);
				} else motorPivot.set(ControlMode.PercentOutput,0);
			} else {
				if (Math.abs(controlWorking.getRawAxis(1)) >= CONTROL_PIVOT_DEADZONE) {
					pivotSetpoint += controlWorking.getRawAxis(1) * 6.5;
				}
			}

			// Level pivot to eject a cargo
			if (controlWorking.getPOV() == 180) {
				pivotSetpoint = -410;
				pivotSetpointControl = true;
			}

			// Switch between setpoint control and powered control
			if (controlWorking.getRawButtonPressed(BUTTON_LSTICK) && !controlWorking.getRawButton(BUTTON_B)) {
				if (pivotSetpointControl == true) pivotSetpointControl = false; else {
					pivotSetpointControl = true;
					pivotSetpoint = motorPivot.getSelectedSensorPosition();
				}
			}

			// Reset pivot encoder with left stick and B button
			if (controlWorking.getRawButton(BUTTON_LSTICK) && controlWorking.getRawButton(BUTTON_B)) {
				motorPivot.setSelectedSensorPosition(0);
				pivotSetpoint = 0;
			}

			if (pivotSetpointControl) setPivot(pivotSetpoint);	// this is default, functions above will enable and disable this

			// Auto hatch place
			if (controlWorking.getRawButton(BUTTON_SELECT)) {
				if (!actionQueues[QUEUE_PLACEHATCH].queueRunning()) actionQueues[QUEUE_PLACEHATCH].queueStart(); else actionQueues[QUEUE_PLACEHATCH].queueStop();
			}
		}

		// End OPERATOR DRIVING
		// Begin UNIVERSAL FUNCTIONS

		// Toggle drive mode if single driver interface is active
		if (INTERFACE_SINGLEDRIVER == true && controlDriver.getRawButton(BUTTON_START) == true) {
			if (singleDriverController == 0) singleDriverController = 1; else singleDriverController = 0;
		}
		
		// Record match time
		matchTime = DriverStation.getInstance().getMatchTime();

		// Run action queues
		runQueues(actionQueues);
		
		// Dashboard dump
		SmartDashboard.putNumber("ControllerID",singleDriverController);
		SmartDashboard.putNumber("LiftEncoder",motorLift.getSelectedSensorPosition());
		SmartDashboard.putNumber("PivotSetpoint",pivotSetpoint);
		SmartDashboard.putNumber("ClimbEncoder",motorClimb.getSelectedSensorPosition());
		SmartDashboard.putNumber("PivotEncoder",motorPivot.getSelectedSensorPosition());
		SmartDashboard.putNumber("CIMCODER", encoderDistance.get());
		SmartDashboard.putNumber("Gyro-Yaw", ahrs.getYaw());
		SmartDashboard.putNumber("Encoder1:", encoderAngle[0].get());
		SmartDashboard.putNumber("Encoder2:", encoderAngle[1].get());
		SmartDashboard.putNumber("Encoder3:", encoderAngle[2].get());
		SmartDashboard.putNumber("Encoder4:", encoderAngle[3].get());
		SmartDashboard.putNumber("YawAxis",ahrs.getYaw());
		SmartDashboard.putBoolean("limelightActive",limelightActive);
		SmartDashboard.putBoolean("limelightSeeking",limelightSeeking);
		SmartDashboard.putNumber("limelightGoalAngle",limelightGoalAngle);
		SmartDashboard.putNumber("limelightROC",limelightROC);
		SmartDashboard.putNumber("LEDnumber",leds.getLEDs());
		SmartDashboard.putBoolean("PivotSetpointCtrl",pivotSetpointControl);
		SmartDashboard.putBoolean("DriverOriented",driverOriented);

		// LED functions
		if (limelightSeeking == false) leds.setLEDs(Blinkin.C1_LARSONSCAN); else leds.setLEDs(Blinkin.LIGHTCHASE_RED);	// rudimentary for now

		// End UNIVERSAL FUNCTIONS
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		if (PIDautoAngle[0].isEnabled()) {
			setAllPIDControllers(PIDautoAngle, false);
		}
		if (PIDautoDistance[0].isEnabled()) {
			setAllPIDControllers(PIDautoDistance, false);
		}
		if (PIDdrive[0].isEnabled()) {
			setAllPIDControllers(PIDdrive, false);
		}
		
		SmartDashboard.putNumber("Joystick y axis", controlDriver.getRawAxis(1));
		
		SmartDashboard.putNumber("Encoder1:", encoderAngle[0].get());
		SmartDashboard.putNumber("Encoder2:", encoderAngle[1].get());
		SmartDashboard.putNumber("Encoder3:", encoderAngle[2].get());
		SmartDashboard.putNumber("Encoder4:", encoderAngle[3].get());
		
		SmartDashboard.putNumber("NavX Pitch:", ahrs.getPitch());
		SmartDashboard.putNumber("Navx Roll:", ahrs.getRoll());
		SmartDashboard.putNumber("NavX Yaw:", ahrs.getYaw());
		SmartDashboard.putNumber("NavX Angle:", ahrs.getAngle());
		SmartDashboard.putNumber("NavX Raw X:", ahrs.getRawGyroX());
		
		if (controlDriver.getRawButton(1)) wheelTune = 0;
		if (controlDriver.getRawButton(2)) wheelTune = 1;
		if (controlDriver.getRawButton(3)) wheelTune = 2;
		if (controlDriver.getRawButton(4)) wheelTune = 3;
		
		// Adjust wheel angle
		if (controlDriver.getRawButton(5)) motorAngle[wheelTune].set(0.3);
		else if (controlDriver.getRawButton(6)) motorAngle[wheelTune].set(-0.3); else motorAngle[wheelTune].set(0);
		
		// Spin wheels
		if (controlDriver.getRawAxis(2) > .09) motorDrive[wheelTune].set(controlDriver.getRawAxis(2) / 2);
		else if (controlDriver.getRawAxis(3) > .09) motorDrive[wheelTune].set(-controlDriver.getRawAxis(3) / 2);
		else motorDrive[wheelTune].set(0);
	}

	/**
	 * This adjusts the angle of the wheels and sets their speed based on joystick/autonomous input.
	 * 
	 * @param FWD The desired forward speed of the robot
	 * @param STR The desired strafing speed of the robot
	 * @param RCW The desired rotation speed of the robot
	 */
	public static void swerve(double FWD,double STR,double RCW,boolean driverOriented) {
		if (driverOriented) {
			rads = ahrs.getYaw() * Math.PI/180;
			temp = FWD*Math.cos(rads) + STR*Math.sin(rads);
			STR = -FWD*Math.sin(rads) + STR*Math.cos(rads);
			FWD = temp;
		}

		a = STR - RCW * (ROBOT_LENGTH / ROBOT_R);
		b = STR + RCW * (ROBOT_LENGTH / ROBOT_R);
		c = FWD - RCW * (ROBOT_WIDTH / ROBOT_R);
		d = FWD + RCW * (ROBOT_WIDTH / ROBOT_R);

		//1..4: front_right, front_left, rear_left, rear_right

		wheelSpeed1 = Math.sqrt(Math.pow(b,2)+Math.pow(c,2));
		wheelSpeed2 = Math.sqrt(Math.pow(b,2)+Math.pow(d,2));
		wheelSpeed3 = Math.sqrt(Math.pow(a,2)+Math.pow(d,2));
		wheelSpeed4 = Math.sqrt(Math.pow(a,2)+Math.pow(c,2));

		encoderSetpointA = wheel[0].calculateWheelAngle(b,c);
		PIDdrive[0].setSetpoint(encoderSetpointA);SmartDashboard.putNumber("Enc. A setpoint", encoderSetpointA);
		
		encoderSetpointB = wheel[1].calculateWheelAngle(b,d);
		PIDdrive[1].setSetpoint(encoderSetpointB);SmartDashboard.putNumber("Enc. B setpoint", encoderSetpointB);
		
		encoderSetpointC = wheel[2].calculateWheelAngle(a,d);
		PIDdrive[2].setSetpoint(encoderSetpointC);SmartDashboard.putNumber("Enc. C setpoint", encoderSetpointC);
		
		encoderSetpointD = wheel[3].calculateWheelAngle(a,c);
		PIDdrive[3].setSetpoint(encoderSetpointD);SmartDashboard.putNumber("Enc. D setpoint", encoderSetpointD);

		max=wheelSpeed1; if(wheelSpeed2>max)max=wheelSpeed2; if(wheelSpeed3>max)max=wheelSpeed3; if(wheelSpeed4>max)max=wheelSpeed4;
		if (max > 1) {wheelSpeed1/=max; wheelSpeed2/=max; wheelSpeed3/=max; wheelSpeed4/=max;}
		
		wheelSpeed1 *= wheel[0].getFlip();
		wheelSpeed2 *= wheel[1].getFlip();
		wheelSpeed3 *= wheel[2].getFlip();
		wheelSpeed4 *= wheel[3].getFlip();
		
		//Move[2].set(testStick.getRawAxis(1));
		
		if (wheelSpeedTimer.get()>0.1) {
			if (wheelSpeed1 - wheelSpeedActual1 > 0.1) {wheelSpeedActual1 += 0.1;} else if (wheelSpeed1 - wheelSpeedActual1 < -0.1) {wheelSpeedActual1 -= 0.1;} else {wheelSpeedActual1 = wheelSpeed1;}
			if (wheelSpeed2 - wheelSpeedActual2 > 0.1) {wheelSpeedActual2 += 0.1;} else if (wheelSpeed2 - wheelSpeedActual2 < -0.1) {wheelSpeedActual2 -= 0.1;} else {wheelSpeedActual2 = wheelSpeed2;}
			if (wheelSpeed3 - wheelSpeedActual3 > 0.1) {wheelSpeedActual3 += 0.1;} else if (wheelSpeed3 - wheelSpeedActual3 < -0.1) {wheelSpeedActual3 -= 0.1;} else {wheelSpeedActual3 = wheelSpeed3;}
			if (wheelSpeed4 - wheelSpeedActual4 > 0.1) {wheelSpeedActual4 += 0.1;} else if (wheelSpeed4 - wheelSpeedActual4 < -0.1) {wheelSpeedActual4 -= 0.1;} else {wheelSpeedActual4 = wheelSpeed4;}
			wheelSpeedTimer.reset();
		}
		//Move[0].set(wsActual1);Move[1].set(wsActual2);Move[2].set(wsActual3);Move[3].set(wsActual4);
		
		motorDrive[0].set(wheelSpeed1);motorDrive[1].set(wheelSpeed2);motorDrive[2].set(wheelSpeed3);motorDrive[3].set(wheelSpeed4);
	}

	/**
	 * Resets all of the SwerveWheel objects, putting them on a clean slate
	 * (eliminates flipped orientations, stacked setpoints, etc.)
	 */
	public void resetAllWheels() {
		for (int i=0;i<=3;i++) {
			wheel[i].reset();
		}
	}

	/**
	 * Sets all drive wheels to a single value. This is good for turning all the motors off.
	 * @param val is the value to set all the wheels to
	 */
	public void setAllWheels(double val) {
		for (int i=0;i<=3;i++) {
			motorDrive[i].set(val * wheel[i].getFlip());
		}
	}
	
	/**
	 * Enables or disables a given array of four PIDController objects.
	 * @param pids The array of PID Controllers to set
	 * @param enabled True to enable, false to disable
	 */
	public void setAllPIDControllers(PIDController[] pids, boolean enabled) {
		for (int i=0;i<=3;i++) {
			pids[i].setEnabled(enabled);
		}
	}
	
	/**
	 * Sets the setpoints for an array of four PIDController objects.
	 * @param pids The array of PID Controllers to set
	 * @param setpoint The setpoint
	 */
	public void setAllPIDSetpoints(PIDController[] pids, double setpoint) {
		for (int i=0;i<=3;i++) {
			pids[i].setSetpoint(setpoint);
		}
	}
	
	/**
	 * Begin to track a target with Limelight. forceAngle is the angle to calibrate. A value of -1 begins "God's Will" mode, rounding the current yaw to the nearest 90 degrees.
	 * @param forceAngle the angle to force, -1 rounds angle to 90 degrees
	 */
	public static void limelightStart(double forceAngle) {
		limelightSeeking = true;
		limelightPhase = 1;
		limelightInputTimer = 20;
		limelightGuideMode = 0;
		limelightInterTimer = CONTROL_CAM_INTERRUPTRECOVERYTIME;
		limelightInterX = 0;limelightInterY = 0;limelightInterArea = 0;
		limelightForceAngle = forceAngle;
		String llModeString = "SET ANGLE " + Double.toString(forceAngle);
		if (forceAngle == -1) llModeString = "GOD'S WILL";
		System.out.println("seeking initiated: mode 0 - multiphase guidance: " + llModeString);
	}

	/**
	 * Reads and posts network table values acquired from Limelight to the Smart Dashboard. Also creates the limelightTargetFound variable.
	 */
	public void limelightGather() {
		// Read Limelight data table values
		if (limelightInvalidValues == false) {
			limelightX = tx.getDouble(0.0);
			limelightY = ty.getDouble(0.0);
			limelightArea = ta.getDouble(0.0);
			limelightWidth = thor.getDouble(0.0);
			limelightHeight = tvert.getDouble(0.0);
			limelightProp = limelightWidth / limelightHeight;
		}
		double ltv = tv.getDouble(0.0);
		limelightTargetFound = false;
		if (ltv != 1.0) limelightTargetFound = false; else limelightTargetFound = true;	// tv.getDouble wasn't working for some odd reason but this is a workaround

		// Validate inputs, this should prevent jarring hits/flipping between 2 targets from impacting tracking
		if (limelightTargetFound && limelightActive && CONTROL_CAM_VALIDATION == true) {
			limelightValidTimer --;
			if (limelightValidTimer <= 0) {
				// Designated time has passed, begin validating
				if (limelightValidX == 0 && limelightValidY == 0 && limelightValidArea == 0) {
					// Valid values haven't been assigned yet
					limelightValidX = limelightX;
					limelightValidY = limelightY;
					limelightValidArea = limelightArea;
					limelightValidProp = limelightProp;
				} else {
					// Check if values are valid by scoring each change
					double lPropTemp = thor.getDouble(0.0) / tvert.getDouble(0.0);
					double lxROC = Math.abs(limelightValidX - tx.getDouble(0.0)) * CONTROL_CAM_VALIDXSCORE; 	// rate of change of the x value
					double lyROC = Math.abs(limelightValidY - ty.getDouble(0.0)) * CONTROL_CAM_VALIDYSCORE;		// rate of change of the y value
					double laROC = Math.abs(limelightValidArea - ta.getDouble(0.0)) * CONTROL_CAM_VALIDASCORE;// rate of change on the area value
					double lpROC = Math.abs(limelightValidProp - lPropTemp) * CONTROL_CAM_VALIDPSCORE;	// rate of change on the prop value
					if ((1.5 < lPropTemp && lPropTemp < 3) == false) lpROC += 35; // target's proportions are too weird to be a target
					if (ta.getDouble(0.0) > 9) laROC += 15;	// target is too big to be a target
					double lScoreSum = lxROC + lyROC + laROC + lpROC;
					SmartDashboard.putNumber("limelightValidScore",lScoreSum);
					if ((lScoreSum > CONTROL_CAM_VALIDSCORETHRESHOLD) && limelightValidTimerTotal >= 0) {
						// Validation failed, cumulative score was greater than the threshold
						System.out.println("limelight values were just rejected");
						limelightInvalidValues = true;
						limelightValidTimerTotal --;
						limelightX = limelightValidX;
						limelightY = limelightValidY;
						limelightArea = limelightValidArea;
						limelightProp = limelightValidProp;
					} else {
						limelightInvalidValues = false;	// Validation succeeded ABABABABABABABAB
						limelightValidTimerTotal = CONTROL_CAM_VALIDTIMEOUT;
					}
				}
				limelightValidTimer = CONTROL_CAM_VALIDATIONTIME;
			}
		} else if (limelightTargetFound == false && CONTROL_CAM_VALIDATION == true)  {
			// Zero the valid values
			limelightValidX = limelightX;
			limelightValidY = limelightY;
			limelightValidArea = limelightArea;
		}

		// Find estimated distance
		//double mathArea = limelightWidth * limelightHeight;
		//limelightEstDistance = 0.0256 * Math.pow(mathArea,4) - 0.4883 * Math.pow(mathArea,3) + 3.4146 * Math.pow(mathArea,2) - 10.631 * mathArea + 13.509;
		
		limelightEstAngle =  -18.1603 * Math.pow(limelightProp,2) + 19.7507 * limelightProp + 62;	// very guess-work-y and not reliable

		// Write Limelight data table values to the dashboard
		SmartDashboard.putNumber("LimelightX", limelightX);
		SmartDashboard.putNumber("LimelightY", limelightY);
		SmartDashboard.putNumber("LimelightArea", limelightArea);
		SmartDashboard.putBoolean("LimelightTargetFound", limelightTargetFound);
		SmartDashboard.putNumber("LimelightWidth", limelightWidth);
		SmartDashboard.putNumber("LimelightHeight", limelightHeight);
		SmartDashboard.putNumber("LimelightEstAngle",limelightEstAngle);
		SmartDashboard.putNumber("LimelightProp",limelightProp);
	}

	/**
	 * Returns a value based on sensor inputs.
	 * @param p - the proportional constant
	 * @param currentSensor - whatever your current sensor value is
	 * @param desiredSensor - whatever you want the sensor to become after change
	 */
	public static double proportionalLoop(double p, double currentSensor, double desiredSensor) {
		return p * (currentSensor - desiredSensor);
	}

	/**
	 * Immediately turns off limelight seeking
	 */
	public void limelightKillSeeking() {
		// Shut off seeking
		limelightSeeking = false;
		limelightPhase = 0;
		limelightInputTimer = -1;
		limelightInterTimer = CONTROL_CAM_INTERRUPTRECOVERYTIME;
		limelightInterX = 0;limelightInterY = 0;limelightInterArea = 0;
	}

	/**
	 * Run the lift up to a specified level using PID inputs. Returns false if an invalid value is given.
	 * @param level the desired level for the lift between 1 and 3
	 */
	public static boolean liftLevel(int level) {
		if (1 > level || level > 3) return false;
		switch (level) {
			case 1: liftSetpoint = CONTROL_LIFTLEVEL1; break;
			case 2: liftSetpoint = CONTROL_LIFTLEVEL2; break;
			case 3: liftSetpoint = CONTROL_LIFTLEVEL3; break;
			default: liftSetpoint = 0; break;
		}
		
		liftSetpointControl = true;
		return true;
	}

	/**
	 * Sets the pivot to a given setpoint using a calibrated proportional loop control. This should probably ALWAYS be used when moving the pivot arm.
	 * @param setpoint the raw desired setpoint, no subtractions or changes
	 */
	public static void setPivot(double setpoint) {
		double p;
		if (motorPivot.getSelectedSensorPosition() > 0 && motorPivot.getSelectedSensorPosition() < 200) p = .015; else p = .025;
		motorPivot.set(ControlMode.PercentOutput,proportionalLoop(p, motorPivot.getSelectedSensorPosition() / 5, setpoint / 5));
	}
	
	/**
	 * Sets the lift to a given setpoint using a calibrated proportional loop control.
	 * @param setpoint the desired setpoint
	 */
	public static void setLift(double setpoint) {
		motorLift.set(ControlMode.PercentOutput, -proportionalLoop(.15,setpoint / 75,motorLift.getSelectedSensorPosition() / 75));
	}

	/**
	 * Tell all of the action queues to run if they are enabled.
	 * @param queues[] the array of queues to iterate through
	 */
	public void runQueues(ActionQueue queues[]) {
		for (int i = 0; i < queues.length; i++) {
			if (queues[i].queueIsRunning == true) queues[i].queueRun();
		}
	}

	public void killQueues(ActionQueue queues[]) {
		for (int i = 0; i < queues.length; i++) {
			queues[i].queueStop();
		}
	}

	/**
	 * The queue action for preparing a turn. This is functionally similar to the queueSwerve command
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the fwd input
	 * @param param2 the second parameter, the str input
	 */
	public static void queuePrepare_Turn(int timeEnd, double param1, double param2) {
		swerve(param1,param2,0,false);
	}

	/**
	 * The queue action for swerving in its raw form. This is completed relative to the ROBOT.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, FWD
	 * @param param2 the second parameter, STR
	 * @param param3 the third parameter, RCW
	 */
	public static void queueSwerve(int timeEnd, double param1, double param2, double param3) {
		swerve(param1,param2,param3,false);
	}

	/**
	 * The queue action for operating the lift.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the value to set the lift to
	 * @param param2 the second parameter, whether to be a percent, 0, or a setpoint, 1
	 */
	public static void queueLift(int timeEnd, double param1, double param2) {
		if (param2 == 1) {
			liftSetpoint = param1;
			liftSetpointControl = true;
			//setLift(param1);	don't think this is needed???
		} else {
			motorLift.set(ControlMode.PercentOutput,param1);
			liftSetpointControl = false;
		}
	}

	/**
	 * The queue action for operating the hatch intake.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, either opening the intake or closing it (0 or 1)
	 */
	public static void queueHatchIntake(int timeEnd, double param1) {
		if (param1 == 0) solenoidHatchIntake.set(DoubleSolenoid.Value.kForward); else if (param1 == 1) solenoidHatchIntake.set(DoubleSolenoid.Value.kReverse);
	}

	/**
	 * The queue action for operating the cargo intake wheels.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the value to set the wheels to
	 */
	public static void queueCargoIntake(int timeEnd, double param1) {
		motorIntake.set(ControlMode.PercentOutput,param1);
	}

	/**
	 * The queue action for operating the foot wheels.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the value to set the wheels to
	 */
	public static void queueFootWheels(int timeEnd, double param1) {
		motorFootWheels.set(ControlMode.PercentOutput,param1);
	}

	/**
	 * The queue action for operating the foot extension.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the value to set the foot to
	 * @param param2 the second parameter, whether to be a percent, 0, or a setpoint, 1
	 */
	public static void queueFootExtend(int timeEnd, double param1, double param2) {
		ControlMode myControlMode = ControlMode.PercentOutput;
		if (param2 == 1) myControlMode = ControlMode.Position;
		motorClimb.set(myControlMode,param1);
	}

	/**
	 * The queue action for operating the pivot arm.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the value to set the pivot to
	 * @param param2 the second parameter, whether to use a percent, 0, or an encoder value, 1
	 */
	public static void queuePivot(int timeEnd, double param1, double param2) {
		if (param2 == 1) setPivot(param1); else motorPivot.set(ControlMode.PercentOutput,param1);
	}
	
	/**
	 * The queue action for raising the lift to a certain level
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the level to set the lift to
	 */
	public static void queueLiftLevel(int timeEnd, double param1) {
		int myLevel = (int) param1;	// converts param1 into an integer
		liftLevel(myLevel);
	}

	/**
	 * The queue action for starting a limelight tracking session, but you sure as hell better give this function enough time to run for it to completely finish
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the desired angle to reach
	 */
	public static void queueLimelightTrack(int timeEnd, double param1) {
		limelightStart(param1);
	}
}