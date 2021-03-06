// FRC TEAM 4855 ROBOT CODE
// 2019 GAME DESTINATION: DEEP SPACE

// Swerve-bot code: Action Queue class

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class ActionQueue {
	enum Command {
		DEAD, PREPARE_TURN, SWERVE, LIFT, PIVOT, HATCH_INTAKE, CARGO_INTAKE, FOOT_WHEELS, FOOT_EXTEND, LIFT_LEVEL, LIMELIGHT_TRACK;
	}
	
	Command queueListActions [] = new Command [20];		// action ID to perform
	int queueListTimeStart [] = new int [20];			// elapsed begin time to run a command
	int queueListTimeEnd [] = new int [20];				// elapsed begin time to end a command, a value of -2 means a desired sensor output must be present to stop
	boolean queueListKillMotor[] = new boolean [20];	// whether to kill designated motors after the command is stopped or not
	double queueListParam1 [] = new double [20];		// parameter 1 for queue item
	double queueListParam2 [] = new double [20];		// parameter 2 for queue item
	double queueListParam3 [] = new double [20];		// parameter 3 for queue item
	
	int queueElapsedTime = 0;				// current elapsed time for this command in code steps (50 steps in 1 second)
	boolean queueIsRunning = false;			// if queue is enabled or not
	
	int queueMaxTime = -1;					// largest end time in queue
	int queueLength = 0;					// queue length
	/**
	 * Feeds the queue a new command. Commands can be assigned in any order.
	 * @param action the action ID to feed
	 * @param timeStart the elapsed time within the queue in which to start the command
	 * @param timeEnd the elapsed time within the queue in which to end the command
	 * @param param1 the 1st parameter
	 * @param param2 the 2nd parameter
	 * @param param3 the 3rd parameter
	 */
	public void queueFeed(Command action, int timeStart, int timeEnd, boolean killMotor, double param1, double param2, double param3) {
		queueListActions[queueLength] = action;
		queueListTimeStart[queueLength] = timeStart;
		queueListTimeEnd[queueLength] = timeEnd;
		queueListKillMotor[queueLength] = killMotor;
		queueListParam1[queueLength] = param1;
		queueListParam2[queueLength] = param2;
		queueListParam3[queueLength] = param3;
		queueLength ++;
	}
	
	/**
	 * In the unlikely event that a queue entry must be deleted, this function will delete an entry at a certain timeframe.
	 * @param action the action ID to kill
	 * @param timeStart the starting time of the command to kill
	 */
	public void queueDelete(Command action, int timeStart) {
		for (int i = 0; i < queueLength; i ++) {
			if (queueListActions[i] == action && queueListTimeStart[i] == timeStart) {
				// The array will still contain a corpse even though the command is deleted
				queueListActions[i] = Command.DEAD;
				queueListTimeStart[i] = -1;
				queueListTimeEnd[i] = -1;
				queueLength --;
				break;
			}
		}
	}
	
	/**
	 * Starts the queue. If this command is run and the queue is already running, the queue will start over.
	 */
	public void queueStart() {
		queueIsRunning = true;
		queueElapsedTime = 0;
	}
	
	/**
	 * Stops the queue.
	 */
	public void queueStop() {
		queueIsRunning = false;
		queueElapsedTime = 0;
	}
	
	/**
	 * Returns if the queue is running or not.
	 * @return true or false based on if the queue is running
	 */
	public boolean queueRunning() {
		return queueIsRunning;
	}

	/**
	 * This function runs through the fed commands, increases elapsed time, and runs robot commands.
	 */
	public void queueRun() {
		queueElapsedTime ++;
		for (int i = 0; i < queueLength; i ++) {
			if (queueListTimeEnd[i] > queueMaxTime) queueMaxTime = queueListTimeEnd[i];
            if (queueListTimeStart[i] <= queueElapsedTime && queueElapsedTime <= queueListTimeEnd[i]) {
                // Run a certain action. Parameters will be shipped to the robot class along with the command.
                switch (queueListActions[i]) {
                    case PREPARE_TURN:
                        Robot.queuePrepare_Turn(queueListTimeEnd[i],queueListParam1[i],queueListParam2[i]);
                        break;
                    case SWERVE:
                        Robot.queueSwerve(queueListTimeEnd[i],queueListParam1[i],queueListParam2[i],queueListParam3[i]);
						break;
                    case HATCH_INTAKE:
						Robot.queueHatchIntake(queueListTimeEnd[i],queueListParam1[i]);
						break;
					case LIFT:
						Robot.queueLift(queueListTimeEnd[i],queueListParam1[i],queueListParam2[i]);
						break;
					case CARGO_INTAKE:
						Robot.queueCargoIntake(queueListTimeEnd[i],queueListParam1[i]);
						break;
					case FOOT_WHEELS:
						Robot.queueFootWheels(queueListTimeEnd[i],queueListParam1[i]);
						break;
					case FOOT_EXTEND:
						Robot.queueFootExtend(queueListTimeEnd[i],queueListParam1[i],queueListParam2[i]);
						break;
					case PIVOT:
						Robot.queuePivot(queueListTimeEnd[i],queueListParam1[i],queueListParam2[i]);
						break;
					case LIFT_LEVEL:
						Robot.queueLiftLevel(queueListTimeEnd[i],queueListParam1[i]);
					case LIMELIGHT_TRACK:
						Robot.queueLimelightTrack(queueListTimeEnd[i],queueListParam1[i]);
					default:
                        break;
                }
			}
			if (queueElapsedTime == queueListTimeEnd[i] + 1 && queueListKillMotor[i]) {
				// Kill the corresponding motor if applicable
				switch (queueListActions[i]) {
					case CARGO_INTAKE:
						Robot.motorIntake.set(ControlMode.PercentOutput,0);
						break;
					case FOOT_WHEELS:
						Robot.motorFootWheels.set(ControlMode.PercentOutput,0);
						break;
					case PIVOT:
						// Be careful with this one as it will override any setpoint input given
						Robot.motorPivot.set(ControlMode.PercentOutput,0);
					default:
						break;
				}
			}
		}
		if (queueMaxTime < queueElapsedTime) queueStop();	// if the last command has finished, the queue can stop
	}
}