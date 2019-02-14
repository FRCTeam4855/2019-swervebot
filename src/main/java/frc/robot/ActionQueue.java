// FRC TEAM 4855 ROBOT CODE
// 2019 GAME DESTINATION: DEEP SPACE

// Swerve-bot code: Action Queue class

package frc.robot;

public class ActionQueue {
	enum Command {
		DEAD, PREPARE_TURN, SWERVE, LIFT, PIVOT, HATCH_INTAKE, CARGO_INTAKE, FOOT_WHEELS, FOOT_EXTEND, LIFT_STAGE;
	}
	
	Command queueListActions [] = {			// action ID to perform
		Command.DEAD
	};
	int queueListTimeStart [] = {-1};		// elapsed begin time to run a command
	int queueListTimeEnd [] = {-1};			// elapsed begin time to end a command, a value of -2 means a desired sensor output must be present to stop
	boolean queueListKillMotor[] = {false};	// whether to kill designated motors after the command is stopped or not
	double queueListParam1 [] = {0.0};		// parameter 1 for queue item
	double queueListParam2 [] = {0.0};		// parameter 2 for queue item
	double queueListParam3 [] = {0.0};		// parameter 3 for queue item
	
	int queueElapsedTime = 0;				// current elapsed time for this command in code steps (50 steps in 1 second)
	boolean queueIsRunning = false;			// if queue is enabled or not
	
	int queueMaxTime = -1;					// largest end time in queue
	
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
		queueListActions[queueListActions.length - 1] = action;
		queueListTimeStart[queueListTimeStart.length - 1] = timeStart;
		queueListTimeEnd[queueListTimeEnd.length - 1] = timeEnd;
		queueListKillMotor[queueListKillMotor.length - 1] = killMotor;
		queueListParam1[queueListParam1.length - 1] = param1;
		queueListParam2[queueListParam2.length - 1] = param2;
		queueListParam3[queueListParam3.length - 1] = param3;
	}
	
	/**
	 * In the unlikely event that a queue entry must be deleted, this function will delete an entry at a certain timeframe.
	 * @param action the action ID to kill
	 * @param timeStart the starting time of the command to kill
	 */
	public void queueDelete(Command action, int timeStart) {
		for (int i = 0; i < queueListActions.length; i ++) {
			if (queueListActions[i] == action && queueListTimeStart[i] == timeStart) {
				// The array will still contain a corpse even though the command is deleted
				queueListActions[i] = Command.DEAD;
				queueListTimeStart[i] = -1;
				queueListTimeEnd[i] = -1;
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
	 * This function runs through the fed commands, increases elapsed time, and runs robot commands.
	 */
	public void queueRun() {
		queueElapsedTime ++;
		for (int i = 0; i < queueListActions.length; i ++) {
			if (queueListTimeStart[i] > queueMaxTime) queueMaxTime = queueListTimeEnd[i];
            if (queueListTimeStart[i] <= queueElapsedTime && queueElapsedTime <= queueListTimeEnd[i] && queueListActions[i] != Command.DEAD) {
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
					default:
                        break;
                }
			}
		}
		if (queueMaxTime > queueMaxTime) queueStop();	// if the last command has finished, the queue can stop
	}
}