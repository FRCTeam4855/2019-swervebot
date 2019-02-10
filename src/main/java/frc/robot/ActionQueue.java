package frc.robot;

public class ActionQueue {
	enum Command {
		DEAD, PREPARE_TURN, SWERVE, LIFT, PIVOT, HATCH_INTAKE, CARGO_INTAKE, FOOT_WHEELS, FOOT_EXTEND;
	}
	
	Command queueListActions [] = {						// action ID to perform
		Command.DEAD, Command.DEAD, Command.DEAD
	};
	int queueListTimeStart [] = {-1,-1,-1,-1,-1};		// elapsed begin time to run a command
	int queueListTimeEnd [] = {-1,-1,-1,-1,-1};			// elapsed begin time to end a command, a value of -2 means a desired sensor output must be present to stop
	double queueListParam1 [] = {0.0,0.0,0.0,0.0,0.0};	// parameter 1 for queue item
	double queueListParam2 [] = {0.0,0.0,0.0,0.0,0.0};	// parameter 2 for queue item
	double queueListParam3 [] = {0.0,0.0,0.0,0.0,0.0};	// parameter 3 for queue item
	
	int queueElapsedTime = 0;							// current elapsed time for this command in code steps (50 steps in 1 second)
	boolean queueIsRunning = false;						// if queue is enabled or not
	
	int queueMaxTime = -1;								// largest end time in queue
	
	/**
	 * Feeds the queue a new command. Commands can be assigned in any order.
	 * 
	 * @param action the action ID to feed
	 * @param timeStart the elapsed time within the queue in which to start the command
	 * @param timeEnd the elapsed time within the queue in which to end the command
	 * @param param1 the 1st parameter
	 * @param param2 the 2nd parameter
	 * @param param3 the 3rd parameter
	 */
	public void queueFeed(Command action, int timeStart, int timeEnd, double param1, double param2, double param3) {
		queueListActions[queueListActions.length] = action;
		queueListTimeStart[queueListTimeStart.length] = timeStart;
		queueListTimeEnd[queueListTimeEnd.length] = timeEnd;
		queueListParam1[queueListParam1.length] = param1;
		queueListParam2[queueListParam2.length] = param2;
		queueListParam3[queueListParam3.length] = param3;
	}
	
	/**
	 * In the unlikely event that a queue entry must be deleted, this function will delete an entry at a certain timeframe.
	 *
	 * @param action the action ID to kill
	 * @param timeStart the starting time of the command to kill
	 *
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
                        Robot.queuePrepare_Turn(queueListTimeEnd[i],queueListParam1[i],queueListParam2[i],queueListParam3[i]);
                        break;
                    case SWERVE:
                        Robot.queuePrepare_Turn(queueListTimeEnd[i],queueListParam1[i],queueListParam2[i],queueListParam3[i]);
                    default:
                        break;
                }
			}
		}
		if (queueMaxTime > queueMaxTime) queueStop();	// if the last command has finished, the queue can stop
	}
}