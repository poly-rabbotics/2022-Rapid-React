package frc.robot.subsystems;

import java.io.*;  
import frc.robot.*;

/**
 * Utilizes the proc file system found on POSIX compliant
 * systems (like linux found on the RoboRio) to calculate
 * CPU usage.
 */
public class ProcStat implements Runnable {
	private static final String POSIX_PROC_STAT = "/proc/stat";	
	private int previousSum = 0;
	private int previousIdle = 0;
	private double cpuUsage = 0.0;

	/**
	 * Gets the current CPU usage.
	 */
	public double getCPUUsage() {
		return cpuUsage;
	}
	
	@Override
	public void run() {
		File statFile = new File(POSIX_PROC_STAT);
		FileInputStream cpuStatFile; 

		try {
			// Open cpu stat file found on POSIX compliant systems.
			cpuStatFile = new FileInputStream(statFile);
		} catch (FileNotFoundException e) {
			RobotMap.dashboardLog.logError(POSIX_PROC_STAT + " was not found.");
			RobotMap.dashboardLog.logError(e);
			return;
		}
		
		String cpuStats = cpuStatFile.toString();
		String currentTime = "";
		int[] cpuTimes = new int[10];
		int column = 0;

		for (int c = 4; c < cpuStats.length(); c++) {
			if (cpuStats.charAt(c) == '\n')
				break;
			
			if ("1234567890".indexOf(cpuStats.charAt(c)) == -1) {
				cpuTimes[column] = Integer.parseInt(currentTime);
				column++;
				currentTime = "";	
			}

			currentTime += cpuStats.charAt(c);
		}	

		try {
			cpuStatFile.close();
		} catch (IOException e) {
			RobotMap.dashboardLog.logError("Could not close " + POSIX_PROC_STAT);
			RobotMap.dashboardLog.logError(e);
			return;
		}

		int cpuSum = 0;

		for (int i = 0; i < cpuTimes.length; i++) {
			cpuSum += cpuTimes[i];
		}

		int cpuDelta = cpuSum - previousSum;
		int cpuIdle = cpuTimes[3];
		int cpuDeltaIdle = cpuIdle - previousIdle;
		int cpuUsed = cpuDelta - cpuDeltaIdle;

		previousIdle = cpuIdle;
		previousSum = cpuSum;

		cpuUsage = 100.0 * (double)cpuUsed / (double)cpuDelta;
	}
}