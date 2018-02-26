package model;

import control.EstimatorInterface;
import model.robot.Robot;

public class RobotManager implements EstimatorInterface{
	public static final int X_LENGTH = 8;
	public static final int Y_LENGTH = 8;
	public Robot robot;
	
	public RobotManager() {
		robot = new Robot(X_LENGTH, Y_LENGTH);
	}
	
	@Override
	public int getNumRows() {
		return X_LENGTH;
	}

	@Override
	public int getNumCols() {
		return Y_LENGTH;
	}

	@Override
	public int getNumHead() {
		return 4;
	}

	@Override
	public void update() {
		robot.move();
		
	}

	@Override
	public int[] getCurrentTruePosition() {
		int[] pos = {robot.x, robot.y};
		return pos;
	}

	@Override
	public int[] getCurrentReading() {
		int[] pos = {robot.sensor.x, robot.sensor.y};
		return pos;
	}

	@Override
	public double getCurrentProb(int x, int y) {
		// TODO Auto-generated method stub
		/*
		 * This is a meaty section
		 */
		return 0;
	}

	@Override
	public double getOrXY(int rX, int rY, int x, int y, int h) {
		// TODO Auto-generated method stub
		/*
		 * This is a meaty section
		 */
		return 0;
	}

	@Override
	public double getTProb(int x, int y, int h, int nX, int nY, int nH) {
		return Robot.transitionMatrix[Robot.tMatrixFormula(x, y, h, Y_LENGTH)][Robot.tMatrixFormula(nX, nY, nH, Y_LENGTH)];
	}

}
