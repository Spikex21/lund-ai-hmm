package model;

import control.EstimatorInterface;
import model.robot.Robot;
import model.robot.Sensor;

public class RobotManager implements EstimatorInterface{
	public static final int X_LENGTH = 6;
	public static final int Y_LENGTH = 6;
	public Robot robot;
	public double[] f = new double[X_LENGTH * Y_LENGTH * 4];
	
	public RobotManager() {
		robot = new Robot(X_LENGTH, Y_LENGTH);
		
		for (int i = 0; i < f.length; i++) {
			f[i] =  1.0 / f.length;
		}
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
		/*
		 *         Equation 15.12 in the book
		 * f_1:t+1 = α * O_t+1 * T transposed * f_1:t 
		 * (returns column vector - above x, y determine state meaning index)
		 * 
		 * alpha - normalization
		 * O_t+1 - diagonal observation matrix based off sensor reading
		 * T transposed - transition matrix transposed
		 * f_1:t - position estimate 
		 * 
		 *  <<<< f_1:0 is supposed to be the initial position estimate, which I just made equal for
		 *  	every position >>>>
		 *  
		 *  Helpful stuff:
		 *  - lecture 9 (slides 18 and 19)
		 *  - 15.3.1 section in the book
		 *  - equation 15.5 and surrounding info
		 */
		
		int states = X_LENGTH*Y_LENGTH*4;
		double[][] obsMatrix;
		
		if (robot.sensor.x == -1) {
			obsMatrix = Sensor.getMatrixFromVector(Sensor.observationVectors[Sensor.observationVectors.length-1]);
		} else {
			obsMatrix = Sensor.getMatrixFromVector(Sensor.observationVectors[Robot.tMatrixFormula(robot.sensor.x, robot.sensor.y, 0)/4]);
		}
		
		
		double[][] tTransposed = new double[states][states];
		
		for (int i = 0; i < states; i++) {
			int start = 1;
			
			for (int k = start; k < states; k++) {
				tTransposed[i][k] = Robot.transitionMatrix[k][i];
				tTransposed[k][i] = Robot.transitionMatrix[i][k];
			}
			start++;
		}
		
		double[][] tempMatrixMultiply = new double[X_LENGTH*Y_LENGTH*4][X_LENGTH*Y_LENGTH*4];
		
		for (int i = 0; i < states; i++) {
			for (int j = 0; j < states; j++) {	
				for (int k = 0; k < states; k++) {
					tempMatrixMultiply[i][j] += obsMatrix[i][k] * tTransposed[k][j];
				}
			}
		}
		
		double[] fNonNormalized = new double[states];
		
		double sum = 0;
		
		for (int i = 0; i < states; i++) {
			for (int j = 0; j < states; j++) {
				fNonNormalized[i] += tempMatrixMultiply[i][j] * f[j];
			}
			sum += fNonNormalized[i];
		}
		
		for (int i = 0; i < states; i++) {
			f[i] = fNonNormalized[i] / sum;
		}
		
		return f[Robot.tMatrixFormula(x, y, 0)] + f[Robot.tMatrixFormula(x, y, 1)] + 
				f[Robot.tMatrixFormula(x, y, 2)] + f[Robot.tMatrixFormula(x, y, 3)];
	}
	
	

	@Override
	public double getOrXY(int rX, int rY, int x, int y, int h) {
		if(rX == -1 || rY == -1)
			return Sensor.observationVectors[Sensor.observationVectors.length-1][Robot.tMatrixFormula(x, y, 0)/4];
		return Sensor.observationVectors[Robot.tMatrixFormula(rX, rY, 0)/4][Robot.tMatrixFormula(x, y, 0)/4];
	}

	@Override
	public double getTProb(int x, int y, int h, int nX, int nY, int nH) {
		return Robot.transitionMatrix[Robot.tMatrixFormula(x, y, h)][Robot.tMatrixFormula(nX, nY, nH)];
	}

}
