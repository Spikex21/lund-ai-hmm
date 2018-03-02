package model.robot;

import java.util.Random;

import model.RobotManager;

public class Robot {

	public static double[][] transitionMatrix;
	
	public int x, y;
	public int heading;
	public Sensor sensor;
	
	public Robot(int xLength, int yLength) {
		fillTransitionMatrix(xLength, yLength);
		Random gen = new Random();
		x = gen.nextInt(xLength);
		y = gen.nextInt(yLength);
		heading = gen.nextInt(4);
		sensor = new Sensor(x, y);
	}
	
	public void move() {
		double moves[] = new double[4];
		if(isInBounds(x-1, y))
			moves[0] = transitionMatrix[tMatrixFormula(x, y, heading)][tMatrixFormula(x-1, y, 0)];
		if(isInBounds(x, y+1))
			moves[1] = transitionMatrix[tMatrixFormula(x, y, heading)][tMatrixFormula(x, y+1, 1)];
		if(isInBounds(x+1, y))
			moves[2] = transitionMatrix[tMatrixFormula(x, y, heading)][tMatrixFormula(x+1, y, 2)];
		if(isInBounds(x, y-1))
			moves[3] = transitionMatrix[tMatrixFormula(x, y, heading)][tMatrixFormula(x, y-1, 3)];
		
		Random gen = new Random();
		double roll = gen.nextDouble();
		
		for(int i = 0; i < moves.length; i++) {
			if(roll < moves[i]) {
				heading = i;
				break;
			}
			roll -= moves[i];
		}
		x += xFromH(heading);
		y += yFromH(heading);
		
		sensor.update(x, y);
	}
	
	public static void fillTransitionMatrix(int xLength, int yLength) {
		transitionMatrix = new double[xLength*yLength*4][xLength*yLength*4];
		for(int x = 0; x < xLength; x++) {
			for(int y = 0; y < yLength; y++) {
				for(int h = 0; h < 4; h++) {
					int newX = x + xFromH(h);
					int newY = y + yFromH(h);
					double remainingChance = 1;
					if(isInBounds(newX, newY)) {
						transitionMatrix[tMatrixFormula(x,y,h)][tMatrixFormula(newX,newY,h)] = .7;
						remainingChance = .3;
					}
					int neighborCount = 0;
					for(int ih = 0; ih < 4; ih++) {
						if(ih == h)
							continue;
						newX = x + xFromH(ih);
						newY = y + yFromH(ih);
						if(isInBounds(newX, newY))
							neighborCount++;
					}
					double neighborOdds = remainingChance/neighborCount;
					for(int ih = 0; ih < 4; ih++) {
						if(ih == h)
							continue;
						newX = x + xFromH(ih);
						newY = y + yFromH(ih);
						if(isInBounds(newX, newY))
							transitionMatrix[tMatrixFormula(x,y,h)][tMatrixFormula(newX,newY,ih)] = neighborOdds;		
					}
				}
			}
		}
	}
	public static int tMatrixFormula(int x, int y, int h) {
		if(!isInBounds(x, y))
			throw new IllegalArgumentException();
		return (x * RobotManager.Y_LENGTH + y) * 4 + h;
	}
	private static int xFromH(int h) {
		if(h > 3 || h < 0)
			throw new IllegalArgumentException();
		return (h-1)%2;
	}
	private static int yFromH(int h) {
		if(h > 3 || h < 0)
			throw new IllegalArgumentException();
		return (2-h)%2;
	}
	public static boolean isInBounds(int x, int y) {
		return (x < RobotManager.X_LENGTH && x >= 0 && y < RobotManager.Y_LENGTH && y >= 0);
	}
	
}
