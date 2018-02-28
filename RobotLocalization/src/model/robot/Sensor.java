package model.robot;

import java.util.Random;

import model.RobotManager;

public class Sensor {
	public static final double OWN_SQUARE_CHANCE = .1;
	public static final double NEIGHBOR_CHANCE = .05;
	public static final double FAR_NEIGHBOR_CHANCE = .025;
	public static final double MISS_CHANCE = .1;
	
	public static double[][] observationVectors;
	
	public int x, y;
	
	public Sensor(int x, int y) {
		populateObservationVectors();
		update(x, y);
	}
	
	public void update(int x, int y) {
		double totalNeighborChance = 8*NEIGHBOR_CHANCE;
		double totalFarNeighborChance = 16*FAR_NEIGHBOR_CHANCE;
		
		Random gen = new Random();
		
		double roll = gen.nextDouble();
		
		this.x = -1;
		this.y = -1;
		
		if(roll < MISS_CHANCE) {
			return;
		}
		roll -= MISS_CHANCE;
		
		if(roll < OWN_SQUARE_CHANCE) {
			this.x = x;
			this.y = y;
			return;
		}
		roll -= OWN_SQUARE_CHANCE;
		
		if(roll < totalNeighborChance) {
			int[] xChange = {0,1,1,1,0,-1,-1,-1};
			int[] yChange = {1,1,0,-1,-1,-1,0,1};
			int square = gen.nextInt(8);
			if(Robot.isInBounds(x + xChange[square], y + yChange[square])) {
				this.x = x + xChange[square];
				this.y = y + yChange[square];
			}

			return;
		}
		roll -= totalNeighborChance;
		
		if(roll < totalFarNeighborChance) {
			int[] xChange = {0,1,2,2,2,2,2,1,0,-1,-2,-2,-2,-2,-2,-1};
			int[] yChange = {2,2,2,1,0,-1,-2,-2,-2,-2,-2,-1,0,1,2,2};
			int square = gen.nextInt(16);
			if(Robot.isInBounds(x + xChange[square], y + yChange[square])) {
				this.x = x + xChange[square];
				this.y = y + yChange[square];
			}
			return;
		}
		throw new IllegalArgumentException("Something went horribly wrong");
	}
	
	public static void populateObservationVectors() {
		observationVectors = new double[RobotManager.X_LENGTH*RobotManager.Y_LENGTH+1][RobotManager.X_LENGTH*RobotManager.Y_LENGTH];
		for(int x = 0; x < RobotManager.X_LENGTH; x++) {
			for(int y = 0; y < RobotManager.Y_LENGTH; y++) {
				double[] vector = observationVectors[Robot.tMatrixFormula(x, y, 0)/4];
				for(int xi = -2; xi <= 2; xi++) {
					for(int yi = -2; yi <= 2; yi++) {
						if(Robot.isInBounds(x+xi, y+yi)) {
							vector[Robot.tMatrixFormula(x+xi, y+yi, 0)/4] = OWN_SQUARE_CHANCE/(Math.pow(2,Math.max(Math.abs(xi), Math.abs(yi))));
						} else {
							observationVectors[observationVectors.length-1][Robot.tMatrixFormula(x, y, 0)/4] += OWN_SQUARE_CHANCE/(Math.pow(2,Math.max(Math.abs(xi), Math.abs(yi))));
						}
					}
				}
			}
		}
		for(int x = 0; x < RobotManager.X_LENGTH; x++) {
			for(int y = 0; y < RobotManager.Y_LENGTH; y++) {
				observationVectors[observationVectors.length-1][Robot.tMatrixFormula(x, y, 0)/4] += .1;
			}
		}
		
	}
	
	public static double[][] getMatrixFromVector(double[] vector){
		double[][] matrix = new double[RobotManager.X_LENGTH*RobotManager.Y_LENGTH*4][RobotManager.X_LENGTH*RobotManager.Y_LENGTH*4];
		for(int x = 0; x < RobotManager.X_LENGTH; x++) {
			for(int y = 0; y < RobotManager.Y_LENGTH; y++) {
				for(int h = 0; h < 4; h++) {
					matrix[Robot.tMatrixFormula(x, y, h)][Robot.tMatrixFormula(x, y, h)] = vector[Robot.tMatrixFormula(x, y, 0)/4];
				}
			}
		}
		return matrix;
	}
	
	
}
