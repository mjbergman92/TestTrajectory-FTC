package Main;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Scanner;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.modifiers.TankModifier;

public class TrajectorySetup {
	
	//assumes the pivot is in the "exact" center of the robot
	double wheelBase_width = 16.375, wheelBase_length = 16;
	double robot_width = 18, robot_length = 20;
	Trajectory left, right;
	Trajectory.Segment segLeftX, segLeftY, segRightX, segRightY, segLV, segRV, segLA, segRA, segLJ, segRJ, segLP, segRP, segTime, segTraj;
	int i1, i2, i3, i4, iLV, iRV, iLA, iRA, iLJ, iRJ, iLP, iRP, iTime, iTraj;
	public double left1x,  right1x, left1y, right1y, left2x,  right2x, left2y, right2y;
	Trajectory trajectory;
	boolean backward;
	public int counter;
	public double xRobotOut1, yRobotOut1, xRobotOut2, yRobotOut2;
	public int posTraj;
	Waypoint[] points;
	private final double absMaxVelocity = 16;
	private double setVelocity = 0;
	public boolean checkDone = false;
	private double robotLoopTime = 0.020;
	public boolean[] firstTimeThroughDone = {false, false, false, false, false, false, false, false};
	
	
	public TrajectorySetup() {
		
	}
	
	public void setup(int step) {
		
		
		checkDone = false;
		posTraj = GraphTrajectory.traj;
		resetCounters();
		waypointsStep(step);
		
		if(firstTimeThroughDone[posTraj - 1]) {
			
			testTrajectory(step);
			Scanner scanner;
			double velocity = 0;
			try {
				
				scanner = new Scanner(new File("trajectory" + posTraj + "/velocity" + "/velocity" + step + posTraj + ".csv"));
				velocity = Double.parseDouble(scanner.next());
				scanner.close();
				
			} catch (FileNotFoundException e1) {
				
				e1.printStackTrace();
				
			}
			Trajectory.Config configuration = new Trajectory.Config(FitMethod.HERMITE_CUBIC, 1000, robotLoopTime, velocity, 180, 400);
			trajectory = Pathfinder.generate(points, configuration);
			
			resetCounters();
			TankModifier modifier = new TankModifier(trajectory);
			modifier.modify(wheelBase_width);
			if(posTraj != 7 && posTraj != 8) {
				if(step == 1) {
					left = modifier.getLeftTrajectory();
					right = modifier.getRightTrajectory();
					backward = false;
				}else if(step == 2) {
					left = modifier.getRightTrajectory();
					right = modifier.getLeftTrajectory();
					backward = true;
				}else if(step == 3) {
					left = modifier.getLeftTrajectory();
					right = modifier.getRightTrajectory();
					backward = false;
				}else if(step == 4) {
					left = modifier.getRightTrajectory();
					right = modifier.getLeftTrajectory();
					backward = true;
				}else if(step == 5) {
					left = modifier.getLeftTrajectory();
					right = modifier.getRightTrajectory();
					backward = false;
				}
			}else if(posTraj == 7 || posTraj == 8){
				if(step == 1) {
					left = modifier.getLeftTrajectory();
					right = modifier.getRightTrajectory();
					backward = false;
				}else if(step == 2) {
					left = modifier.getLeftTrajectory();
					right = modifier.getRightTrajectory();
					backward = false;
				}else if(step == 3) {
					left = modifier.getRightTrajectory();
					right = modifier.getLeftTrajectory();
					backward = true;
				}
			}
			
			
		}else { 
			
			new File("trajectory" + posTraj + "/center" + "/trajectorystep" + step + "traj" + posTraj + ".csv").delete();
			new File("trajectory" + posTraj + "/right" + "trajectorystep" + step + "traj" + posTraj + ".csv").delete();
			new File("trajectory" + posTraj + "left" + "trajectorystep" + step + "traj" + posTraj + ".csv").delete();
			new File("trajectory" + posTraj + "/velocity" + "/velocity" + step + posTraj + ".csv").delete();

			
			testTrajectory(step);
			Trajectory.Config configuration = new Trajectory.Config(FitMethod.HERMITE_CUBIC, 1000, robotLoopTime, setVelocity, 180, 400);
			trajectory = Pathfinder.generate(points, configuration);
			Pathfinder.writeToCSV(new File("trajectory" + posTraj + "/center" + "/trajectorystep" + step + "traj" + posTraj + ".csv"), trajectory);
			
			resetCounters();
			TankModifier modifier = new TankModifier(trajectory);
			modifier.modify(wheelBase_width);
			if(posTraj != 7 && posTraj != 8) {
				if(step == 1) {
					left = modifier.getLeftTrajectory();
					right = modifier.getRightTrajectory();
					backward = false;
				}else if(step == 2) {
					left = modifier.getRightTrajectory();
					right = modifier.getLeftTrajectory();
					backward = true;
				}else if(step == 3) {
					left = modifier.getLeftTrajectory();
					right = modifier.getRightTrajectory();
					backward = false;
				}else if(step == 4) {
					left = modifier.getRightTrajectory();
					right = modifier.getLeftTrajectory();
					backward = true;
				}else if(step == 5) {
					left = modifier.getLeftTrajectory();
					right = modifier.getRightTrajectory();
					backward = false;
				}
			}else if(posTraj == 7 || posTraj == 8){
				if(step == 1) {
					left = modifier.getLeftTrajectory();
					right = modifier.getRightTrajectory();
					backward = false;
				}else if(step == 2) {
					left = modifier.getLeftTrajectory();
					right = modifier.getRightTrajectory();
					backward = false;
				}else if(step == 3) {
					left = modifier.getRightTrajectory();
					right = modifier.getLeftTrajectory();
					backward = true;
				}
			}
			
			Pathfinder.writeToCSV(new File("trajectory" + posTraj + "/right" + "trajectorystep" + step + "traj" + posTraj + ".csv"), right);
			Pathfinder.writeToCSV(new File("trajectory" + posTraj + "/left" + "trajectorystep" + step + "traj" + posTraj + ".csv"), left);
			
			try {
				
				FileWriter fw = new FileWriter("trajectory" + posTraj + "/velocity" + "/velocity" + step + posTraj + ".csv");
				BufferedWriter bw = new BufferedWriter(fw);
				PrintWriter pw = new PrintWriter(bw);
				
				pw.print(setVelocity);
				pw.flush();
				pw.close();
				
			} catch (IOException e) {
				
				e.printStackTrace();
			
			}
		}
		
		checkDone = true;
		
	}
	
	private void testTrajectory(int step) {
				
		long originalTime = System.currentTimeMillis();
		double countTimeTest = 1;
		boolean good;
		while(countTimeTest <= absMaxVelocity) {
			
			if(System.currentTimeMillis() - originalTime > countTimeTest * 40) {
				good = true;
				resetCounters();
				double test = countTimeTest;
				//System.out.println(test);
				Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_CUBIC, 1000, robotLoopTime, test, 180, 400);
				Trajectory trajectory = Pathfinder.generate(points, config);
				TankModifier modifier = new TankModifier(trajectory);
				modifier.modify(wheelBase_width);
				Trajectory left = modifier.getLeftTrajectory();
				Trajectory right = modifier.getRightTrajectory();
				
				for(int l = 0; l < left.length(); l++) {
					
					double leftVel = left.get(l).velocity;
					
					if(leftVel > absMaxVelocity) {
						
						good = false;
						
					}
				}
				
				for(int r = 0; r < right.length(); r++) {
					
					double rightVel = right.get(r).velocity;
					
					if(rightVel > absMaxVelocity) {
						
						good = false;
						
					}
				}
				
				if(good) {
					
					setVelocity = test;
					//System.out.println(test);
					
				}			
				
				countTimeTest = countTimeTest + 0.25;
				
			}
		}
	}
	
	private void waypointsStep(int step) {
		switch(posTraj) {
		case 1: //center left switch
			
			this.getTraj1Points(step);
			
			break;
		case 2: //center right switch
			
			this.getTraj2Points(step);
			
			break;
		case 3: //left side switch
			
			this.getTraj3Points(step);
			
			break;
		case 4: //right side switch
			
			this.getTraj4Points(step);
			
			break;
		case 5: //left side scale
			
			this.getTraj5Points(step);
			
			break;
		case 6: //right side scale
			
			this.getTraj6Points(step);
			
			break;
		case 7: //left side scale cross
			
			this.getTraj7Points(step);
			
			break;
		case 8: //right side scale cross
			
			this.getTraj8Points(step);
			
			break;
	
		}
	}
	
	private void getTraj1Points(int step) {
		
		double originalX = 66 - (Math.sqrt(Math.pow(11.5 + 3.125, 2) / 2));
		double originalY = (Math.sqrt(Math.pow(11.5 + 3.125, 2) / 2));
		
		if(step == 1) {
			
			setPoints(step);
			
			
		}else if(step == 2) {
			
			
			points = new Waypoint[] {
					new Waypoint(11, -35, Pathfinder.d2r(270)),
					new Waypoint(11, 58, Pathfinder.d2r(270))
				};
			
		}else if(step == 3) {
			
			points = new Waypoint[] {
					new Waypoint(originalX, originalY, Pathfinder.d2r(135)),
					new Waypoint(originalX - Math.sqrt(Math.pow(20, 2) / 2), originalY + Math.sqrt(Math.pow(20, 2) / 2), Pathfinder.d2r(135))
				};
			
		}else if(step == 4) {
			
			points = new Waypoint[] {
					new Waypoint(originalX - Math.sqrt(Math.pow(20, 2) / 2), originalY + Math.sqrt(Math.pow(20, 2) / 2), Pathfinder.d2r(135)),
					new Waypoint(originalX, originalY, Pathfinder.d2r(135))
				};
			
		}else if(step == 5) {
			
			points = new Waypoint[] {
					new Waypoint(originalX, originalY, Pathfinder.d2r(135)),
					new Waypoint(originalX - Math.sqrt(Math.pow(20, 2) / 2), originalY + Math.sqrt(Math.pow(20, 2) / 2), Pathfinder.d2r(135))
				};
		}
		
	}
	
	private void getTraj2Points(int step) {
		
		if(step == 1) {
			
			points = new Waypoint[] {
					new Waypoint(66 - 12.5, -15.5, Pathfinder.d2r(-92)),
					//new Waypoint(55, -45, Pathfinder.d2r(-110)),
					new Waypoint(27,-66 + 14,Pathfinder.d2r(-180))
				};
			
		}else if(step == 2) {
			
			points = new Waypoint[] {
					new Waypoint(140 - (robot_length/2),-72+(robot_width/2),0),
					new Waypoint(robot_length/2, 0, 0)
				};
			
		}else if(step == 3) {
			
			points = new Waypoint[] {
					new Waypoint(robot_length/2, 0, 0),
					new Waypoint(100 - robot_length/2,0,0)
				};
			
		}else if(step == 4) {
			
			points = new Waypoint[] {
					new Waypoint(100 - robot_length/2,0,0),
					new Waypoint(robot_length/2, 0, 0)
				};
			
		}else if(step == 5) {
			
			points = new Waypoint[] {
					new Waypoint((robot_length/2), 0, 0),
					new Waypoint(140 - (robot_length/2),-72+(wheelBase_width/2),0)
				};
			
		}
		
	}
	
	private void getTraj3Points(int step) {
		
		if(step == 1) {
			
			points = new Waypoint[] {
					new Waypoint((robot_length/2),-(robot_width/2)+132,0),
					new Waypoint(120, 135, 0),
					new Waypoint(168, 76.44 + (robot_length/2),Pathfinder.d2r(270))
				};
			
		}else if(step == 2) {
			
			points = new Waypoint[] {
					new Waypoint(168, 76.44 + (robot_length/2),Pathfinder.d2r(270)),
					new Waypoint(245, 135,Pathfinder.d2r(-135))
				};
			
		}else if(step == 3) {
			
			points = new Waypoint[] {
					new Waypoint(245, 135,Pathfinder.d2r(-135)),
					new Waypoint(229, 90,Pathfinder.d2r(-180+45))
				};
			
		}else if(step == 4) {
			
			points = new Waypoint[] {
					new Waypoint(229, 90,Pathfinder.d2r(-180+45)),
					new Waypoint(245, 135,Pathfinder.d2r(-135))
				};
			
		}else if(step == 5) {
			
			points = new Waypoint[] {
					new Waypoint(245, 135,Pathfinder.d2r(-135)),
					new Waypoint(168, 76.44 + (robot_length/2),Pathfinder.d2r(270))
				};
			
		}
		
	}
	
	private void getTraj4Points(int step) {
		
		if(step == 1) {
			
			points = new Waypoint[] {
					new Waypoint((robot_length/2),(robot_width/2)-132,0),
					new Waypoint(140, -125,Pathfinder.d2r(22.5)),
					new Waypoint(168, -76.44 - (robot_length/2),Pathfinder.d2r(90))
				};
			
		}else if(step == 2) {
			
			points = new Waypoint[] {
					new Waypoint(168, -76.44 - (robot_length/2),Pathfinder.d2r(90)),
					new Waypoint(243, -120,Pathfinder.d2r(180 - 40))
				};
			
		}else if(step == 3) {
			
			points = new Waypoint[] {
					new Waypoint(243, -120,Pathfinder.d2r(180 - 40)),
					new Waypoint(209, -70 ,Pathfinder.d2r(180-45))
				};
			
		}else if(step == 4) {
			
			points = new Waypoint[] {
					new Waypoint(209, -70 ,Pathfinder.d2r(180-45)),
					new Waypoint(243, -120,Pathfinder.d2r(180 - 40))
				};
			
		}else if(step == 5) {
			
			points = new Waypoint[] {
					new Waypoint(243, -120,Pathfinder.d2r(180 - 40)),
					new Waypoint(168, -76.44 - (robot_length/2),Pathfinder.d2r(90))
				};
			
		}
		
	}
	
	private void getTraj5Points(int step) {
		
		if(step == 1) {
			
			points = new Waypoint[] {
					new Waypoint((robot_length/2),-(robot_width/2)+132,0),
					new Waypoint(280, 130, Pathfinder.d2r(-2.5)),
					new Waypoint(314, 90, Pathfinder.d2r(270))
				};
			
		}else if(step == 2) {
			
			points = new Waypoint[] {
					new Waypoint(314, 90, Pathfinder.d2r(270)),
					new Waypoint(314, 120, Pathfinder.d2r(270))
				};
			
		}else if(step == 3) {
			
			points = new Waypoint[] {
					new Waypoint(314, 120, Pathfinder.d2r(270)),
					new Waypoint(250, 90, Pathfinder.d2r(225)),
					new Waypoint(209, 72-6.5,Pathfinder.d2r(180))
				};
			
		}else if(step == 4) {
			
			points = new Waypoint[] {
					new Waypoint(209, 72-6.5,Pathfinder.d2r(180)),
					new Waypoint(250, 90, Pathfinder.d2r(225)),
					new Waypoint(314, 120, Pathfinder.d2r(270))
				};
			
		}else if(step == 5) {
			
			points = new Waypoint[] {
					new Waypoint(314, 120, Pathfinder.d2r(270)),
					new Waypoint(314, 90, Pathfinder.d2r(270))
				};
			
		}
		
	}
	
	private void getTraj6Points(int step) {
		
		if(step == 1) {
			
			points = new Waypoint[] {
					new Waypoint((robot_length/2),(robot_width/2)-132,0),
					new Waypoint(280, -130, Pathfinder.d2r(2.5)),
					new Waypoint(314, -90, Pathfinder.d2r(90))
				};
			
		}else if(step == 2) {
			
			points = new Waypoint[] {
					new Waypoint(314, -90, Pathfinder.d2r(90)),
					new Waypoint(314, -120, Pathfinder.d2r(90))
				};
			
		}else if(step == 3) {
			
			points = new Waypoint[] {
					new Waypoint(314, -120, Pathfinder.d2r(90)),
					new Waypoint(250, -90, Pathfinder.d2r(135)),
					new Waypoint(209, -72+6.5,Pathfinder.d2r(180))
				};
			
		}else if(step == 4) {
			
			points = new Waypoint[] {
					new Waypoint(209, -72+6.5,Pathfinder.d2r(180)),
					new Waypoint(250, -90, Pathfinder.d2r(135)),
					new Waypoint(314, -120, Pathfinder.d2r(90))
				};
			
		}else if(step == 5) {
			
			points = new Waypoint[] {
					new Waypoint(314, -120, Pathfinder.d2r(90)),
					new Waypoint(314, -90, Pathfinder.d2r(90))
				};
			
		}
	}
	
	private void getTraj7Points(int step) {
		
		if(step == 1) {
			
		}else if(step == 2) {
			
		}else if(step == 3) {
			
		}else if(step == 4) {
			
		}else if(step == 5) {
			
		}
		
		/*
		}else if(posTraj == 7) {//left side scale cross
			points = new Waypoint[] {
				new Waypoint((robot_length/2),-(robot_width/2)+132,0),
				new Waypoint(185, 72 + 4 + (robot_width/2),Pathfinder.d2r(0)),
				new Waypoint(209 + 4 + (robot_width/2), 70, Pathfinder.d2r(-90))
				remove line ? new Waypoint(300, -90 + 7, Pathfinder.d2r(0))
			};*/
				/*
		}else if(posTraj == 7) {	
			points = new Waypoint[] {
				new Waypoint(209 + 4 + (robot_width/2), 70, Pathfinder.d2r(-90)),
				new Waypoint(209 + 4 + (robot_width/2), -50, Pathfinder.d2r(-90)),
				new Waypoint(300, -90 + 7, Pathfinder.d2r(0))
			};*/
		
	}
	
	private void getTraj8Points(int step) {
		
		if(step == 1) {
			
		}else if(step == 2) {
			
		}else if(step == 3) {
			
		}else if(step == 4) {
			
		}else if(step == 5) {
			
		}
		
	}
	
	private void setPoints(int step) {
		
		Scanner in;
		try {
			
			in = new Scanner(new File("trajectory" + posTraj + "/points" + "/trajectory" + posTraj + "step" + step + "points.csv"));
		
				
			ArrayList<String[]> waypoints = new ArrayList<String[]>();
			for(int i = 0; i < 3; i++) {
				
				String inLine = in.nextLine();
				System.out.println(inLine);
				String lines[] = inLine.split(",");
				switch(i + 1) {
				
				case 1:
					waypoints.add(0, lines);;
					break;
				case 2:
					waypoints.add(1, lines);
					break;
				case 3:
					waypoints.add(2, lines);
					break;
				
				}
				
			}
			
			float xPoint1 = Float.parseFloat(waypoints.get(0)[0]);
			float yPoint1 = Float.parseFloat(waypoints.get(0)[1]);
			float hPoint1 = Float.parseFloat(waypoints.get(0)[2]);
			float xPoint2 = Float.parseFloat(waypoints.get(1)[0]);
			float yPoint2 = Float.parseFloat(waypoints.get(1)[1]);
			float hPoint2 = Float.parseFloat(waypoints.get(1)[2]);
			float xPoint3 = Float.parseFloat(waypoints.get(2)[0]);
			float yPoint3 = Float.parseFloat(waypoints.get(2)[1]);
			float hPoint3 = Float.parseFloat(waypoints.get(2)[2]);
			
			
			points = new Waypoint[] {
					new Waypoint(xPoint1, yPoint1, hPoint1),
					new Waypoint(xPoint2, yPoint2, hPoint2),
					new Waypoint(xPoint3, yPoint3, hPoint3)
				};

			
			in.close();
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			e.getMessage();
		}
		
	}
	
	public boolean setupisFinished() {
		return i1 == left.length();
	}
	
	public void resetCounters() {
		i1 = 0; i2 = 0; i3 = 0; i4 = 0; iLV = 0; iRV = 0; iLA = 0; iRA = 0; iLJ = 0; iRJ = 0; iLP = 0; iRP = 0; iTraj = 0;
	}
	
	public double leftXTrajectory() {
		if(i1 < left.length()) {
			segLeftX = left.get(i1);
			i1++;
		}
		return segLeftX.x;
	}
	
	public double leftYTrajectory() {
		if(i2 < left.length()) {
			segLeftY = left.get(i2);
			i2++;
		}
		return segLeftY.y;
	}
	
	public double rightXTrajectory() {
		if(i3 < right.length()) {
			segRightX = right.get(i3);
			i3++;
		}
		return segRightX.x;
	}
	
	public double rightYTrajectory() {
		if(i4 < right.length()) {
			segRightY = right.get(i4);
			i4++;
		}
		return segRightY.y;
	}
	
	public double leftVelocity() {
		if(iLV < left.length()) {
			segLV = left.get(iLV);
			iLV++;
		}
		return segLV.velocity;
	}
	
	public double rightVelocity() {
		if(iRV < right.length()) {
			segRV = right.get(iRV);
			iRV++;
		}
		return segRV.velocity;
	}
	
	public double leftAcceleration() {
		if(iLA < left.length()) {
			segLA = left.get(iLA);
			iLA++;
		}
		return segLA.acceleration;
	}
	
	public double rightAcceleration() {
		if(iRA < right.length()) {
			segRA = right.get(iRA);
			iRA++;
		}
		return segRA.acceleration;
	}
	
	public double leftJerk() {
		if(iLJ < left.length()) {
			segLJ = left.get(iLJ);
			iLJ++;
		}
		return segLJ.jerk;
	}
	
	public double rightJerk() {
		if(iRJ < right.length()) {
			segRJ = right.get(iRJ);
			iRJ++;
		}
		return segRJ.jerk;
	}
	
	public double leftDistance() {
		if(iLP < left.length()) {
			segLP = left.get(iLP);
			iLP++;
		}
		return segLP.position;
	}
	
	public double rightDistance() {
		if(iRP < right.length()) {
			segRP = right.get(iRP);
			iRP++;
		}
		return segRP.position;
	}
	
	public double timePassed() {
		//number of segments * timeStep = total time passed
			iTime++;
		return  iTime * robotLoopTime;
	}
	
	public void robotBox(double lx, double ly, double rx, double ry) {
		if(iTraj < trajectory.length()) {
			segTraj = trajectory.get(iTraj);
			iTraj++;
		}
		double x2 = 0;
		double xOutside = 0;
		double xInside = 0;
		double yOutside = 0;
		double yInside = 0;
		double lengthDiff = 0;
		double shortLength = 0;
		double longLength = 0;
		double lengthMult = robot_length/2;
		double widthMult = (robot_width - wheelBase_width)/2;
		double angle = segTraj.heading;
		while(angle > Math.PI) {
			angle -= 2 * Math.PI;
		}
		while(angle < -Math.PI) {
			angle += 2 * Math.PI;
		}
		
		if(ly == ry) {
			if(lx > rx) {
				left1x = lx + widthMult;
				right1x = rx - widthMult;
				left1y = ly - lengthMult;
				right1y = ry - lengthMult;
				left2x = lx + widthMult;
				right2x = rx - widthMult;
				left2y = ly + lengthMult;
				right2y = ry + lengthMult;
			}else {
				left1x = lx - widthMult;
				right1x = rx + widthMult;
				left1y = ly + lengthMult;
				right1y = ry + lengthMult;
				left2x = lx - widthMult;
				right2x = rx + widthMult;
				left2y = ly - lengthMult;
				right2y = ry - lengthMult;
			}
		}else if(lx == rx) {
			if(ly > ry) {
				left1x = lx + lengthMult;
				right1x = rx + lengthMult;
				left1y = ly + widthMult;
				right1y = ry - widthMult;
				left2x = lx - lengthMult;
				right2x = rx - lengthMult;
				left2y = ly + widthMult;
				right2y = ry - widthMult;
			}else {
				left1x = lx - lengthMult;
				right1x = rx - lengthMult;
				left1y = ly - widthMult;
				right1y = ry + widthMult;
				left2x = lx + lengthMult;
				right2x = rx + lengthMult;
				left2y = ly - widthMult;
				right2y = ry + widthMult;
			}
		}else{
			
			if(backward) {
				
				if(angle > 0) {
					
					angle -= Math.PI;
					
				}else {
					
					angle += Math.PI;
					
				}
				
			}
			
			if(ly > ry) {
				if(lx > rx) { 						//angle < 0 and > about -1.57
					angle = Math.abs(angle);
					x2 = widthMult / Math.sin(angle);
					lengthDiff = widthMult / Math.tan(angle);
					shortLength = lengthMult - lengthDiff;
					longLength = lengthMult + lengthDiff;
					xOutside = shortLength * Math.cos(angle) + x2;
					xInside = longLength * Math.cos(angle) - x2;
					yOutside = shortLength * Math.sin(angle);
					yInside = longLength * Math.sin(angle);
					
					left1x = lx + xOutside;
					right1x = rx + xInside;
					left1y = ly - yOutside;
					right1y = ry - yInside;
					left2x = lx - xInside;
					right2x = rx - xOutside;
					left2y = ly + yInside;
					right2y = ry + yOutside;
				}else{ 								//angle > 0 and < about 1.57 
					angle = Math.abs(angle);
					angle = (Math.PI / 2) - angle;
					x2 = widthMult / Math.sin(angle);
					lengthDiff = widthMult / Math.tan(angle);
					shortLength = lengthMult - lengthDiff;
					longLength = lengthMult + lengthDiff;
					xOutside = shortLength * Math.sin(angle);
					xInside = longLength * Math.sin(angle);
					yOutside = shortLength * Math.cos(angle) + x2;  
					yInside = longLength * Math.cos(angle) - x2;
					
					left1x = lx + xOutside;
					right1x = rx + xInside;
					left1y = ly + yOutside;
					right1y = ry + yInside;
					left2x = lx - xInside;
					right2x = rx - xOutside;
					left2y = ly - yInside;
					right2y = ry - yOutside;
					
				}
			}else {
				if(lx > rx) {						//angle < -1.57 and > about -3.14
					angle = Math.abs(angle);
					angle = angle - (Math.PI / 2);
					x2 = widthMult / Math.sin(angle);
					lengthDiff = widthMult / Math.tan(angle);
					shortLength = lengthMult - lengthDiff;
					longLength = lengthMult + lengthDiff;
					xOutside = shortLength * Math.sin(angle);
					xInside = longLength * Math.sin(angle);
					yOutside = shortLength * Math.cos(angle) + x2;  
					yInside = longLength * Math.cos(angle) - x2;
					
					left1x = lx - xOutside;
					right1x = rx - xInside;
					left1y = ly - yOutside;
					right1y = ry - yInside;
					left2x = lx + xInside;
					right2x = rx + xOutside;
					left2y = ly + yInside;
					right2y = ry + yOutside;
					
				}else{								//angle > 1.57 and < 3.14
					angle = Math.abs(angle);
					angle = Math.PI - angle;
					x2 = widthMult / Math.sin(angle);
					lengthDiff = widthMult / Math.tan(angle);
					shortLength = lengthMult - lengthDiff;
					longLength = lengthMult + lengthDiff;
					xOutside = shortLength * Math.cos(angle) + x2;
					xInside = longLength * Math.cos(angle) - x2;
					yOutside = shortLength * Math.sin(angle);
					yInside = longLength * Math.sin(angle);
					
					left1x = lx - xOutside;
					right1x = rx - xInside;
					left1y = ly + yOutside;
					right1y = ry + yInside;
					left2x = lx + xInside;
					right2x = rx + xOutside;
					left2y = ly - yInside;
					right2y = ry - yOutside;
					
				}
			}
		}
	
	}	
	
	public void drawRobot(){
		int countEvenOdd = counter;			

		double xChange1 = (left2x - left1x)/24;
		double yChange1 = (left2y - left1y)/24;
		double xChange2 = (left1x - right1x)/24;
		double yChange2 = (left1y - right1y)/24;
			
		while(countEvenOdd > 1){
			countEvenOdd -= 2;
		}
			
		if(countEvenOdd == 0){
			xRobotOut1 = left1x + xChange1*counter;
			yRobotOut1 = left1y + yChange1*counter;
			xRobotOut2 = left1x - xChange2*counter;
			yRobotOut2  = left1y - yChange2*counter;
		}else{
			xRobotOut1 = right1x + xChange1*counter;
			yRobotOut1 = right1y + yChange1*counter;
			xRobotOut2 = left2x - xChange2*counter;
			yRobotOut2  = left2y - yChange2*counter;
		}
	
		counter++;
	}

}

