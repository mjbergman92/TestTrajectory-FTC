package Main;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartMouseEvent;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.RectangleInsets;

public class GraphTrajectory {

	static String baseInstructions = "Enter Trajectory #. Then Press the 'r' Key", trajSet = "Trajectory ", running = " Running", pause = " Paused", pressR = " Press the 'r' Key to Run Trajectory", wait = "Please Wait";
	static double counter, timePassed; //tracking and calculating for loop and time sensitive data for the graphs
	static double originalTime; //set at the beginning when load is clicked. current time of the system in milliseconds
	static boolean run = false; //for the while loop for the "Load" and "End"
	static int step = 1; //the step of the trajectory. default step 1
	static boolean paused = false; //the boolean used to determine if the graph is paused controlled by the enter/return key
	static int traj = 0; //the trajectory choice
	static boolean setup = false;
	static boolean firstTime = true;
	static boolean wait1 = true;
	
	public static void main(String args[]) {
		
		TrajectorySetup trajectorySetup = new TrajectorySetup();
		
		//create and configure the window
		JFrame window = new JFrame();
		window.setTitle("Trajectory GUI");
		window.setSize(1450,1000);
		window.isResizable();
		window.setLayout(new BorderLayout());
		window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);;
		
		//create and configure the button and label
		JButton button = new JButton("End");
		button.setAlignmentX(145);
		JLabel label = new JLabel();
		label.setText(baseInstructions);
		label.setFont(new Font(Font.DIALOG, Font.PLAIN, 25));
		JPanel topPanel = new JPanel();
		topPanel.setLayout(new BorderLayout());
		topPanel.add(button, BorderLayout.EAST);
		topPanel.add(label, BorderLayout.CENTER);
		window.add(topPanel,BorderLayout.NORTH);
		
		//create panels for window
		JPanel Lpanel = new JPanel();
		Lpanel.setLayout(new BorderLayout());
		JPanel Rpanel = new JPanel();
		Rpanel.setLayout(new BorderLayout());
		
		//create xy line graph for x and y coordinates
		XYSeries leftSeries = new XYSeries("Left Side Robot");
		XYSeries rightSeries = new XYSeries("Right Side Robot");
		XYSeries lineAcross = new XYSeries("Line Across");
		XYSeries robot = new XYSeries("Robot");
		XYSeriesCollection dataset = new XYSeriesCollection(leftSeries);
		dataset.addSeries(rightSeries);
		dataset.addSeries(lineAcross);
		dataset.addSeries(robot);
		dataset.setAutoWidth(false);
		JFreeChart chart = ChartFactory.createXYLineChart("Pathfinder Trajectory", "Distance X (Inches)", "Distance Y (Inches)", dataset);
		chart.getXYPlot().setBackgroundImage(new ImageIcon("Pic.png").getImage());
		chart.setPadding(new RectangleInsets(0, 120, 0, 120));
		NumberAxis numberRangeAxis = (NumberAxis)chart.getXYPlot().getRangeAxis();
		numberRangeAxis.setRange(-66, 66);
		NumberAxis numberDomainAxis = (NumberAxis)chart.getXYPlot().getDomainAxis();
		numberDomainAxis.setRange(0, 132);
		ChartPanel mainChartPanel = new ChartPanel(chart);
		Lpanel.add(mainChartPanel,BorderLayout.NORTH);
		
		//create xy line graph for velocity
		XYSeries leftVSeries = new XYSeries("Left Velocity");
		XYSeries rightVSeries = new XYSeries("Right Velocity");
		XYSeries leftASeries = new XYSeries("Left Acceleration");
		XYSeries rightASeries = new XYSeries("Right Acceleration");
		XYSeries leftJSeries = new XYSeries("Left Jerk");
		XYSeries rightJSeries = new XYSeries("Right Jerk");
		XYSeriesCollection VAJdataset = new XYSeriesCollection(leftVSeries);
		VAJdataset.addSeries(rightVSeries);
		VAJdataset.addSeries(leftASeries);
		VAJdataset.addSeries(rightASeries);
		VAJdataset.addSeries(leftJSeries);
		VAJdataset.addSeries(rightJSeries);
		JFreeChart VAJchart = ChartFactory.createXYLineChart("Velocity, Acceleration, and Jerk", "Time (Seconds)", "Distance/Second (Inches/Second), Distance/Second/Second (Inches/Second^2), AND Distance/Second/Second/Second (Inches/Second^3)", VAJdataset);
		Rpanel.add(new ChartPanel(VAJchart),BorderLayout.NORTH);
		
		//create xy line graph for left position/distance
		XYSeries leftDSeries = new XYSeries("Left Distance");
		XYSeriesCollection dataLset = new XYSeriesCollection(leftVSeries);
		dataLset.addSeries(leftDSeries);
		JFreeChart Lchart = ChartFactory.createXYLineChart("Left Side Distance and Velocity", "Time (Seconds)", "Distance (Inches) AND Distance/Second (Inches/Second", dataLset);
		ChartPanel LCPanel = new ChartPanel(Lchart);
		LCPanel.getChart().setBackgroundPaint(Color.RED);
		Lpanel.add(LCPanel,BorderLayout.SOUTH);
		
		//create xy line graph for right position/distance
		XYSeries rightDSeries = new XYSeries("Right Distance");
		XYSeriesCollection dataRset = new XYSeriesCollection(rightVSeries);
		dataRset.addSeries(rightDSeries);
		JFreeChart Rchart = ChartFactory.createXYLineChart("Right Side Distance and Velocity", "Time (Seconds)", "Distance (Inches) AND Distance/Second (Inches/Second", dataRset);
		ChartPanel RCPanel = new ChartPanel(Rchart);
		RCPanel.getChart().setBackgroundPaint(Color.BLUE);
		Rpanel.add(new ChartPanel(Rchart),BorderLayout.SOUTH);
		
		//add panels to window
		window.add(Lpanel, BorderLayout.WEST);
		window.add(Rpanel, BorderLayout.EAST);
		
		button.addKeyListener(new KeyListener() {

			@Override
			public void keyPressed(KeyEvent e) {
				
				if(e.getKeyCode() == 10) { //enter/return key 
					
					if(paused == true) {
						
						paused = false;
						label.setText(trajSet + traj + running);
						
					}else {
						
						paused = true;
						label.setText(trajSet + traj + pause);
						
					}
					
				}else if(!paused) {
					
					if(e.getKeyChar() == 'r') {
						
						if(setup) {
							
							if(wait1) {
								
								label.setFont(new Font(Font.DIALOG, Font.BOLD, 30));
								
							}else {
								if(button.getText() == "Load") {
									
									button.doClick();
									label.setText(trajSet + traj + running);
									label.setFont(new Font(Font.DIALOG, Font.PLAIN, 25));
									
								}else {
									
									button.doClick();
									label.setText(trajSet + traj + "." + pressR + " OR Select a Different Trajectory.");
									label.setFont(new Font(Font.DIALOG, Font.PLAIN, 25));
									firstTime = true;
									
	
								}
							}
						}else {
							
							label.setText(baseInstructions);
							label.setFont(new Font(Font.DIALOG, Font.BOLD, 30));
							
						}
						
					}else if(e.getKeyChar() == '1') {
						
						traj = 1;
						setup = true;
						firstTime = true;
						label.setText(trajSet + traj + "." + pressR);
						label.setFont(new Font(Font.DIALOG, Font.PLAIN, 25));
						
					}else if(e.getKeyChar() == '2') {
						
						traj = 2;
						setup = true;
						firstTime = true;
						label.setText(trajSet + traj + "." + pressR);
						label.setFont(new Font(Font.DIALOG, Font.PLAIN, 25));
						
					}else if(e.getKeyChar() == '3') {
						
						traj = 3;
						setup = true;
						firstTime = true;
						label.setText(trajSet + traj + "." + pressR);
						label.setFont(new Font(Font.DIALOG, Font.PLAIN, 25));
						
					}else if(e.getKeyChar() == '4') {
						
						traj = 4;
						setup = true;
						firstTime = true;
						label.setText(trajSet + traj + "." + pressR);
						label.setFont(new Font(Font.DIALOG, Font.PLAIN, 25));
						
					}else if(e.getKeyChar() == '5') {
						
						traj = 5;
						setup = true;
						firstTime = true;
						label.setText(trajSet + traj + "." + pressR);
						label.setFont(new Font(Font.DIALOG, Font.PLAIN, 25));
						
					}else if(e.getKeyChar() == '6') {
						
						traj = 6;
						setup = true;
						firstTime = true;
						label.setText(trajSet + traj + "." + pressR);
						label.setFont(new Font(Font.DIALOG, Font.PLAIN, 25));
						
					}else if(e.getKeyChar() == '7') {
						
					}else if(e.getKeyChar() == '8') {
						
					}else {
						
						if(!setup) {
							
							label.setText(baseInstructions);
							label.setFont(new Font(Font.DIALOG, Font.BOLD, 30));
							
						}else {
							
						}
					}
				}
			}

			@Override
			public void keyTyped(KeyEvent e) {
				
			}

			@Override
			public void keyReleased(KeyEvent e) {
				
			}
		});
		
		button.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				if(button.getText().equals("Load")) {
					
					button.setText("End");
					
					run = true;
					counter = 0;
					originalTime = System.currentTimeMillis();
					
					Thread thread = new Thread() {
						@Override public void run() {
							while(run) {
								
								if(!paused) {
									
									if(((System.currentTimeMillis() - originalTime)/1000) >= counter*.020 + .020) {
										
										try {
											switch(step) {
											case 1:
												if(trajectorySetup.setupisFinished()) {
													step++;
													trajectorySetup.setup(2);
													originalTime = System.currentTimeMillis() - (((counter * .020) + .020) * 1000);	
													leftSeries.clear();
													rightSeries.clear();
												}
												break;
											case 2:
												if(trajectorySetup.setupisFinished()) {
													step++;
													trajectorySetup.setup(3);
													originalTime = System.currentTimeMillis() - (((counter * .020) + .020) * 1000);	
													leftSeries.clear();
													rightSeries.clear();
												}
												break;
											case 3:
												if(trajectorySetup.setupisFinished()) {
													step++;
													trajectorySetup.setup(4);
													originalTime = System.currentTimeMillis() - (((counter * .020) + .020) * 1000);	
													leftSeries.clear();
													rightSeries.clear();
												}
												break;
											case 4:
												if(trajectorySetup.setupisFinished()) {
													step++;
													trajectorySetup.setup(5);
													originalTime = System.currentTimeMillis() - (((counter * .020) + .020) * 1000);	
													leftSeries.clear();
													rightSeries.clear();
												}
												break;
											default:
												break;
											}
											timePassed = trajectorySetup.timePassed();
											lineAcross.clear();
											double leftX = trajectorySetup.leftXTrajectory(), leftY = trajectorySetup.leftYTrajectory(), rightX = trajectorySetup.rightXTrajectory(), rightY = trajectorySetup.rightYTrajectory();
											leftSeries.add(leftX, leftY);
											rightSeries.add(rightX, rightY);
											trajectorySetup.robotBox(leftX,leftY,rightX,rightY);
											trajectorySetup.counter = 0;
											robot.clear();
											for(int i = 0; i <= 24; i++){
												trajectorySetup.drawRobot();
												robot.add(trajectorySetup.xRobotOut1, trajectorySetup.yRobotOut1);
												robot.add(trajectorySetup.xRobotOut2, trajectorySetup.yRobotOut2);
											}
											lineAcross.add(leftX, leftY);
											lineAcross.add(rightX, rightY);
											
											leftVSeries.add(timePassed, trajectorySetup.leftVelocity());
											rightVSeries.add(timePassed, trajectorySetup.rightVelocity());
											leftASeries.add(timePassed, trajectorySetup.leftAcceleration());
											rightASeries.add(timePassed, trajectorySetup.rightAcceleration());
											leftJSeries.add(timePassed, trajectorySetup.leftJerk());
											rightJSeries.add(timePassed, trajectorySetup.rightJerk());
											
											leftDSeries.add(timePassed, trajectorySetup.leftDistance());
											rightDSeries.add(timePassed, trajectorySetup.rightDistance());
											
										}catch(Exception e) {}
										counter++;
									}
								}
							}
						}
					};
				thread.start();
				}else if(button.getText().equals("End")){
					Thread starterThread = new Thread() {
						@Override public void run() {
							button.setText("Load");
							run = false;
							trajectorySetup.resetCounters();
							
							step = 1;
							wait1 = true;
							
							while(!run) {
								if(setup && firstTime) {
									
									trajectorySetup.setup(2);
									trajectorySetup.setup(3);
									trajectorySetup.setup(4);
									trajectorySetup.setup(5);
									trajectorySetup.setup(1);

									label.setText(trajSet + traj + "." + pressR);
									wait1 = false;
									firstTime = false;
								}
								leftSeries.clear();
								rightSeries.clear();
								robot.clear();
								lineAcross.clear();
								leftVSeries.clear();
								rightVSeries.clear();
								leftASeries.clear();
								rightASeries.clear();
								leftJSeries.clear();
								rightJSeries.clear();
								
								leftDSeries.clear();
								rightDSeries.clear();
								trajectorySetup.iTime = 0;
								System.out.print("");
							}
							
							while(!trajectorySetup.checkDone) {
								
							}
							
							System.out.print("");
								
						}
					};
					starterThread.start();
				}
			}
			
		});
		
		mainChartPanel.addMouseListener(new MouseListener() {

			@Override
			public void mouseClicked(MouseEvent e) {
				
				
				Point2D p = mainChartPanel.translateScreenToJava2D(new ChartMouseEvent(chart, e, null).getTrigger().getPoint());
				Rectangle2D plotArea = mainChartPanel.getScreenDataArea();
				XYPlot plot = (XYPlot) chart.getPlot(); // your plot
				double chartX = plot.getDomainAxis().java2DToValue(p.getX(), plotArea, plot.getDomainAxisEdge());
				double chartY = plot.getRangeAxis().java2DToValue(p.getY(), plotArea, plot.getRangeAxisEdge());
				label.setText(chartX + "");
				
			}

			@Override
			public void mousePressed(MouseEvent e) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void mouseReleased(MouseEvent e) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void mouseEntered(MouseEvent e) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void mouseExited(MouseEvent e) {
				// TODO Auto-generated method stub
				
			}
			
			
		});
		
		//show the window
		window.setVisible(true);
		button.doClick();
		
	}
}
