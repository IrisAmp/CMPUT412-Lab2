import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;

public class RobotArm 
{
	public static final double L1 = 13.5; // cm
	public static final double L2 = 17.2; // cm
	public static final double CONVERT = Math.PI / 180.0;

	EV3MediumRegulatedMotor theta1;
	EV3MediumRegulatedMotor theta2;

	public RobotArm() {
		theta1 = new EV3MediumRegulatedMotor(MotorPort.A);
		theta2 = new EV3MediumRegulatedMotor(MotorPort.D);
	}
	
	public void finish(){
		theta1.close();
		theta2.close();
		System.out.println("Finished");
		Button.waitForAnyPress();
	}
	
	// Part 4
	public void turnDegrees(double angle1, double angle2) {
		theta1.setSpeed(50);
		theta2.setSpeed(50);

		// Motors are upside-down
		theta1.rotate((int) -angle1, true);
		theta2.rotate((int) -angle2, false);

		while (theta1.isMoving() || theta2.isMoving()){
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
			}
		}

		angle1 *= CONVERT;
		angle2 *= CONVERT;

		double x = (L1 * Math.cos(angle1)) + (L2 * Math.cos(angle1 + angle2));
		double y = (L1 * Math.sin(angle1)) + (L2 * Math.sin(angle1 + angle2));

		System.out.printf("x = %f\n", x);
		System.out.printf("y = %f\n", -y);
	}

	//Part 5
	public void calculateDistance() {
		theta1.resetTachoCount();
		theta2.resetTachoCount();

		System.out.printf("0: %d\n", theta1.getTachoCount());
		System.out.printf("0: %d\n", theta2.getTachoCount());

		System.out.printf("Record first...\n");
		Button.waitForAnyPress();

		int i_pos1 = theta1.getTachoCount();
		int i_pos2 = theta2.getTachoCount();

		System.out.printf("i_angle1: %d\n", i_pos1);
		System.out.printf("i_angle2: %d\n", i_pos2);

		System.out.printf("Record second...\n");
		Button.waitForAnyPress();

		int f_pos1 = theta1.getTachoCount();
		int f_pos2 = theta2.getTachoCount();

		System.out.printf("f_angle1: %d\n", f_pos1);
		System.out.printf("f_angle2: %d\n", f_pos2);

		double xi = (L1 * Math.cos(i_pos1 * CONVERT))
				+ (L2 * Math.cos((i_pos1 * CONVERT) + (i_pos2 * CONVERT)));
		double yi = (L1 * Math.sin(i_pos1 * CONVERT))
				+ (L2 * Math.sin((i_pos1 * CONVERT) + (i_pos2 * CONVERT)));

		double xf = (L1 * Math.cos(f_pos1 * CONVERT))
				+ (L2 * Math.cos((f_pos1 * CONVERT) + (f_pos2 * CONVERT)));
		double yf = (L1 * Math.sin(f_pos1 * CONVERT))
				+ (L2 * Math.sin((f_pos1 * CONVERT) + (f_pos2 * CONVERT)));

		double dist = Math.hypot((yf - yi), (xf - xi));

		System.out.printf("d = %f\n", dist);
	}
	
	//Part 6
	public void reverseKinematics(double x, double y)
	{
		double D = ((x*x) + (y*y) - (L1*L1) - (L2*L2)) / (2 *L1 * L2);
		double theta2_a = Math.atan2((Math.sqrt(1 - (D*D))),D);
		double theta2_b = Math.atan2(-(Math.sqrt(1 - (D*D))),D);
		
		double theta1_a = Math.atan2(y, x) - Math.atan2((L2 * Math.sin(theta2_a)) , (L1 + (L2 * Math.cos(theta2_a))));
		double theta1_b = Math.atan2(y, x) - Math.atan2((L2 * Math.sin(theta2_b)) , (L1 + (L2 * Math.cos(theta2_b))));
		
		System.out.printf("Solution 1:\ntheta1 = %5f\ntheta2=%5f\n", theta1_a / CONVERT, theta2_a / CONVERT);
		System.out.printf("Solution 2:\ntheta1 = %5f\ntheta2=%5f\n", theta1_b / CONVERT, theta2_b / CONVERT);
		Button.waitForAnyPress();
		
		if(theta1_b>theta1_a){
			turnDegrees(theta1_b/CONVERT,theta2_b/CONVERT);
		}else{
			turnDegrees(theta1_a/CONVERT,theta2_a/CONVERT);
		}
	}
	
	
	//Part 7
	public void getMidPoints(){
		theta1.resetTachoCount();
		theta2.resetTachoCount();

		System.out.println("-"+theta1.getTachoCount());
		System.out.println("-"+theta2.getTachoCount());
		System.out.printf("Record first position...\n");
		Button.waitForAnyPress();

		int i_pos1 = -theta1.getTachoCount();
		int i_pos2 = -theta2.getTachoCount();

		System.out.printf("Record second position...\n");
		Button.waitForAnyPress();

		int f_pos1 = -theta1.getTachoCount();
		int f_pos2 = -theta2.getTachoCount();

		double[] initx_y = getPosition(i_pos1, i_pos2); 
		double[] endx_y = getPosition(f_pos1, f_pos2);
		
		double midX= (endx_y[0]+initx_y[0])/2;
		double midY= (endx_y[1]+initx_y[1])/2;
		
		double[] thetas = getAngles(midX,midY);
		
		
		boolean fromZero=false;
		if(fromZero){
			System.out.println("Move to Zero...");
			Button.waitForAnyPress();
			turnDegrees(thetas[0],thetas[1]); //Starting at 0,0.
		}else{
			//Horrible fix....
			theta1.close(); theta2.close();
			theta1 = new EV3MediumRegulatedMotor(MotorPort.A);
			theta2 = new EV3MediumRegulatedMotor(MotorPort.D);
			theta1.resetTachoCount();
			theta2.resetTachoCount();
			double ang1 = thetas[0]-f_pos1;
			double ang2 = thetas[1]-f_pos2;
			turnDegrees(ang1,ang2); //Starting at end position.
		}
		
		System.out.println("X="+midX);
		System.out.println("Y="+midY);
		
	}
	
	public double[] getPosition(double angle1, double angle2) {
		angle1 *= CONVERT;
		angle2 *= CONVERT;
		double x = (L1 * Math.cos(angle1)) + (L2 * Math.cos(angle1 + angle2));
		double y = (L1 * Math.sin(angle1)) + (L2 * Math.sin(angle1 + angle2));
		return new double[]{x,y};
	}
	
	public double[] getAngles(double x, double y)
	{
		double D = ((x*x) + (y*y) - (L1*L1) - (L2*L2)) / (2 *L1 * L2);
		double theta2_a = Math.atan2(Math.sqrt(1 - (D*D)),D);
		double theta2_b = Math.atan2(-Math.sqrt(1 - (D*D)),D);
		
		double theta1_a = Math.atan2(y,x) - Math.atan2((L2 * Math.sin(theta2_a)) , (L1 + (L2 * Math.cos(theta2_a))));
		double theta1_b = Math.atan2(y,x) - Math.atan2((L2 * Math.sin(theta2_b)) , (L1 + (L2 * Math.cos(theta2_b))));
		
		if(theta1_b > theta1_a)
			return new double[]{theta1_b/CONVERT,theta2_b/CONVERT};
		
		return new double[]{theta1_a/CONVERT,theta2_a/CONVERT};
	}
	
	//Part 8a
	public void drawLine(double ix, double iy, double fx, double fy){
		
		System.out.println("start?");
		Button.waitForAnyPress();
		
		theta1.resetTachoCount();
		theta2.resetTachoCount();
		double[] ithetas = getAngles(ix,iy);
		double ang1=ithetas[0];
		double ang2=ithetas[1];
		moveSameTime(ang1,ang2);
		
		double points = 10.;
		double incx=(fx-ix)/points;
		double incy=(fy-iy)/points;
		
		for (double n=0;n<points;n++){
			double[] fthetas = getAngles(ix+incx*(n+1.),iy+incy*(n+1.));
			//moveSameTime(fthetas[0]-ang1,fthetas[1]-ang2);
			moveSameTime(fthetas[0],fthetas[1]);
			ang1 = fthetas[0];
			ang2 = fthetas[1];
		}
		
	}
	
	public void drawVector(double ix, double iy, double d, double ang){
		
		//Just calculate the final point from this data...
		double dx=d*Math.cos(ang*CONVERT);
		double dy=d*Math.sin(ang*CONVERT);
		double fx=ix+dx;
		double fy=iy+dy;
		drawLine(ix, iy, fx, fy);		
	}
	
	public void drawArc(double x,double y,double r, double rd, double p){
		double maxAngle=360/rd;

		double initAngle=-maxAngle/2;
		double endAngle=-initAngle;
		
		double maxStep = initAngle<endAngle ? p : -p;
		for (double angle = initAngle; Math.abs(angle)-1 < Math.abs(endAngle);angle+=maxStep){
			double dx=x+r*Math.cos(angle*CONVERT);
			double dy=y+r*Math.sin(angle*CONVERT);
			double[] ithetas = getAngles(dx,dy);
			moveSameTime(ithetas[0],ithetas[1]);
		}
	
	}
	
	public void moveSameTime(double angle1, double angle2) {
		
		double relation = 1;//angle1/angle2;
		if (relation > 4)
			theta1.setSpeed(100);
		else
			theta1.setSpeed((int)(25*relation));
		theta2.setSpeed(25);

		// Motors are upside-down
		theta1.rotateTo((int) -angle1, true);
		theta2.rotateTo((int) -angle2, false);

		try {
			while (theta1.isMoving() || theta2.isMoving()) {
				Thread.sleep(10);
			}
		} catch (InterruptedException e) {}

	}
	
	
}
