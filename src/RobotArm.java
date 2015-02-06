
import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Matrix;

public class RobotArm 
{
	public static double L1 = 13.6; // cm
	public static double L2 = 17.3; // cm
	public static double L3 = 14.7; // cm
	public static double Zo = 8.5; // cm
	public static final  double CONVERT = Math.PI / 180.0;
	public final boolean waitButton = false;
	EV3MediumRegulatedMotor theta1;
	EV3MediumRegulatedMotor theta2;
	EV3MediumRegulatedMotor theta3;

	public RobotArm() {
		theta1 = new EV3MediumRegulatedMotor(MotorPort.A);
		theta2 = new EV3MediumRegulatedMotor(MotorPort.D);
		theta3 = new EV3MediumRegulatedMotor(MotorPort.C); // <--- Motor C
		theta1.resetTachoCount();
		theta2.resetTachoCount();
		theta3.resetTachoCount();
	}
	
	public void finish(){
		theta1.close();
		theta2.close();
		theta3.close();
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
		System.out.printf("y = %f\n", y);
	}

	//Part 5
	public void calculateDistance() {
		//theta1.resetTachoCount();
		//theta2.resetTachoCount();

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
	
	public void calculateAngle() {
		//theta1.resetTachoCount();
		//theta2.resetTachoCount();

		System.out.printf("0: %d\n", theta1.getTachoCount());
		System.out.printf("0: %d\n", theta2.getTachoCount());

		System.out.printf("Record first...\n");
		Button.waitForAnyPress();

		int p1_1 = theta1.getTachoCount();
		int p1_2 = theta2.getTachoCount();

		System.out.printf("i_angle1: %d\n", p1_1);
		System.out.printf("i_angle2: %d\n", p1_2);

		System.out.printf("Record second...\n");
		Button.waitForAnyPress();

		int p2_1 = theta1.getTachoCount();
		int p2_2 = theta2.getTachoCount();

		System.out.printf("f_angle1: %d\n", p2_1);
		System.out.printf("f_angle2: %d\n", p2_2);
		
		System.out.printf("Record second...\n");
		Button.waitForAnyPress();

		int p3_1 = theta1.getTachoCount();
		int p3_2 = theta2.getTachoCount();

		System.out.printf("f_angle1: %d\n", p3_1);
		System.out.printf("f_angle2: %d\n", p3_2);

		double p1x = (L1 * Math.cos(p1_1 * CONVERT))
				+ (L2 * Math.cos((p1_1 * CONVERT) + (p1_2 * CONVERT)));
		double p1y = (L1 * Math.sin(p1_1 * CONVERT))
				+ (L2 * Math.sin((p1_1 * CONVERT) + (p1_2 * CONVERT)));

		double p2x = (L1 * Math.cos(p2_1 * CONVERT))
				+ (L2 * Math.cos((p2_1 * CONVERT) + (p2_2 * CONVERT)));
		double p2y = (L1 * Math.sin(p2_1 * CONVERT))
				+ (L2 * Math.sin((p2_1 * CONVERT) + (p2_2 * CONVERT)));

		double p3x = (L1 * Math.cos(p3_1 * CONVERT))
				+ (L2 * Math.cos((p3_1 * CONVERT) + (p3_2 * CONVERT)));
		double p3y = (L1 * Math.sin(p3_1 * CONVERT))
				+ (L2 * Math.sin((p3_1 * CONVERT) + (p3_2 * CONVERT)));
		
		System.out.printf( "1 x: %2.f y%2.f\n", p1x, p1y );
		System.out.printf( "2 x: %2.f y%2.f\n", p2x, p2y );
		System.out.printf( "3 x: %2.f y%2.f\n", p3x, p3y );
		
		// Find the angle between A, B, C
		double 
			ABx = p1x - p2x,
			ABy = p1y - p2y,
			BCx = p3x - p2x,
			BCy = p3y - p2y;
		
		double 
			ABdotBC = (ABx * BCx) + (ABy * BCy),
			lenAB = Math.hypot(ABx, ABy),
			lenBC = Math.hypot(BCx, BCy);
			
		
		double angle = Math.acos(ABdotBC / (lenAB * lenBC)) / CONVERT;

		System.out.printf("angle = %f deg\n", angle);
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
		//theta1.resetTachoCount();
		//theta2.resetTachoCount();

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
		
		System.out.println("start line?");
		if(waitButton)
			Button.waitForAnyPress();
		
		
		double[] ithetas = getAngles(ix,iy);
		double ang1=ithetas[0];
		double ang2=ithetas[1];
		moveSameTime(ang1,ang2);
		
		double points = 10.;
		double incx=(fx-ix)/points;
		double incy=(fy-iy)/points;
		
		for (double n=0;n<points;n++){
			double[] fthetas = getAngles(ix+incx*(n+1.),iy+incy*(n+1.));
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
	
	public void drawArc(double x,double y,double r, double rd, double p, double up){
		System.out.println("start arc?");
		if(waitButton)
			Button.waitForAnyPress();
		double maxAngle=360/rd;

		double initAngle=-maxAngle/2;
		double endAngle=-initAngle;
		if(up<0){
			initAngle*=-1;
			endAngle*=-1;
		}
		
		
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
			theta1.setSpeed(200);
		else
			theta1.setSpeed((int)(30*relation));
		theta2.setSpeed(30);

		// Motors are upside-down
		theta1.rotateTo((int) -angle1, true);
		theta2.rotateTo((int) -angle2, false);

		try {
			while (theta1.isMoving() || theta2.isMoving()) {
				Thread.sleep(10);
			}
		} catch (InterruptedException e) {}

	}
	
	// so all angles are between -360 and +360
	public double fixAngle(double angle){
		if(angle<0){
			while(Math.abs(angle)>180){
				angle+=360;
			}
		}else{
			while(Math.abs(angle)>180){
				angle-=360;
			}
		}
		return angle;
	}
	

	public void move3D(double x, double y, double z){
		L2 = 6.3;
		
		System.out.println("start 3DOF?");
		if(waitButton)
			Button.waitForAnyPress();
		System.out.println("calculating...");
		double[] angles = initialGuess3(x,y,z);
		
		
		angles[0]=fixAngle(angles[0]);
		if(angles[0]<0)
			angles[0]+=360;//final fix because j1 cannot go to the right (physical limitation)
		angles[1]=fixAngle(angles[1]);
		angles[2]=fixAngle(angles[2]);
		System.out.println("j1:"+angles[0]);
		System.out.println("j2:"+angles[1]);
		System.out.println("j3:"+angles[2]);
		
		theta1.setSpeed(50);
		theta2.setSpeed(50);
		theta3.setSpeed(50);

		// Motors are upside-down
		theta1.rotate((int) -angles[0], true);
		theta2.rotate((int) -angles[1], true);
		theta3.rotate((int) -angles[2], false);

		while (theta1.isMoving() || theta2.isMoving() || theta3.isMoving()){
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
			}
		}
	}
	
	public double[] initialGuess3(double posX,double posY,double posZ){
		double a1=1;
		double a2=1;
		double a3=1;
		
		double[] posXYZ = forwarKin3(a1,a2,a3);
		double N=100.;
		double difX=(posX-posXYZ[0])/N;
		double difY=(posY-posXYZ[1])/N;
		double difZ=(posZ-posXYZ[2])/N;
		
		for (int n=1;n<N+1;n++){
			double[] dA=inverseNewton3(difX*n+posXYZ[0],difY*n+posXYZ[1],difZ*n+posXYZ[2],a1,a2,a3);
			a1=dA[0]; a2=dA[1]; a3=dA[2];
		}
		
		return new double[]{a1,a2,a3};
	}
	
	public double[] inverseNewton3(double posX,double posY,double posZ, double a1, double a2, double a3){
		double[][] dx = new double[][]{{a1},{a2},{a3}};
		double[][] y = new double[][]{{posX},{posY},{posZ}};
		Matrix mdx = new Matrix(dx);
		Matrix my = new Matrix(y);
		for (int n = 0; n<100;n++){
			a1=mdx.get(0, 0);;
			a2=mdx.get(1, 0);;
			a3=mdx.get(2, 0);;
			double[] cor = forwarKin3(a1,a2,a3);
			double[] cor_1 = forwarKin3(a1+0.001,a2,a3);
			double[] cor_2 = forwarKin3(a1,a2+0.001,a3);
			double[] cor_3 = forwarKin3(a1,a2,a3+0.001);
			
			double[][] f= new double[][]{{cor[0]},{cor[1]},{cor[2]}};
			double[][] df = new double[][]
					{
					{cor_1[0]-cor[0],cor_2[0]-cor[0],cor_3[0]-cor[0]},
					{cor_1[1]-cor[1],cor_2[1]-cor[1],cor_3[1]-cor[1]},
					{cor_1[2]-cor[2],cor_2[2]-cor[2],cor_3[2]-cor[2]}
					};
			
			Matrix mdf = new Matrix(df);
			Matrix mf = new Matrix(f);
			Matrix sub = mdx.plus(mdf.inverse().times(my.minus(mf)));
			mdx.plusEquals(sub.times(0.001));
		}
		
		a1 = mdx.get(0, 0);
		a2 = mdx.get(1, 0);
		a3 = mdx.get(2, 0);
		
		return new double[]{a1,a2,a3};
	}
	
	public double[] forwarKin3(double a1, double a2, double a3){
		a1*=CONVERT;
		a2*=CONVERT;
		a3*=CONVERT;
		double x = (L1 * Math.cos(a1)) + (L2 * Math.cos(a1 + a2)) + (L3*Math.cos(a1+a2))*Math.cos(a3);
		double y = (L1 * Math.sin(a1)) + (L2 * Math.sin(a1 + a2)) + (L3*Math.sin(a1+a2))*Math.cos(a3);
		double z = Zo + L3*Math.sin(a3);
		
		return new double[]{x,y,z};
	}
	
}
