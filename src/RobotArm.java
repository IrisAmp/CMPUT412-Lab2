import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;

public class RobotArm 
{
	public static final double L1 = 13.5; // cm
	public static final double L2 = 14.2; // cm
	public static final double CONVERT = Math.PI / 180.0;

	EV3MediumRegulatedMotor theta1;
	EV3MediumRegulatedMotor theta2;

	public RobotArm() {
		theta1 = new EV3MediumRegulatedMotor(MotorPort.A);
		theta2 = new EV3MediumRegulatedMotor(MotorPort.D);
	}

	public void move(double angle1, double angle2) {
		theta1.setSpeed(25);
		theta2.setSpeed(25);

		// Motors are upside-down
		theta1.rotateTo((int) -angle1, true);
		theta2.rotateTo((int) -angle2, false);

		theta1.stop();
		theta2.stop();

		angle1 *= CONVERT;
		angle2 *= CONVERT;

		double x = (L1 * Math.cos(angle1)) + (L2 * Math.cos(angle1 + angle2));
		double y = (L1 * Math.sin(angle1)) + (L2 * Math.sin(angle1 + angle2));

		System.out.printf("x = %f\n", x);
		System.out.printf("y = %f\n", -y);

		Button.waitForAnyPress();
	}

	public void calc() {
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

		Button.waitForAnyPress();
	}
	
	public void reverseKinematics(double x, double y)
	{
		double D = ((x*x) + (y*y) - (L1*L1) - (L2*L2)) / (2 *L1 * L2);
		double theta2_a = Math.atan(Math.sqrt(1 - (D*D))/D);
		double theta2_b = Math.atan(-Math.sqrt(1 - (D*D))/D);
		
		double theta1_a = Math.atan2(y, x) - Math.atan((L2 * Math.sin(theta2_a)) / (L1 + (L2 * Math.cos(theta2_a))));
		double theta1_b = Math.atan2(y, x) - Math.atan((L2 * Math.sin(theta2_b)) / (L1 + (L2 * Math.cos(theta2_b))));
		
		System.out.printf("Solution 1:\ntheta1 = %5f\ntheta2=%5f\n", theta1_a / CONVERT, theta2_a / CONVERT);
		System.out.printf("Solution 2:\ntheta1 = %5f\ntheta2=%5f\n", theta1_b / CONVERT, theta2_b / CONVERT);
		
		Button.waitForAnyPress();
	}
}
