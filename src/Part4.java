import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;


public class Part4 
{
	public static void main(String[] args)
	{
		RobotArm arm = new RobotArm();
		
		arm.move(45, 45);
	}
	
	public static class RobotArm
	{
		public static final double L1 = 13.5; // cm
		public static final double L2 = 14.2; // cm
		public static final double CONVERT = Math.PI / 180.0;
		
		EV3MediumRegulatedMotor theta1;
		EV3MediumRegulatedMotor theta2;
		
		public RobotArm()
		{
			theta1 = new EV3MediumRegulatedMotor(MotorPort.A);
			theta2 = new EV3MediumRegulatedMotor(MotorPort.D);
		}
		
		public void move(double angle1, double angle2)
		{
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
	}
}
