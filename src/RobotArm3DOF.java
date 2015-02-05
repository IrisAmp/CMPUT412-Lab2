import lejos.hardware.motor.EV3MediumRegulatedMotor;
import Jama.Matrix;


public class RobotArm3DOF
{
	public static void main(String [] args)
	{
		RobotArm3DOF x = new RobotArm3DOF();
	}
	
	// DH parameters
	public static final double alpha0 = 0;
	public static final double a0     = 0;
	public static final double d0     = 0;
	public static final double alpha1 = 0;
	public static final double a1     = 13.8; // cm
	public static final double d1     = 0;
	public static final double alpha2 = 90; // degrees
	public static final double a2     = 6.3; // cm
	public static final double d2     = 2.3; // cm
	public static final double alpha3 = 0;
	public static final double a3     = 14.7; // cm
	public static final double d3     = 0;

	// Actuators
	private EV3MediumRegulatedMotor theta0;
	private EV3MediumRegulatedMotor theta1;
	private EV3MediumRegulatedMotor theta2;
	
	// End effector
	private Matrix ee = new Matrix(new double[][] {{0}, {0}, {0}, {1}});
	
	public RobotArm3DOF()
	{
		/*
		theta0 = new EV3MediumRegulatedMotor(MotorPort.A);
		theta1 = new EV3MediumRegulatedMotor(MotorPort.B);
		theta2 = new EV3MediumRegulatedMotor(MotorPort.C);
		//*/

		
	}
	
	public void ik(double x, double y, double z)
	{
		
	}
	
	private double toR(double deg)
	{
		return deg * (Math.PI / 180);
	}
	
	private double toDeg(double r)
	{
		return r * (180 / Math.PI);
	}
}