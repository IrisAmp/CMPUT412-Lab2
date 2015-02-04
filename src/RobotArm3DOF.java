import java.util.ArrayList;

import Jama.Matrix;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;


public class RobotArm3DOF
{
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
	private ArrayList<Double> q;
	
	// End effectors
	private Point ee0 = new Point(0, 0, 0);
	private Point ee1 = new Point(0, 0, 0);
	private Point ee2 = new Point(0, 0, 0);
	private Point ee3 = new Point(0, 0, 0);
	private ArrayList<Point> x;
	
	public RobotArm3DOF()
	{
		/*
		theta0 = new EV3MediumRegulatedMotor(MotorPort.A);
		theta1 = new EV3MediumRegulatedMotor(MotorPort.B);
		theta2 = new EV3MediumRegulatedMotor(MotorPort.C);
		//*/
		
		// Initialize the position of the base (constant)
		ee0.x = 0;
		ee0.y = 0;
		ee0.z = d0;
		
		
	}
	
	public void ik(double x, double y, double z)
	{
		// Reset initial position
		resetTachos();
		
		// Initialize the positions of the end effectors
		measureEffectorPos();
		
		
	}

	private void measureEffectorPos()
	{
		// position of ee0 is constant.
		
	}

	private void resetTachos() 
	{
		theta0.resetTachoCount();
		theta1.resetTachoCount();
		theta2.resetTachoCount();
	}
	
	private Matrix getTransformationMatrix(double angle0, double angle1, double angle2)
	{
		double A = Math.cos(angle0);
		double B = Math.sin(angle0);
		double C = Math.cos(angle1);
		double D = Math.sin(angle1);
		double E = Math.cos(angle2);
		double F = Math.sin(angle2);

		// http://www.wolframalpha.com/input/?i=matrix+multiplication&a=*C.matrix+multiplication-_*Calculator.dflt-&f2=%7B%7BA*C-B*D%2C+-B*C-A*D%2C+0%2C+13.8*A%7D%2C+%7BB*C+%2B+A*D%2C+A*C+-+B*D%2C+0%2C+13.8*A%7D%2C+%7B0%2C+0%2C+1%2C+0%7D%2C+%7B0%2C+0%2C+0%2C+1%7D%7D&f=MatricesOperations.theMatrix1%5Cu005f%7B%7BA*C-B*D%2C+-B*C-A*D%2C+0%2C+13.8*A%7D%2C+%7BB*C+%2B+A*D%2C+A*C+-+B*D%2C+0%2C+13.8*A%7D%2C+%7B0%2C+0%2C+1%2C+0%7D%2C+%7B0%2C+0%2C+0%2C+1%7D%7D&f3=%7B%7BX%2C+-F%2C+0%2C+14.7*X+%2B+6.3%7D%2C+%7B0%2C+0%2C+1%2C+-2.3%7D%2C+%7BF%2C+X%2C+0%2C+14.7*F%7D%2C+%7B0%2C+0%2C+0%2C+1%7D%7D&f=MatricesOperations.theMatrix2_%7B%7BX%2C+-F%2C+0%2C+14.7*X+%2B+6.3%7D%2C+%7B0%2C+0%2C+1%2C+-2.3%7D%2C+%7BF%2C+X%2C+0%2C+14.7*F%7D%2C+%7B0%2C+0%2C+0%2C+1%7D%7D&a=*FVarOpt.1-_**-.***MatricesOperations.theMatrix3---.*--
		double[][] t = new double[][]
		{
			{
				((A * C) - (B * D)) * E,
				-(( A * C) - (B * D)) * F,
				-(B * C) - (A * D),
				(a1 * A) - (d2 * (-(B * C) - (A * D))) + ((A * C) - (B * D)) * (a3 * E + a2)
			},
			{
				((B * C) + (A * D)) * E,
				-((B * C) + (A * D)) * F,
				(A * C) - (B * D),
				(a1 * A) - (d2 * ((A * C) - (B * D)) + (((B * C) + ( A * D)) * ((a3 * E) + a2)))
			},
			{
				F,
				E,
				0,
				a3 * F
			},
			{
				0,
				0,
				0,
				1
			},
		};
		
		return new Matrix(t);
	}
	
	private Matrix getPointCol(Point p)
	{
		double [][] x = 
			{
				{
					p.x
				},
				{
					p.y
				},
				{
					p.z
				},
				{
					1
				},
			};
		
		return new Matrix(x);
	}
	
	private static class Point
	{
		public double x;
		public double y;
		public double z;
		
		public Point(double x, double y, double z)
		{
			this.x = x;
			this.y = y;
			this.z = z;
		}
	}
}