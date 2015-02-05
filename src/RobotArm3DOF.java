import lejos.hardware.motor.EV3MediumRegulatedMotor;
import Jama.Matrix;



public class RobotArm3DOF
{
	public static final double L1 = 13.6;
	public static final double L2 = 6.3;
	public static final double L3 = 14.7;
	public static final double Z0 = 2.3;
	
	public static final int NEWTON_ITERATIONS          = 100;
	public static final double NEWTON_STEP             = 0.001;
	public static final int INITIAL_GUESS_SUBDIVISIONS = 100;
	
	private EV3MediumRegulatedMotor theta0;
	private EV3MediumRegulatedMotor theta1;
	private EV3MediumRegulatedMotor theta2;
	
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
		z -= Z0;
		
		double[] angles = initialGuess(x, y, z);
		double[] pos    = fk(angles[0], angles[1], angles[2]);

		System.out.printf("a: %f b:%f c:%f\n", (float) angles[0], (float) angles[1], (float) angles[2]);
		System.out.printf("x: %f y:%f z:%f\n", (float) pos   [0], (float) pos   [1], (float) pos   [2]);
	}
		
	private double[] initialGuess(double x, double y, double z)
	{
		double[] a = new double[] {1, 1, 1};
		double[] W = fk(a[0], a[1], a[2]);
		
		double
			dx = (x - W[0]) / INITIAL_GUESS_SUBDIVISIONS,
			dy = (y - W[1]) / INITIAL_GUESS_SUBDIVISIONS,
			dz = (z - W[2]) / INITIAL_GUESS_SUBDIVISIONS;
		
		for (int i = 0; i < INITIAL_GUESS_SUBDIVISIONS; i++)
		{
			a = inverseNewton
				(
					dx * i + W[0], 
					dy * i + W[1], 
					dz * i + W[2], 
					a
				);
		}
		
		return a;
	}

	private double[] inverseNewton(double x, double y, double z, double[] a)
	{
		double[][] 
			dx = {{a[0]}, {a[1]}, {a[2]}},
			dy = {{x},    {y},    {z}   };
		
		Matrix
			mdx = new Matrix(dx),
			mdy = new Matrix(dy);
		
		for (int i = 0; i < NEWTON_ITERATIONS; i++)
		{
			double[] cor  = fk(mdx.get(0, 0), mdx.get(1, 0), mdx.get(2, 0));
			
			double[] cor1 = fk(mdx.get(0, 0) + NEWTON_STEP, mdx.get(1, 0),               mdx.get(2, 0));
			double[] cor2 = fk(mdx.get(0, 0),               mdx.get(1, 0) + NEWTON_STEP, mdx.get(2, 0));
			double[] cor3 = fk(mdx.get(0, 0),               mdx.get(1, 0),               mdx.get(2, 0) + NEWTON_STEP);
			
			double[][] f  = {{cor[0]}, {cor[1]}, {cor[2]}};
			
			double[][] df = 
				{
					{
						cor1[0] - cor[0], cor2[0] - cor[0], cor3[0] - cor[0]
					},
					{
						cor1[1] - cor[1], cor2[1] - cor[1], cor3[1] - cor[1]
					},
					{
						cor1[2] - cor[2], cor2[2] - cor[2], cor3[2] - cor[2]
					}
				};
			
			Matrix
				mf  = new Matrix( f),
				mdf = new Matrix(df);
			
			mdx.plusEquals
			(
				mdf.inverse().times
				(
					mdy.minus(mf).times(1/1000)
				)
			);

			System.out.printf("x: %f y: %f z: %f\n", mdx.get(0, 0), mdx.get(1, 0), mdx.get(2, 0));
		}
		
		return new double[]{mdx.get(0, 0), mdx.get(1, 0), mdx.get(2, 0)};
	}

	private double[] fk(double a1, double a2, double a3)
	{
		a1 = toRad(a1);
		a2 = toRad(a2);
		a3 = toRad(a3);
		
		double
			retX = (L1 * Math.cos(a1)) + (L2 * Math.cos(a1 + a2)) + (L3 * Math.cos(a1+a2)) * Math.cos(a3),
			retY = (L1 * Math.sin(a1)) + (L2 * Math.sin(a1 + a2)) + (L3 * Math.sin(a1+a2)) * Math.cos(a3),
			retZ = Z0 + L3 * Math.sin(a3);
		
		return new double[]{retX, retY, retZ};
	}
	
	private double toRad(double deg)
	{
		return deg * (Math.PI / 180);
	}
	
	private double toDeg(double rad)
	{
		return rad * (180 / Math.PI);
	}
}