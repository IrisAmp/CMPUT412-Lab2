
public class Part12 
{
	public static void main(String [] args)
	{
		RobotArm3DOF robot = new RobotArm3DOF();
		double[] angles = robot.ik(-10, 25, 10);
		robot.turnDegrees(angles[0], angles[1], angles[2]);
		robot.finish();
	}
}
