
public class Part8 
{
	public static void main(String[] args)
	{
		RobotArm arm = new RobotArm();	
		//arm.drawLine(20,5  , 14,22); //a)
		arm.drawLine(20,5  , 10,5); //a)
		//arm.drawVector(20.,5.,10.,180.); //b) x,y,d,ang
		arm.finish();
	}
}
