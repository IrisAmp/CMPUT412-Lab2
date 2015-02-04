
public class Part9 
{
	public static void main(String[] args)
	{
		RobotArm arm = new RobotArm();
		double xCorr=-5;
		double yCorr=1;
		arm.drawLine(30+xCorr,0+yCorr , 30+xCorr,5+yCorr);// segment 1
		arm.drawLine(30+xCorr,5+yCorr , 27.8+xCorr,6.5+yCorr);// segment 2
		arm.drawLine(27.8+xCorr,6.5+yCorr , 25.9+xCorr,6.5+yCorr);// segment 3
		
		arm.drawArc(24.5+xCorr, 9.4, 3.2,   2.8, 10,+1); //segment 4
		
		arm.drawLine(25.9+xCorr,12.4+yCorr , 23.8+xCorr,12.4+yCorr);// segment 5
		arm.drawLine(23.8+xCorr,12.4+yCorr  ,  23.8+xCorr,8.5+yCorr);// segment 6
		arm.drawLine(23.8+xCorr,8.5+yCorr   ,  22+xCorr,8.5+yCorr);// segment 7
		arm.drawLine(22+xCorr,8.5+yCorr   ,  22+xCorr,12.4+yCorr);// segment 8
		arm.drawLine(22+xCorr,12.4+yCorr   ,  20.1+xCorr,12.4+yCorr);// segment 9
		arm.drawLine(20.1+xCorr,12.4+yCorr   ,  20.1+xCorr,8.5+yCorr);// segment 10
		arm.drawLine(20.1+xCorr,8.5+yCorr   ,  18.3+xCorr,8.5+yCorr);// segment 11
		arm.drawLine(18.3+xCorr,8.5+yCorr   ,  18.3+xCorr,12.4+yCorr);// segment 12
		arm.drawLine(18.3+xCorr,12.4+yCorr   ,  15.8+xCorr,12.4+yCorr);// segment 13
		
		arm.drawArc(13+xCorr, 10.9+yCorr, 3.2,   6.2, 10,-1); //segment 14
		
		arm.drawLine(14.9+xCorr,8.3+yCorr   ,  12.7+xCorr,6+yCorr);// segment 15
		arm.drawLine(12.7+xCorr,6+yCorr   ,  9.5+xCorr,6+yCorr);// segment 16
		arm.drawLine(9.5+xCorr,6+yCorr   ,  9.5,0+yCorr);// segment 17
		arm.finish();
	}
}
