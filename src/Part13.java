import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

/**
 * 
 * Part 13, image drawer.
 * Works by finding pixels in the image that are darker than a certain 
 * threshold and draws a dot at that point. Unfortunately, the EV3 arms don't
 * really have the accuracy to draw a good image.
 *
 */
public class Part13
{
	public static void main(String[] args)
	{
		RobotArm arm = new RobotArm();
		BufferedImage img = null;
		
		try 
		{
			img = ImageIO.read(new File("part13input.jpg"));
		} 
		catch (IOException exc) 
		{
			System.out.printf("Error reading the file: " + exc.getMessage());
			return;
		}
		
		for (int x = 0; x < img.getWidth(); x++) 
		{
			for (int y = 0; y < img.getHeight(); y++) 
			{
				Color pix = new Color(img.getRGB(x, y));
				
				if ( 
						// If the pixel is darker than gray
						pix.getRed()   < Color.GRAY.getRed()   &&
						pix.getGreen() < Color.GRAY.getGreen() &&
						pix.getBlue()  < Color.GRAY.getBlue()
					)
				{
					// Scale the image pixel position into the robot's workspace.
					// For a 100x100 image, puts the arm within [5, 15] cm
					// The EV3 doesn't really have the accuracy to perform 
					// this reliably, but it works in theory.
					double posx = ((double) x * 0.1) + 5; // cm
					double posy = ((double) y * 0.1) + 5; // cm
					
					// Lift the arm and move over the position to draw.
					arm.move3D(posx, posy, 5);
					// Move the arm onto the page.
					arm.move3D(posx, posy, 0);
				};
			}
		}
	}
}
