package ca.mcgill.ecse211.wallfollowing;

import java.awt.Graphics2D;

public class DrawMap {

	public void drawCenteredCircle(Graphics2D g, int x, int y, int r) {
		  x = x-(r/2);
		  y = y-(r/2);
		  g.fillOval(x,y,r,r);
		}
}
