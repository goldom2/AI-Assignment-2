package problem;

import problem.Box;

import java.awt.geom.Point2D;


/**
 * This class represents one of the moving boxes in Assignment 1.
 * 
 * @author Sergiy Dudnikov
 */
public class MovingBox extends Box {

	/**
	 * Constructs a Moving box at a position width a side width
	 * 
	 * @param pos
	 *            the position of the box
	 * @param width
	 *            the width (and height) of the box
	 */

	private Point2D endPos;

	public MovingBox(Point2D pos, double width) {
		super(pos, width);
	}

    public MovingBox(Point2D pos, Point2D endPos, double width) {
        super(pos, width);
		this.endPos = endPos;
	}

	/**
	 * Calculates the minimum distance to the goal
	 * @return
	 */
	public double distanceToGoal() {
		double result = 0;
		result += Math.abs(getPos().getX() - getEndPos().getX());
		result += Math.abs(getPos().getY() - getEndPos().getY());
		return result;
	}

	public Point2D getEndPos(){
    	return endPos;
	}
}
