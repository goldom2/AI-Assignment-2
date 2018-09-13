package problem;

import problem.Box;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;


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

	private Set<RobotConfig> dockPos = new HashSet<>();
	private Set<Box> nodeList = new HashSet<>();

	// Probably best to avoid using this constructor
	public MovingBox(Point2D pos, double width) {
		super(pos, width);
	}

    public MovingBox(Point2D pos, Point2D endPos, double width) {
        super(pos, width);
		this.endPos = endPos;
	}

	/**
	 * Calculates the minimum distance to the goal
     *
	 * @return
	 */
	public double distanceToGoal() {
		double result = 0;
		result += Math.abs(getPos().getX() - getEndPos().getX());
		result += Math.abs(getPos().getY() - getEndPos().getY());
		return result;
	}

	public void addToNodeList(MovingBox box){
        nodeList.add(box);
    }

    public Set<Box> getNodeList(){
	    return nodeList;
    }

	public Point2D getEndPos(){
    	return endPos;
	}
    public void setEndPos(Point2D endPos){
        this.endPos = endPos;
    }

	public Set<RobotConfig> getDockPos() { return dockPos; }

	public void setDockPos(Set<RobotConfig> dockPos) { this.dockPos = dockPos; }

    @Override
    public boolean equals(Object o) {
	    if (o instanceof MovingBox) {
	        MovingBox other = (MovingBox) o;
            return super.equals(other)
                    && (this.getEndPos() == null
                    ? other.getEndPos() == null : this.getEndPos().equals(other.getEndPos()));
        }
        return false;
    }
}
