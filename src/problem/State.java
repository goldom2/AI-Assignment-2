package problem;

import java.awt.geom.Rectangle2D;
import java.util.List;

public class State {

    private RobotConfig robot;
    private List<MovingBox> movingBoxes;
    private List<MovingObstacle> movingObstacles;



    public State(RobotConfig robot,
             List<MovingBox> movingBoxes,
             List<MovingObstacle> movingObstacles){
        this.robot = robot;
        this.movingBoxes = movingBoxes;
        this.movingObstacles = movingObstacles;
    }

    /**
     * Takes the static obstacles from the problem spec and the size of the boxes
     * Returns if the configuration is valid, i.e. has no collisions and is within
     * the space
     *
     * @param obstacles
     * @return
     */
    public boolean isValid(List<StaticObstacle> obstacles){
        Rectangle2D space = new Rectangle2D.Double(0, 0, 1, 1);
        // Check all moving boxes are valid
        for (int i = 0; i < movingBoxes.size(); i++) {
            if (!space.contains(movingBoxes.get(i).getRect())) {
                return false;
            }
            for (int j = 0; j < obstacles.size(); j++) {
                if (movingBoxes.get(i).getRect().intersects(obstacles.get(j).getRect())) {
                    return false;
                }
            }
            for (int j = 0; j < movingObstacles.size(); j++) {
                if (movingBoxes.get(i).getRect().intersects(movingObstacles.get(j).getRect())) {
                    return false;
                }
            }
            for (int j = 0; j < movingBoxes.size(); j++) {
                if (movingBoxes.get(i).getRect().intersects(movingBoxes.get(j).getRect())
                        && i != j) {
                    return false;
                }

            }
        }

        // Check all moving obstacles are valid
        for (int i = 0; i < movingObstacles.size(); i++) {
            if (!space.contains(movingObstacles.get(i).getRect())) {
                return false;
            }
            for (int j = 0; j < obstacles.size(); j++) {
                if (movingObstacles.get(i).getRect().intersects(obstacles.get(j).getRect())) {
                    return false;
                }
            }
            for (int j = 0; j < movingObstacles.size(); j++) {
                if (movingObstacles.get(i).getRect().intersects(movingObstacles.get(j).getRect())
                        && i != j) {
                    return false;
                }
            }
        }

        return true;
    }

    /**
     * Calculates the minimum distance for each box to the goal state
     *
     * @return
     */
    public double distanceToGoalState() {
        double result = 0;
        for (MovingBox box : movingBoxes) {
            result += box.distanceToGoal();
        }
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof State) {
            State other = (State) obj;
            return other.robot.equals(this.robot)
                    && other.movingBoxes.equals(this.movingBoxes)
                    && other.movingObstacles.equals(this.movingObstacles);
        }
        return false;
    }
}
