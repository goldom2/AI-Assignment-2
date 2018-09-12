package problem;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.List;

public class State {

    private RobotConfig robot;
    private List<MovingBox> movingBoxes;
    private List<MovingObstacle> movingObstacles;

    private double roboWidth;
    private boolean roboAttach;

    public State(RobotConfig robot,
             List<MovingBox> movingBoxes,
             List<MovingObstacle> movingObstacles){

        this.robot = robot;
        this.roboWidth = movingBoxes.get(0).getWidth();
        this.roboAttach =  false;

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

            if(robot.getPos().equals(movingBoxes.get(i).getPos())){
                roboAttach = true;
            }else{
                roboAttach = false;
            }

            if (!space.contains(movingBoxes.get(i).getRect())) {
                return false;
            }

            if(getRoboConfigAsRec().intersects(movingBoxes.get(i).getRect()) &&
                roboAttach == false){
                return false;
            }

            for (int j = 0; j < obstacles.size(); j++) {
                if (movingBoxes.get(i).getRect().intersects(obstacles.get(j).getRect())) {
                    return false;
                }
            }
//            for (int j = 0; j < movingObstacles.size(); j++) {
//                if (movingBoxes.get(i).getRect().intersects(movingObstacles.get(j).getRect())) {
//                    return false;
//                }
//            }
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
            if(getRoboConfigAsRec().intersects(movingObstacles.get(i).getRect())){
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

        for (int i = 0; i < obstacles.size(); i++){
            if(getRoboConfigAsRec().intersects(obstacles.get(i).getRect())){
                return false;
            }
        }

        return true;
    }

    public List<MovingBox> getMovingBoxes(){
        return movingBoxes;
    }

    public List<MovingObstacle> getMovingObstacles(){
        return movingObstacles;
    }

    public RobotConfig getRobo(){
        return robot;
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

    @Override
    public int hashCode() {
        // TODO
        return super.hashCode();
    }

    /**
     * return
     *
     * @return
     */
    public String printState() {
        String output = robot.getPos().getX() + " " + robot.getPos().getY() + " " + robot.getOrientation();
        for (MovingBox box : movingBoxes) {
            output += " " + (box.getRect().getMinX() + box.getWidth() / 2);
            output += " " + (box.getRect().getMinY() + box.getWidth() / 2);
        }
        for (MovingObstacle obstacle : movingObstacles) {
            output += " " + (obstacle.getRect().getMinX() + obstacle.getWidth() / 2);
            output += " " + (obstacle.getRect().getMinY() + obstacle.getWidth() / 2);
        }
        return output;
    }

    private Line2D getRoboConfigAsRec(){
        Point2D p1 = new Point2D.Double(robot.getX1(roboWidth), robot.getY1(roboWidth));
        Point2D p2 = new Point2D.Double(robot.getX2(roboWidth), robot.getY2(roboWidth));

        return new Line2D.Double(p1, p2);    //why RoboConfig doesn't extend Line2D is a joke...
    }
}
