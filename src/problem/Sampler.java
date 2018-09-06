package problem;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Sampler {

    private RobotConfig robo;
    private List<MovingBox> movingBoxes;
    private List<StaticObstacle> staticObstacles;
    private List<MovingObstacle> movingObstacles;

    private double minStepSize = 0.001;

    private MovingBox focusBox;

    public Sampler(RobotConfig robo,
                   List<MovingBox> movingBoxes,
                   List<StaticObstacle> staticObstacles,
                   List<MovingObstacle> movingObstacles){

        this.robo = robo;
        this.movingBoxes = movingBoxes;
        this.staticObstacles = staticObstacles;
        this.movingObstacles = movingObstacles;
    }

    /**
     * temp visualisation method
     *
     * @param origin
     * @param mb
     * @param cur_pos
     * @return
     */
    private State createNewState(State origin, MovingBox mb, MovingBox cur_pos){
        List<MovingBox> newMovingBoxes = new ArrayList<>();

        for(MovingBox box : origin.getMovingBoxes()){
            if(box.equals(mb)){
                MovingBox temp = new MovingBox(
                        new Point2D.Double(cur_pos.getPos().getX(), cur_pos.getPos().getY()), box.getWidth());
                newMovingBoxes.add(temp);
                box.addToNodeList(temp);
            }else{
                newMovingBoxes.add(box);
            }
        }
        return new State(origin.getRobo(), newMovingBoxes, origin.getMovingObstacles());
    }

    private State sampleNewState(State origin, MovingBox mb){
        List<MovingBox> newMovingBoxes = new ArrayList<>();

        for(MovingBox box : origin.getMovingBoxes()){
            if(box.equals(mb)){
                MovingBox temp = new MovingBox(
                        new Point2D.Double(Math.random(), Math.random()), box.getWidth());
                newMovingBoxes.add(temp);
                box.addToNodeList(temp);
            }else{
                newMovingBoxes.add(box);
            }
        }
        State temp = new State(origin.getRobo(), newMovingBoxes, origin.getMovingObstacles());

        if(temp.isValid(staticObstacles)){
            return temp;
        }
        return null;
    }

    /**
     * Write solution file
     *
     * @param solution
     */
    public void printOutput(List<State> solution) {
        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter("solution1.txt"));

            writer.write(Integer.toString(solution.size()));
            writer.newLine();
            for (State step : solution) {
                writer.write(step.printState());
                writer.newLine();
            }
            writer.close();
        } catch (IOException e) {
            System.out.println("Error writing solution file");
            e.printStackTrace();
        }

    }

    public List<State> stepObjectiveSampling(){

        State nextState;
        List<MovingBox> update = new ArrayList<>();
        update.addAll(movingBoxes);

        boolean origin = true;
        List<State> orderedStates = new ArrayList<>();

        for(MovingBox mb : movingBoxes){
            if(origin){
                for(int i = 0; i < 1000; i++){
                    nextState = sampleNewState(new State(robo, movingBoxes, movingObstacles), mb);

                    if(nextState != null){
                        orderedStates.add(nextState);
                    }
                }

                origin = false;
            }else{
                for(int i = 0; i < 1000; i++){
                    nextState = sampleNewState(new State(robo, update, movingObstacles), mb);

                    if(nextState != null){
                        orderedStates.add(nextState);
                    }
                }
            }

            update.set(update.indexOf(mb), new MovingBox(
                    new Point2D.Double(mb.getEndPos().getX(), mb.getEndPos().getY()), mb.getWidth()));
        }

//        printOutput(orderedStates);

        int count = 0;
        focusBox = movingBoxes.get(0);

        List<State> path = new ArrayList<>();
        State step = createNewState(new State(robo, movingBoxes, movingObstacles), focusBox, focusBox);
        path.add(step);

        MovingBox og = movingBoxes.get(0);
//        System.out.println(og.getPos().getX() + ", " + og.getPos().getY());

        while(count < 100){ //hard limit placed
            MovingBox next = continueMovingBoxPath(og);

            if(next != null) {
//                System.out.println(next.getPos().getX() + ", " + next.getPos().getY());

                for(MovingBox sStep : buildStep(og, next)){
                    step = createNewState(new State(robo, movingBoxes, movingObstacles), focusBox, sStep);
                    path.add(step);
                }

                og = next;
            }else{
                System.out.println("next == null");
            }

            count++;
        }

        printOutput(path);
        return orderedStates;
    }

    public MovingBox continueMovingBoxPath(MovingBox mb){
        MovingBox goal = null;

//        System.out.println(mb.getPos().getX() + ", " + mb.getPos().getY());

        for (MovingBox node : getNeighbourNodes(mb)) {
//            System.out.println("-------- " + node.getPos().getX() + ", " + node.getPos().getY());
            // weigh each option (currently using distance and adding by the diagonal)
            if (goal == null) {   //empty goal
                goal = node;
            } else if (node.getDistanceToGoal() < goal.getDistanceToGoal()) {
                goal = node;
            }
        }

        return goal;
    }

    public Set<MovingBox> getNeighbourNodes(MovingBox mb){

        Set<MovingBox> neighbourNodes = new HashSet<>();

        for(MovingBox node : focusBox.getNodeList()){

            double dx = Math.abs(node.getPos().getX() - mb.getPos().getX());
            double dy = Math.abs(node.getPos().getY() - mb.getPos().getY());

            if(dx < 0.1 && dy < 0.1){

//                System.out.println(dx + ", " + dy);

                double ddx = Math.abs(node.getPos().getX() - focusBox.getEndPos().getX());
                double ddy = Math.abs(node.getPos().getY() - focusBox.getEndPos().getY());

                node.setDistanceToGoal(Math.sqrt(Math.pow(ddx, 2) + Math.pow(ddy, 2)));
                neighbourNodes.add(node);
            }
        }

        return neighbourNodes;
    }

    public List<MovingBox> buildStep(MovingBox origin, MovingBox end){

        // check path between origin and end (Rec around the origin and end points)
        Point2D mapOrigin = new Point2D.Double(min(origin.getRect().getMinX(), end.getRect().getMinX()),
                min(origin.getRect().getMinY(), end.getRect().getMinY()));

        double width = Math.abs(mapOrigin.getX() - max(origin.getRect().getMinX(), end.getRect().getMaxX()));
        double height = Math.abs(mapOrigin.getY() - max(origin.getRect().getMinY(), end.getRect().getMaxY()));

        Rectangle2D pathSpace = new Rectangle2D.Double(
                mapOrigin.getX(), mapOrigin.getY(), width, height);



        // build path
        List<MovingBox> path = new ArrayList<>();
        path.add(origin);

        double goalX = getReducedDouble(end.getPos().getX(), 3);
        double goalY = getReducedDouble(end.getPos().getY(), 3);

        System.out.println(goalX + ", " + goalY);


        double curX = -1, curY = -1;

        while(curX != goalX || curY != goalY){
            MovingBox last = path.get((path.size() - 1));

            double lastX = getReducedDouble(last.getPos().getX(), 3);
            double lastY = getReducedDouble(last.getPos().getY(), 3);

            // correct X position
            if(last.getPos().getX() < goalX){ //move right
                curX = lastX + minStepSize;
                curY = lastY;
            }
            else if(last.getPos().getX() > goalX){ //move left
                curX = lastX - minStepSize;
                curY = lastY;
            }

            // correct Y position
            else if(last.getPos().getY() < goalY){ //move up
                curX = lastX;
                curY = lastY + minStepSize;
            }
            else if(last.getPos().getY() > goalY){ //move down
                curX = lastX;
                curY = lastY - minStepSize;

            }

            // need check to see if path intersects with moving object
            // if the path intersects with the moving object
            // move intersecting object ...new c-space from this point???
            // ideally want to move it out of the path
            // given the end and origin move obstacle away from end??? ...that would be origin though???

            MovingBox projMB = new MovingBox(
                    new Point2D.Double(curX, curY), last.getWidth());

            // can check projected, if intersects with obstacle

            path.add(projMB);

        }

        return path;
    }

    private double getReducedDouble(double value, int to){
        return BigDecimal.valueOf(value)
                .setScale(to, RoundingMode.HALF_UP)
                .doubleValue();
    }

    private double min(double v1, double v2){
        if(v1 < v2) return v1;

        return v2;
    }

    private double max(double v1, double v2){
        if(v1 > v2) return v1;

        return v2;
    }
}
