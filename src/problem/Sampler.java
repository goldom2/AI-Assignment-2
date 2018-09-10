package problem;

import java.awt.*;
import java.awt.geom.Line2D;
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
    private double roboWidth;

    private Set<RobotConfig> posRoboConfig;

    public Sampler(RobotConfig robo,
                   List<MovingBox> movingBoxes,
                   List<StaticObstacle> staticObstacles,
                   List<MovingObstacle> movingObstacles){

        this.robo = robo;
        this.roboWidth = movingBoxes.get(0).getWidth();
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
    private State createNewState(State origin, MovingBox mb, MovingBox cur_pos, List<MovingObstacle> obsPos, RobotConfig newRobo){
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
        return new State(newRobo, newMovingBoxes, obsPos);
    }

    private Set<RobotConfig> sampleNewRobo(State origin, RobotConfig roboConfig){

        Set<RobotConfig> posRobo = new HashSet<>();
        int count = 0;

        while(count < 10000){
            RobotConfig newConfig = new RobotConfig(
                    new Point2D.Double(Math.random(), Math.random()), roboConfig.getOrientation());
            State temp = new State(newConfig, origin.getMovingBoxes(), origin.getMovingObstacles());

            if(temp.isValid(staticObstacles)){
                posRobo.add(newConfig);
            }

            count++;
        }

        return posRobo;
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

    public Point2D findDock(RobotConfig origin, MovingBox mb){
        Point2D center = new Point2D.Double(mb.getPos().getX() + mb.getWidth()/2, mb.getPos().getY() + mb.getWidth()/2);
        Point2D newPt = origin.getPos();

        double dist = 1;

        List<Point2D> dockPos = new ArrayList<>();
        dockPos.add(new Point2D.Double(center.getX() + mb.getWidth(), center.getY()));    //right hand side ...robo width 0.01
        dockPos.add(new Point2D.Double(center.getX() - mb.getWidth(), center.getY()));    //left hand side
        dockPos.add(new Point2D.Double(center.getX(), center.getY() + mb.getWidth()/2 + 0.001));    //upper hand side
        dockPos.add(new Point2D.Double(center.getX(), center.getY() - mb.getWidth()/2 - 0.001));    //under hand side

        for(Point2D point : dockPos){
            double proj = origin.getPos().distance(point);
            if(proj < dist){
                dist = proj;
                newPt = point;
            }
        }

        return newPt;
    }

    public List<State> rotateBot(RobotConfig robo, double target, State state){
        List<State> path = new ArrayList<>();

        double angle = robo.getOrientation()/(Math.PI/180);
        double maxIncrementAngle = (minStepSize/(Math.PI*(roboWidth/2)))*360;
        double margin = maxIncrementAngle*Math.floor(Math.abs(target - angle)/maxIncrementAngle);
        double offset = Math.abs(target - margin);

        if(angle < target){
            while(angle < (margin - offset)){
                angle += maxIncrementAngle;
                System.out.println(angle);
                path.add(new State(new RobotConfig(robo.getPos(), angle*(Math.PI/180)), state.getMovingBoxes(), state.getMovingObstacles()));
            }

            angle += offset;
            System.out.println(Math.floor(angle)*(Math.PI/180));
            path.add(new State(new RobotConfig(robo.getPos(), Math.floor(angle)*(Math.PI/180)), state.getMovingBoxes(), state.getMovingObstacles()));
        }
        else if(angle > target){
            while(angle > (margin + offset)){
                angle -= maxIncrementAngle;
                path.add(new State(new RobotConfig(robo.getPos(), angle*(Math.PI/180)), state.getMovingBoxes(), state.getMovingObstacles()));
            }

            angle -= offset;
            path.add(new State(new RobotConfig(robo.getPos(), Math.floor(angle)*(Math.PI/180)), state.getMovingBoxes(), state.getMovingObstacles()));
        }

        return path;
    }

    public List<State> orientRobot(MovingBox mb, State state){
        RobotConfig cur = state.getRobo();
        List<State> path = new ArrayList<>();
        path.add(state);

        Point2D center = new Point2D.Double(mb.getPos().getX() + mb.getWidth()/2, mb.getPos().getY() + mb.getWidth()/2);

        if(cur.getPos().getX() > center.getX() || cur.getPos().getX() < center.getX()){    //right/left
            path.addAll(rotateBot(cur, 90, state));
            State last = path.get(path.size() - 1);

            while(last.getRobo().getPos().getX() < center.getX() - (roboWidth/2)){
                Point2D close = new Point2D.Double(last.getRobo().getPos().getX() + minStepSize, last.getRobo().getPos().getY());
                RobotConfig push = new RobotConfig(close, last.getRobo().getOrientation());
                path.add(new State(push, last.getMovingBoxes(), last.getMovingObstacles()));

                last = path.get(path.size() - 1);
            }
            while(last.getRobo().getPos().getX() > center.getX() + (roboWidth/2)){
                Point2D close = new Point2D.Double(last.getRobo().getPos().getX() - minStepSize, last.getRobo().getPos().getY());
                RobotConfig push = new RobotConfig(close, last.getRobo().getOrientation());
                path.add(new State(push, last.getMovingBoxes(), last.getMovingObstacles()));

                last = path.get(path.size() - 1);
            }

        }else if(cur.getPos().getY() > center.getY() || cur.getPos().getY() < center.getY()){    //above/below
            path.addAll(rotateBot(cur, 0, state));

            State last = path.get(path.size() - 1);

            while(last.getRobo().getPos().getY() < center.getY() - (roboWidth/2)){
                Point2D close = new Point2D.Double(last.getRobo().getPos().getX(), last.getRobo().getPos().getY() + minStepSize);
                RobotConfig push = new RobotConfig(close, last.getRobo().getOrientation());
                path.add(new State(push, last.getMovingBoxes(), last.getMovingObstacles()));

                last = path.get(path.size() - 1);
            }
            while(last.getRobo().getPos().getY() > center.getY() + (roboWidth/2)){
                Point2D close = new Point2D.Double(last.getRobo().getPos().getX(), last.getRobo().getPos().getY() - minStepSize);
                RobotConfig push = new RobotConfig(close, last.getRobo().getOrientation());
                path.add(new State(push, last.getMovingBoxes(), last.getMovingObstacles()));

                last = path.get(path.size() - 1);
            }
        }


        return path;
    }


    public List<State> stepObjectiveSampling(){

        State nextState;
        List<MovingBox> update = new ArrayList<>();
        update.addAll(movingBoxes);

        boolean origin = true;
        State og = new State(robo, movingBoxes, movingObstacles);

        List<State> orderedStates = new ArrayList<>();
        List<MovingBox> observed = new ArrayList<>();

        List<State> path = new ArrayList<>();

        posRoboConfig = sampleNewRobo(og, robo);
//        for(RobotConfig rc : posRoboConfig){
//            path.add(new State(rc, movingBoxes, movingObstacles));
//        }

        //mapped moving Boxes at assumed states
        for(MovingBox mb : movingBoxes){

            //find the most accessible/closest position to "dock" the robot
            //need the start position for the robot... not necessarily origin
            Point2D robDockPos = findDock(robo, mb);
            mb.setDockPos(robDockPos);

//            System.out.println(robDockPos.getX() + ", " + robDockPos.getY());
            posRoboConfig.add(new RobotConfig(robDockPos, robo.getOrientation()));

            if(origin){
                for(int i = 0; i < 1000; i++){
                    nextState = sampleNewState(og, mb);

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

            MovingBox arrived = new MovingBox(
                    new Point2D.Double(mb.getEndPos().getX(), mb.getEndPos().getY()), mb.getWidth());
            update.set(update.indexOf(mb), arrived);
        }

        State step = createNewState(new State(robo, movingBoxes, movingObstacles), focusBox, focusBox, movingObstacles, robo);
        path.add(step);

        for(MovingBox mbog : movingBoxes){ //hard limit placed

            focusBox = mbog;
            MovingBox init = mbog;

            State intoNewBox = step;

            if(!mbog.getDockPos().equals(robo.getPos())){   //path to box
                path.addAll(pathBot(robo, intoNewBox, focusBox.getDockPos()));
            }

            path.addAll(orientRobot(mbog, path.get(path.size() - 1)));
            //orient robot based on position


            int count = 0;

            while(count < 1000){
                MovingBox next = continueMovingBoxPath(init, observed);

                if(next != null) {
                    System.out.println(next.getPos().getX() + ", " + next.getPos().getY());

                    // check path between origin and end (Rec around the origin and end points)
                    Point2D mapOrigin = new Point2D.Double(min(init.getRect().getMinX(), next.getRect().getMinX()),
                            min(init.getRect().getMinY(), next.getRect().getMinY()));

                    double width = Math.abs(mapOrigin.getX() - max(init.getRect().getMaxX(), next.getRect().getMaxX()));
                    double height = Math.abs(mapOrigin.getY() - max(init.getRect().getMaxY(), next.getRect().getMaxY()));

                    Rectangle2D pathSpace = new Rectangle2D.Double(
                            mapOrigin.getX(), mapOrigin.getY(), width, height);

                    List<MovingObstacle> newMoPos = moveMovingObstacles(pathSpace, step);

                    for(MovingBox sStep : buildStep(init, next)){

                        step = createNewState(intoNewBox, focusBox, sStep, newMoPos, robo);
                        path.add(step);
                    }

                    observed.add(next);
                    init = next;
                }else{
                    System.out.println("next == null");
                }

                count++;
            }
        }

        printOutput(path);
        return orderedStates;
    }

    public MovingBox continueMovingBoxPath(MovingBox mb, List<MovingBox> mbs){
        MovingBox goal = null;
        boolean correct = true;
//        System.out.println(mb.getPos().getX() + ", " + mb.getPos().getY());

        for (MovingBox node : getNeighbourNodes(mb)) {
            for(StaticObstacle so : staticObstacles){
                if(node.getRect().intersects(so.getRect())){
                    correct = false;
                }
            }
            if(correct && !mbs.contains(node)){
                // weigh each option (currently using distance and adding by the diagonal)
                if (goal == null) {   //empty goal
                    goal = node;
                } else if (node.getDistanceToGoal() < goal.getDistanceToGoal()) {
                    goal = node;
                }
            }

            correct = true;
        }

        return goal;
    }

    public Set<MovingBox> getNeighbourNodes(MovingBox mb){

        Set<MovingBox> neighbourNodes = new HashSet<>();

        for(MovingBox node : focusBox.getNodeList()){

            double dx = Math.abs(node.getPos().getX() - mb.getPos().getX());
            double dy = Math.abs(node.getPos().getY() - mb.getPos().getY());

            if(dx < 1 && dy < 1){

                double ddx = Math.abs(node.getPos().getX() - focusBox.getEndPos().getX());
                double ddy = Math.abs(node.getPos().getY() - focusBox.getEndPos().getY());

                node.setDistanceToGoal(Math.sqrt(Math.pow(ddx, 2) + Math.pow(ddy, 2)));
                neighbourNodes.add(node);
            }
        }

        return neighbourNodes;
    }

    public List<MovingObstacle> moveMovingObstacles(Rectangle2D pathSpace, State curState){
        List<MovingObstacle> newObstPos = new ArrayList<>();

        for(MovingObstacle mo : curState.getMovingObstacles()){

            newObstPos.add(mo);
            if(mo.getRect().intersects(pathSpace)){ // a moving obstacle intersects with path

                // BFS approach, find a lateral direction where the obstacle can be moved outside of the path
                List<Point2D> validPos = new ArrayList<>();
                validPos.add(new Point2D.Double((mo.getPos().getX() - pathSpace.getWidth()), mo.getPos().getY()));
                validPos.add(new Point2D.Double((mo.getPos().getX() + (pathSpace.getWidth() + mo.getWidth())), mo.getPos().getY()));
                validPos.add(new Point2D.Double(mo.getPos().getX(), (mo.getPos().getY() - pathSpace.getWidth())));
                validPos.add(new Point2D.Double(mo.getPos().getX(), (mo.getPos().getY() + (pathSpace.getHeight() + mo.getWidth()))));


                for(Point2D pos : validPos){
                    MovingObstacle newMO = new MovingObstacle(pos, mo.getWidth());
//                    System.out.println(newMO.getPos().getX() + ", " + newMO.getPos().getY());

                    if(validateNewMovingObjectPos(newMO, curState)){
                        newObstPos.set(curState.getMovingObstacles().indexOf(mo), newMO);
                        break;
                    }

                    // need a condition for if the obstacle can't be moved
                }
            }
        }

        // ideally should return a new state
        return newObstPos;
    }

    private boolean validateNewMovingObjectPos(MovingObstacle mo, State curState){
        //validate this position (given you are already out of path, need to check MovingBoxes/StaticObstacles/other
        // MovingObstacles ...not mo)
        for(MovingBox mb : curState.getMovingBoxes()){
            if(mb.getRect().intersects(mo.getRect())){
                return false;
            }
        }

        for(MovingObstacle altmo : curState.getMovingObstacles()){
            if(!altmo.equals(mo) && altmo.getRect().intersects(mo.getRect())){
                return false;
            }
        }

        for(StaticObstacle so : staticObstacles){
            if(so.getRect().intersects(mo.getRect())){
                return false;
            }
        }

        return true;
    }

    public List<MovingBox> buildStep(MovingBox origin, MovingBox end){

        // build path
        List<MovingBox> path = new ArrayList<>();
        path.add(origin);

        double goalX = getReducedDouble(end.getPos().getX(), 3);
        double goalY = getReducedDouble(end.getPos().getY(), 3);

        double curX = getReducedDouble(origin.getPos().getX(), 3);
        double curY = getReducedDouble(origin.getPos().getY(), 3);

        while(curX != goalX || curY != goalY){
            MovingBox last = path.get((path.size() - 1));

            double lastX = getReducedDouble(last.getPos().getX(), 3);
            double lastY = getReducedDouble(last.getPos().getY(), 3);

            // correct X position
            if (lastX < goalX) { //move right
                curX = lastX + minStepSize;
                curY = lastY;
            } else if (lastX > goalX) { //move left
                curX = lastX - minStepSize;
                curY = lastY;
            }

            // correct Y position
            else if (lastY < goalY) { //move up
                curX = lastX;
                curY = lastY + minStepSize;
            } else if (lastY > goalY) { //move down
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

    // the idea behind this method was to test for situations where the robot has to go around large obstacles
    private RobotConfig pathProj(RobotConfig prev, RobotConfig next, State state){

        List<RobotConfig> testPoints = new ArrayList<>();
        testPoints.add(new RobotConfig(
                new Point2D.Double(prev.getPos().getX(), next.getPos().getY()), prev.getOrientation()));
        testPoints.add(new RobotConfig(
                new Point2D.Double(next.getPos().getX(), prev.getPos().getY()), prev.getOrientation()));

        for(RobotConfig point : testPoints){
            if(validateRoboTransition(prev, point, state) &&
                    validateRoboTransition(point, next, state)){
                return point;    // should return adjoining point
            }
        }
        return null;
    }

    private boolean validateRoboTransition(RobotConfig prev, RobotConfig next, State state){

        Point2D projPoint =  new Point2D.Double(min(prev.getX1(roboWidth), next.getX1(roboWidth)) + minStepSize,
                min(prev.getY1(roboWidth), next.getY1(roboWidth)));
        double width = Math.abs(projPoint.getX() - max(prev.getX2(roboWidth), next.getX2(roboWidth))) - minStepSize;
        double height = Math.abs(projPoint.getY() - max(prev.getY2(roboWidth), next.getY2(roboWidth)));

        Rectangle2D pathProjection = new Rectangle2D.Double(projPoint.getX(), projPoint.getY(), width, height);

        for(MovingBox mb : state.getMovingBoxes()){
            if(pathProjection.intersects(mb.getRect())){
                return false;
            }
        }

        for(MovingObstacle altmo : state.getMovingObstacles()){
            if(pathProjection.intersects(altmo.getRect())){
                return false;
            }
        }

        for(StaticObstacle so : staticObstacles){
            if(pathProjection.intersects(so.getRect())){
                return false;
            }
        }

        return true;
    }

    private RobotConfig findNextRobo(RobotConfig curRobo, Point2D end, State state, List<RobotConfig> observed){

        double curBest = 1;
        RobotConfig path = curRobo;

        for(RobotConfig node : posRoboConfig){

            double dx = Math.abs(curRobo.getPos().getX() - node.getPos().getX());
            double dy = Math.abs(curRobo.getPos().getY() - node.getPos().getY());

            if(dx < 0.05 && dy < 0.05){
                double dist = node.getPos().distance(end);

//                if(pathProj(curRobo, node, state) != null){
//                    System.out.println(pathProj(curRobo, node, state).getPos().getX() +
//                            ", " + pathProj(curRobo, node, state).getPos().getY());
//                }

                if(validateRoboTransition(curRobo, node, state) && dist < curBest && !observed.contains(node)){
//                    System.out.println(curBest + " " + dist + " " + (dist < curBest) + " " + validateRoboTransition(curRobo, node, state));
//                    System.out.println(node.getPos().getX() +  ", " + node.getPos().getY());

                    curBest = dist;
                    path = node;

                }
            }
        }

        return path;
    }

    private List<State> pathBot(RobotConfig origin, State state, Point2D end){

        List<State> path = new ArrayList<>();
        List<RobotConfig> markers = new ArrayList<>();

        path.add(createNewState(state, focusBox, focusBox, state.getMovingObstacles(), origin));
        markers.add(origin);
        int count = 0;

        RobotConfig prev;
        RobotConfig next;

//        System.out.println(origin.getPos().getX() + ", " + origin.getPos().getY());
//        System.out.println(end.getX() + ", " + end.getY());
        do{
            // find neighbours
            prev = markers.get(markers.size() - 1);
            next = findNextRobo(prev, end, state, markers);
            // path to neighbour
//            System.out.println("prev: " + markers.get(markers.size() - 1).getPos().getX() + ", " + markers.get(markers.size() - 1).getPos().getY());
//            System.out.println("next: " + next.getPos().getX() + ", " + next.getPos().getY());

            path.addAll(moveBot(next.getPos(), state, prev));

//            path.add(new State(next, state.getMovingBoxes(), state.getMovingObstacles()));

            markers.add(next);
            count++;

//            System.out.println(count + ": " + next.getPos().getX() + ", " + next.getPos().getY());
//            System.out.println(count + ": " + focusBox.getDockPos().getX() + ", " + focusBox.getDockPos().getY());
//            System.out.println(count + ": " + end.equals(next.getPos()));
        }while (!end.equals(next.getPos()) && count < 50);

        // return total path
        return path;
    }

    // this is won't work "well" needs to be redone with c-space
    private List<State> moveBot(Point2D end, State state, RobotConfig marker){

        // check orientation (0 == horizontal. 90/180 == vertical)
        // robot-pos marked as the center of the robot

        List<State> path = new ArrayList<>();
        path.add(new State(marker, state.getMovingBoxes(), state.getMovingObstacles()));

        double curX = marker.getPos().getX();
        double curY = marker.getPos().getY();

        double goalX = getReducedDouble(end.getX(), 3);
        double goalY = getReducedDouble(end.getY(), 3);
//        System.out.println("cur: " + curX + ", " + curY);
//        System.out.println("goal: " + goalX + ", " + goalY);

        while(curX != goalX || curY != goalY){
            RobotConfig last = path.get((path.size() - 1)).getRobo();

            double lastX = getReducedDouble(last.getPos().getX(), 3);
            double lastY = getReducedDouble(last.getPos().getY(), 3);

            // correct X position
            if (lastX < goalX) { //move right
                curX = lastX + minStepSize;
                curY = lastY;
            } else if (lastX > goalX) { //move left
                curX = lastX - minStepSize;
                curY = lastY;
            }

            // correct Y position
            else if (lastY < goalY) { //move up
                curX = lastX;
                curY = lastY + minStepSize;
            } else if (lastY > goalY) { //move down
                curX = lastX;
                curY = lastY - minStepSize;
            }

            curX = getReducedDouble(curX, 3);
            curY = getReducedDouble(curY, 3);

//            System.out.println("cur: " + curX + ", " + curY);
            RobotConfig newRoboConfig = new RobotConfig(
                    new Point2D.Double(curX, curY), state.getRobo().getOrientation());

            // can check projected, if intersects with obstacle
            path.add(new State(newRoboConfig, state.getMovingBoxes(), state.getMovingObstacles()));
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
