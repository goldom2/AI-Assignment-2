package problem;

import javax.swing.plaf.synth.SynthLookAndFeel;
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

    /**
     * Samples a state for the given MovingBox leaving all other boxes where they are
     *
     * @param origin
     * @param mb
     * @return
     */
    private State sampleNewState(State origin, MovingBox mb){
        List<MovingBox> newMovingBoxes = new ArrayList<>();
        MovingBox temp;

        for(MovingBox box : origin.getMovingBoxes()){

            if(box.equals(mb)){
                do {
                    temp = new MovingBox(
                            new Point2D.Double(Math.random(), Math.random()), box.getEndPos(), box.getWidth());
                } while (staticCollision(temp));

                newMovingBoxes.add(temp);
                box.addToNodeList(temp);
            }else{
                newMovingBoxes.add(box);
            }
        }

        State newState = new State(origin.getRobo(), newMovingBoxes, origin.getMovingObstacles());

        if(newState.isValid(staticObstacles)){
            return newState;
        }
        return null;
    }

    /**
     * Returns true if the given box collides with any static obstacles
     *
     * @param box
     * @return
     */
    public boolean staticCollision(Box box) {
        for (StaticObstacle so : staticObstacles) {
            if (box.getRect().intersects(so.getRect())) {
                return true;
            }
        }
        return false;
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

    public List<State> rotateBot(double target, State state, boolean right){
        List<State> path = new ArrayList<>();
        RobotConfig robo = state.getRobo();

        double angle = robo.getOrientation()/(Math.PI/180);
//        System.out.println(angle);

        if(right && angle == 360){
            angle = 0;
        }else if(!right && angle == 0){
            angle = 360;
        }

        System.out.println("cur angle: " + angle + " target: " + target);

        double maxIncrementAngle = (minStepSize/(Math.PI*(roboWidth/2)))*360;
        double margin = maxIncrementAngle*Math.floor(Math.abs(target - angle)/maxIncrementAngle);
        double offset = Math.abs(target - margin);

        if(angle < target){
            while(angle < (margin - offset)){
                angle += maxIncrementAngle;
//                System.out.println(angle);
                path.add(new State(new RobotConfig(robo.getPos(), angle*(Math.PI/180)), state.getMovingBoxes(), state.getMovingObstacles()));
            }

            angle += offset;
//            System.out.println(Math.floor(angle)*(Math.PI/180));
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

    public List<State> refaceRobotTransition(State state, double dist, double dir, boolean updown){
        List<State> path = new ArrayList<>();
        double orien = state.getRobo().getOrientation();
        double step = 0;

        RobotConfig newRobo = state.getRobo();

        while(step < dist){
            if(!updown){
                newRobo = new RobotConfig(
                        new Point2D.Double(newRobo.getPos().getX() + dir*minStepSize, newRobo.getPos().getY()), orien);
            }else{
                newRobo = new RobotConfig(
                        new Point2D.Double(newRobo.getPos().getX(), newRobo.getPos().getY() + dir*minStepSize), orien);
            }

            path.add(new State(
                    newRobo,
                    state.getMovingBoxes(),
                    state.getMovingObstacles()
            ));

            step += minStepSize;
        }

        return path;
    }

    public List<State> refaceRobot(MovingBox prev, MovingBox next, State state){
        List<State> path = new ArrayList<>();
        RobotConfig cur = state.getRobo();
        System.out.println(state.getRobo().getOrientation());

        Point2D center = new Point2D.Double(prev.getPos().getX() + prev.getWidth()/2,
                prev.getPos().getY() + prev.getWidth()/2);
        Point2D nc = new Point2D.Double(next.getPos().getX() + next.getWidth()/2,
                next.getPos().getY() + next.getWidth()/2);

        System.out.println("robot: " + cur.getPos().getX() + ", " + cur.getPos().getY());
        System.out.println("center: " + center.getX() + ", " + center.getY());
        System.out.println("next: " + nc.getX() + ", " + nc.getY());

        if(cur.getPos().getX() < center.getX()){    //left
            System.out.println("bot left of center");

            if(nc.getY() < center.getY()){    //next pos is under prev pos so move bot on top

            }else if(nc.getY() > center.getY()){  //above so below

            }else if(nc.getX() < center.getX()){  //left so right
                System.out.println("move to the left");
                path.addAll(refaceRobotTransition(state, prev.getWidth()/2, 1, true));
                path.addAll(rotateBot(180, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), prev.getWidth(), 1, false));
                path.addAll(rotateBot(270, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), prev.getWidth()/2, -1, true));
                System.out.println("path to rotate: " + path.size());
            }

        }else if(cur.getPos().getX() > center.getX()){  //right
            System.out.println("bot right of center");

        }else if(cur.getPos().getY() < center.getY()){  //down
            System.out.println("bot below of center");

        }else if(cur.getPos().getY() > center.getY()){  //up
            System.out.println("bot above of center");

        }

        return path;
    }

    public List<State> orientRobot(MovingBox mb, State state){
        RobotConfig cur = state.getRobo();
        List<State> path = new ArrayList<>();
        path.add(state);

        Point2D center = new Point2D.Double(mb.getPos().getX() + mb.getWidth()/2, mb.getPos().getY() + mb.getWidth()/2);

        if(cur.getPos().getX() > center.getX() || cur.getPos().getX() < center.getX()){    //right/left
            if(cur.getPos().getX() > center.getX()){
                path.addAll(rotateBot(270, state, true));
            }else{
                path.addAll(rotateBot(90, state, true ));
            }
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
            if(cur.getPos().getY() > center.getY()){
                path.addAll(rotateBot(180, state, true));
            }else{
                path.addAll(rotateBot(360, state, true));
            }

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

            mb.addToNodeList(new MovingBox(mb.getEndPos(), mb.getWidth()));

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


//            int count = 0;

//            while(count < 1000){
//                MovingBox next = continueMovingBoxPath(init, observed);
            List<MovingBox> boxPath = findBoxPath(focusBox);
            MovingBox next = boxPath.get(1);
//            for (MovingBox next : boxPath) {

                if(next != null) {
//                    System.out.println(next.getPos().getX() + ", " + next.getPos().getY());

                    // check path between origin and end (Rec around the origin and end points)
                    Point2D mapOrigin = new Point2D.Double(min(init.getRect().getMinX(), next.getRect().getMinX()),
                            min(init.getRect().getMinY(), next.getRect().getMinY()));

                    double width = Math.abs(mapOrigin.getX() - max(init.getRect().getMaxX(), next.getRect().getMaxX()));
                    double height = Math.abs(mapOrigin.getY() - max(init.getRect().getMaxY(), next.getRect().getMaxY()));

                    Rectangle2D pathSpace = new Rectangle2D.Double(
                            mapOrigin.getX(), mapOrigin.getY(), width, height);

                    List<MovingObstacle> newMoPos = moveMovingObstacles(pathSpace, step);

                    MovingBox last = init;
                    MovingBox sStep = buildStep(init, next).get(1);
//                    for(MovingBox sStep : buildStep(init, next)){
                        State pastState = path.get(path.size() - 1);
                        path.addAll(refaceRobot(last, sStep, pastState));

                        step = createNewState(intoNewBox, focusBox, sStep, newMoPos, pastState.getRobo());
                        path.add(step);

                        last = sStep;
//                    }

                    observed.add(next);
                    init = next;
                }else{
                    System.out.println("next == null");
                }

//                count++;
            }
//        }

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

            if(dx < 0.1 && dy < 0.1 && !node.getPos().equals(mb.getPos())){

                double ddx = Math.abs(node.getPos().getX() - focusBox.getEndPos().getX());
                double ddy = Math.abs(node.getPos().getY() - focusBox.getEndPos().getY());

                node.setDistanceToGoal(Math.sqrt(Math.pow(ddx, 2) + Math.pow(ddy, 2)));

                neighbourNodes.add(node);
            }
        }
        System.out.println("Number of neighbours: " + neighbourNodes.size() + " Number of nodes: " + focusBox.getNodeList().size());
        return neighbourNodes;
    }

    public List<MovingBox> findBoxPath(MovingBox mb) {
//        System.out.println(mb.getNodeList().size());

        MovingBox goal = new MovingBox(mb.getEndPos(), mb.getWidth());
        Set<Node> queue = new HashSet<>();
        Set<Point2D> visited = new HashSet<>();
        Node current = new Node(mb, 0.0, heuristic(mb, goal), null); //how can you minimise if the initial weight is zero
        queue.add(current);

        while (!queue.isEmpty()) {
            current = getMinimumNode(queue);
            System.out.println("size: " + queue.size() + " cur pos: "
                    + current.getBox().getPos().getX() + ", " + current.getBox().getPos().getY()
                    + " cur weight: " + current.getWeight()
                    + " heuristic: " + current.getHeuristic()
                    + " sum: " + (current.getWeight() + current.getHeuristic()));

            queue.remove(current);
            visited.add(current.getBox().getPos());
            if (current.getBox().equals(goal)) {
                break;
            }
            for (MovingBox box : getNeighbourNodes(current.getBox())) {
//                System.out.println("-->> " + box.getPos().getX() + ", " + box.getPos().getY());
                if (visited.contains(box.getPos())) {
                    continue;
                }
                double weight = current.getWeight() +
                        + Math.abs(current.getBox().getPos().getX() - box.getPos().getX())
                        + Math.abs(current.getBox().getPos().getY() - box.getPos().getY());
//                double weight = heuristic(box, goal);
                queue.add(new Node(box, weight, heuristic(box, goal), current));
            }
        }

//        for(MovingBox obsv : observed){
//            System.out.println(obsv.getPos().getX() + ", " +
//                    obsv.getPos().getY());
//        }

        if (!current.getBox().equals(goal)) {
            System.out.println("Could not find a path to the goal");
//            return null;
        }

        List<MovingBox> boxPath = new ArrayList<>();
        while (current != null) {
            boxPath.add(0, current.getBox());
            current = current.getPrev();
        }

        System.out.println("final: " + boxPath.get(boxPath.size() - 1).getPos().getX()
                + ", " + boxPath.get(boxPath.size() - 1).getPos().getY());
        System.out.println("goal: " + goal.getPos().getX() + ", " + goal.getPos().getY());
        return boxPath;
    }

    public Node getMinimumNode(Set<Node> queue) {
        Node result = null;
        double minWeight = 0;
        for (Node node : queue) {
            // Excluding the heuristic makes it perform like BFS
//            if (result == null || node.getWeight() < minWeight) {
//                result = node;
//                minWeight = node.getWeight();
//            }
            if (result == null || (node.getWeight() + node.getHeuristic()) < minWeight) {
                result = node;
                minWeight = node.getWeight() + node.getHeuristic();
            }
        }
        return result;
    }

    public double heuristic(MovingBox box, MovingBox goal) {
        return distanceToGoal(box, goal);
    }

    /**
     * Calculates the minimum distance to the goal
     *
     * @return
     */
    public double distanceToGoal(MovingBox box, MovingBox goal) {
        double result = 0;
        result += Math.abs(box.getPos().getX() - goal.getPos().getX());
        result += Math.abs(box.getPos().getY() - goal.getPos().getY());
        return result;
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
