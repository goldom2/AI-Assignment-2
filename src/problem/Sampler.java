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
    private State createNewState(State origin, MovingBox mb, MovingBox cur_pos, MovingBox old_pos,
                                 List<MovingObstacle> obsPos, RobotConfig newRobo, boolean following){

        List<MovingBox> newMovingBoxes = new ArrayList<>();
        RobotConfig robotConfig = newRobo;

        for(MovingBox box : origin.getMovingBoxes()){
            if(box.equals(mb)){
                MovingBox temp = new MovingBox(
                        new Point2D.Double(cur_pos.getPos().getX(), cur_pos.getPos().getY()), box.getWidth());
                if(following){
                    robotConfig = new RobotConfig(new Point2D.Double(
                            getReducedDouble(newRobo.getPos().getX() + (cur_pos.getPos().getX() - old_pos.getPos().getX()), 3),
                            getReducedDouble(newRobo.getPos().getY() + (cur_pos.getPos().getY() - old_pos.getPos().getY()), 3)
                    ), newRobo.getOrientation());
                }
                newMovingBoxes.add(temp);
            }else{
                newMovingBoxes.add(box);
            }
        }
        return new State(robotConfig, newMovingBoxes, obsPos);
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
     * @param box
     * @return
     */
    private void sampleNewState(State origin, MovingBox box){
        MovingBox temp;
        // Randomise samples around the board
        for (int i = 0; i < 100; i++) {
            do {
                temp = new MovingBox(
                        new Point2D.Double(Math.random(), Math.random()), box.getEndPos(), box.getWidth());
            } while (staticCollision(temp.getRect()));
            box.addToNodeList(temp);
        }
        // Add samples on static obstacle corners
        for (StaticObstacle staticObstacle : staticObstacles) {
            // Bottom left
            temp = new MovingBox(
                    new Point2D.Double(staticObstacle.getRect().getMinX() - box.getWidth(), staticObstacle.getRect().getMinY() - box.getWidth()),
                    box.getEndPos(), box.getWidth());
            if (!staticCollision(temp.getRect())) {
                box.addToNodeList(temp);
            }
            // Bottom right
            temp = new MovingBox(
                    new Point2D.Double(staticObstacle.getRect().getMaxX(), staticObstacle.getRect().getMinY() - box.getWidth()),
                    box.getEndPos(), box.getWidth());
            if (!staticCollision(temp.getRect())) {
                box.addToNodeList(temp);
            }
            // Top left
            temp = new MovingBox(
                    new Point2D.Double(staticObstacle.getRect().getMinX() - box.getWidth(), staticObstacle.getRect().getMaxY()),
                    box.getEndPos(), box.getWidth());
            if (!staticCollision(temp.getRect())) {
                box.addToNodeList(temp);
            }
            // Top right
            temp = new MovingBox(
                    new Point2D.Double(staticObstacle.getRect().getMaxX(), staticObstacle.getRect().getMinY()),
                    box.getEndPos(), box.getWidth());
            if (!staticCollision(temp.getRect())) {
                box.addToNodeList(temp);
            }
        }
        // Add goal to samples
        temp = new MovingBox(box.getEndPos(), box.getEndPos(), box.getWidth());
        box.addToNodeList(temp);
//        List<MovingBox> newMovingBoxes = new ArrayList<>();
//        MovingBox temp;
//
//        for(MovingBox box : origin.getMovingBoxes()){
//
//            if(box.equals(mb)){
//                do {
//                    temp = new MovingBox(
//                            new Point2D.Double(Math.random(), Math.random()), box.getEndPos(), box.getWidth());
//                } while (staticCollision(temp.getRect()));
//
//                newMovingBoxes.add(temp);
//                box.addToNodeList(temp);
//            }else{
//                newMovingBoxes.add(box);
//            }
//        }
    }

    /**
     * Returns true if the given Rectangle collides with any static obstacles or the edge of the board
     *
     * @param box
     * @return
     */
    public boolean staticCollision(Rectangle2D box) {
        for (StaticObstacle so : staticObstacles) {
            if (box.intersects(so.getRect())) {
                return true;
            }
        }
        Rectangle2D board = new Rectangle2D.Double(0, 0, 1, 1);
        if (!board.contains(box)) {
            return true;
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
        dockPos.add(new Point2D.Double(center.getX(), center.getY() + mb.getWidth() + 0.001));    //upper hand side
        dockPos.add(new Point2D.Double(center.getX(), center.getY() - mb.getWidth() - 0.001));    //under hand side

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
        angle = getReducedDouble(angle, 8);
//        System.out.println(angle);

        if(right && angle == 360){
            angle = 0;
        }else if(!right && angle == 0){
            angle = 360;
        }

//        System.out.println("cur angle: " + angle + " target: " + target);

        double maxIncrementAngle = getReducedDouble(minStepSize/(Math.PI*(roboWidth/2))*360, 8);
        double margin = getReducedDouble(maxIncrementAngle*Math.floor(Math.abs(target - angle)/maxIncrementAngle), 8);
        double offset = getReducedDouble(Math.abs(Math.abs(target - angle) - margin), 8);

//        System.out.println("da: " + maxIncrementAngle + " offset: " + offset);

        if(angle < target){
            while(angle < (target - offset)){
                angle += maxIncrementAngle;
                angle = getReducedDouble(angle, 8);
//                System.out.println(angle);
                path.add(new State(new RobotConfig(robo.getPos(), angle*(Math.PI/180)), state.getMovingBoxes(), state.getMovingObstacles()));
            }

            angle += offset;
//            System.out.println(Math.floor(angle)*(Math.PI/180));
            path.add(new State(new RobotConfig(robo.getPos(), Math.floor(angle)*(Math.PI/180)), state.getMovingBoxes(), state.getMovingObstacles()));
        }
        else if(angle > target){
            while(angle > (target + offset)){
                angle -= maxIncrementAngle;
                angle = getReducedDouble(angle, 8);
//                System.out.println(getReducedDouble(angle, 8));
                path.add(new State(new RobotConfig(robo.getPos(), angle*(Math.PI/180)), state.getMovingBoxes(), state.getMovingObstacles()));
            }

            angle -= offset;
//            System.out.println(Math.floor(angle)*(Math.PI/180));
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


    public List<State> sideRotate(State state, int face1, int face2, double halfWidth){
        List<State> path = new ArrayList<>();

        if(face1 == 1){ //Top
            if(face2 == 2){ // Right
                path.addAll(refaceRobotTransition(state, halfWidth, 1, false));
                path.addAll(rotateBot(270, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, true));
            }
            else { //Left
                path.addAll(refaceRobotTransition(state, halfWidth, -1, false));
                path.addAll(rotateBot(90, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, true));
            }
        }
        else if(face1 == 2){ // Right
            if(face2 == 1){ // Top
                path.addAll(refaceRobotTransition(state, halfWidth, 1, true));
                path.addAll(rotateBot(360, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, false));
            }
            else{ // Bottom
                path.addAll(refaceRobotTransition(state, halfWidth, -1, true));
                path.addAll(rotateBot(180, path.get(path.size() - 1), false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, false));
            }
        }
        else if(face1 == 3){ // Bottom
            if(face2 == 2){ // Right
                path.addAll(refaceRobotTransition(state, halfWidth, 1, false));
                path.addAll(rotateBot(270, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, true));
            }
            else{ // Left
                path.addAll(refaceRobotTransition(state, halfWidth, -1, false));
                path.addAll(rotateBot(90, path.get(path.size() - 1), false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, true));
            }
        }
        else if(face1 == 4){ // Left
            if(face2 == 3){ //Bottom
                path.addAll(refaceRobotTransition(state, halfWidth, -1, true));
                path.addAll(rotateBot(180, path.get(path.size() - 1), false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, false));
            }
            else{ // Top
                path.addAll(refaceRobotTransition(state, halfWidth, 1, true));
                path.addAll(rotateBot(0, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, false));
            }
        }
        return path;
    }

    public List<State> backRotate(State state, int face1, int face2, double halfWidth){
        List<State> path = new ArrayList<>();

        if(face1 == 1){ //Top
            if(face2 == 2){ // Right
                path.addAll(refaceRobotTransition(state, halfWidth, 1, true));
                path.addAll(rotateBot(270, path.get(path.size() - 1), false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth * 2, -1, true));
            }
            else { //Left
                path.addAll(refaceRobotTransition(state, halfWidth, 1, true));
                path.addAll(rotateBot(90, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth * 2, -1, true));
            }
        }
        else if(face1 == 2){ // Right
            if(face2 == 1){ // Top
                path.addAll(refaceRobotTransition(state, halfWidth, 1, false));
                path.addAll(rotateBot(0, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth * 2, -1, false));
            }
            else{ // Bottom
                path.addAll(refaceRobotTransition(state, halfWidth, 1, false));
                path.addAll(rotateBot(180, path.get(path.size() - 1), false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth * 2, -1, false));
            }
        }
        else if(face1 == 3){ // Bottom
            if(face2 == 2){ // Right
                path.addAll(refaceRobotTransition(state, halfWidth, -1, true));
                path.addAll(rotateBot(270, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth * 2, 1, true));
            }
            else{ // Left
                path.addAll(refaceRobotTransition(state, halfWidth, -1, true));
                path.addAll(rotateBot(90, path.get(path.size() - 1), false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth * 2, 1, true));
            }
        }
        else if(face1 == 4){ // Left
            if(face2 == 3){ //Bottom
                path.addAll(refaceRobotTransition(state, halfWidth, -1, false));
                path.addAll(rotateBot(180, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth * 2, 1, false));
            }
            else{ // Top
                path.addAll(refaceRobotTransition(state, halfWidth, -1, false));
                path.addAll(rotateBot(0, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth * 2, 1, false));
            }
        }
        return path;
    }

    public List<State> nextRotate(State state, int face1, int face2, double halfWidth){
        List<State> path = new ArrayList<>();

        if(face1 == 1){ //Top
            if(face2 == 2){ // Right
                path.addAll(refaceRobotTransition(state, halfWidth * 2, 1, false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, true));
                path.addAll(rotateBot(270, path.get(path.size() - 1), false));
                path.addAll(refaceRobotTransition(state, halfWidth, -1, false));


            }
            else { //Left
                path.addAll(refaceRobotTransition(state, halfWidth * 2, -1, false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, true));
                path.addAll(rotateBot(90, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(state, halfWidth, 1, false));
            }
        }
        else if(face1 == 2){ // Right
            if(face2 == 1){ // Top
                path.addAll(refaceRobotTransition(state, halfWidth * 2, 1, true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, false));
                path.addAll(rotateBot(0, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(state, halfWidth, -1, true));
            }
            else{ // Bottom
                path.addAll(refaceRobotTransition(state, halfWidth * 2, -1, true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, -1, false));
                path.addAll(rotateBot(180, path.get(path.size() - 1), false));
                path.addAll(refaceRobotTransition(state, halfWidth, 1, true));
            }
        }
        else if(face1 == 3){ // Bottom
            if(face2 == 2){ // Right
                path.addAll(refaceRobotTransition(state, halfWidth * 2, 1, false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, true));
                path.addAll(rotateBot(270, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(state, halfWidth, -1, false));
            }
            else{ // Left
                path.addAll(refaceRobotTransition(state, halfWidth * 2, -1, false));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, true));
                path.addAll(rotateBot(90, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(state, halfWidth, 1, false));
            }
        }
        else if(face1 == 4){ // Left
            if(face2 == 3){ //Bottom
                path.addAll(refaceRobotTransition(state, halfWidth * 2, -1, true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, false));
                path.addAll(rotateBot(180, path.get(path.size() - 1), true));
                path.addAll(refaceRobotTransition(state, halfWidth, 1, true));
            }
            else{ // Top
                path.addAll(refaceRobotTransition(state, halfWidth * 2, 1, true));
                path.addAll(refaceRobotTransition(path.get(path.size() - 1), halfWidth, 1, false));
                path.addAll(rotateBot(0, path.get(path.size() - 1), false));
                path.addAll(refaceRobotTransition(state, halfWidth, -1, true));
            }
        }
        return path;
    }

    public List<State> refaceRobot(MovingBox prev, MovingBox next, State state){
        List<State> path = new ArrayList<>();
        RobotConfig cur = state.getRobo();
        int face1 = 0;
        int face2 = 0;
        int intFace = 0;
        int method;
        int method2;
        double width = this.roboWidth;
        double halfWidth = width / 2;
        double deltaX;
        double deltaY;
        boolean flag = false;
        double x;
        double y;
//        System.out.println(state.getRobo().getOrientation());

        Point2D center = new Point2D.Double(prev.getPos().getX() + prev.getWidth()/2,
                prev.getPos().getY() + prev.getWidth()/2);
        Point2D nc = new Point2D.Double(next.getPos().getX() + next.getWidth()/2,
                next.getPos().getY() + next.getWidth()/2);

        //System.out.println("robot: " + cur.getPos().getX() + ", " + cur.getPos().getY());
        //System.out.println("center: " + center.getX() + ", " + center.getY());
        //System.out.println("next: " + nc.getX() + ", " + nc.getY());

        x = cur.getPos().getX();
        y = cur.getPos().getY();

        deltaX = center.getX() - x;
        deltaY = center.getY() - y;

        if(Math.abs(deltaX) < 0.004 && deltaX != 0){
            if(deltaX < 0){
                path.addAll(refaceRobotTransition(state, Math.abs(deltaX), -1, false));
                flag = true;
            }
            else{
                path.addAll(refaceRobotTransition(state, Math.abs(deltaX), 1, false));
                flag = true;
            }
            state = path.get(path.size() - 1);
            x = x + deltaX;
        }


        if(Math.abs(deltaY) < 0.004 && deltaY != 0){
            if(deltaY < 0){
                path.addAll(refaceRobotTransition(state, Math.abs(deltaY), -1, true));
                flag = true;
            }
            else{
                path.addAll(refaceRobotTransition(state, Math.abs(deltaY), 1, true));
                flag = true;
            }
            state = path.get(path.size() - 1);
            y = y + deltaY;

        }

        System.out.println("Delta: " + deltaX + ", " + deltaY);

        if(x < center.getX()){    //left
            face1 = 4;
//            System.out.println("bot left of center");
            if(nc.getY() > center.getY()){    //next pos is above prev pos so move bot on below
                face2 = 3;
            }else if(nc.getY() < center.getY()) {  //down so above
                face2 = 1;
            }else if(nc.getX() < center.getX()){  //left so right
                face2 = 2;
                intFace = 3;
            }
            else{
                return path;
            }

        }else if(x > center.getX()){  //right
            face1 = 2;
            if(nc.getY() > center.getY()){  //above so below
                face2 = 3;
            }else if(nc.getY() < center.getY()){  //above so below
                face2 = 1;
            }else if(nc.getX() > center.getX()){  //above so below
                face2 = 4;
                intFace = 3;
            }
            else{
                return path;
            }

        }else if(y < center.getY()){  //down
            face1 = 3;
            if(nc.getX() < center.getX()){  //left so right
                face2 = 2;
            }else if(nc.getX() > center.getX()){  //right so left
                face2 = 4;
            }else if(nc.getY() < center.getY()){  //below so above
                face2 = 1;
                intFace = 2;
            }
            else{
                return path;
            }

        }else if(y > center.getY()){  //up
            face1 = 1;
            if(nc.getX() < center.getX()){  //left so right
                face2 = 2;
            }else if(nc.getX() > center.getX()){  //right so left
                face2 = 4;
            }else if(nc.getY() > center.getY()){  //above so below
                face2 = 3;
                intFace = 2;
            }
            else{
                return path;
            }
        }

        if(flag){
            state = path.get(path.size() - 1);
        }

        if(intFace == 0){
            method = checkRotation(state, face1, face2);
            System.out.println("method: " + method + "//////////////////////////////////////////////////////////////////");
            if(method ==1){
                path.addAll(sideRotate(state, face1, face2, halfWidth));
            }
            else if(method == 2){
                path.addAll(backRotate(state, face1, face2, halfWidth));
            }
            else if(method == 3){
                path.addAll(nextRotate(state, face1, face2, halfWidth));
            }
            if(method == 0){
                System.out.println("Shit");
            }

        }
        else{
            System.out.println("Switch");
            method = checkRotation(state, face1, intFace);
            method2 = checkRotation(state, intFace, face2);
            if(method == 0 || method2 == 0){
                if(intFace == 2){
                    intFace = 4;
                }
                else{
                    intFace = 1;
                }
                method = checkRotation(state, face1, intFace);
                method2 = checkRotation(state, intFace, face2);
            }
            if(method ==1){
                path.addAll(sideRotate(state, face1, intFace, halfWidth));
            }
            else if(method == 2){
                path.addAll(backRotate(state, face1, intFace, halfWidth));
            }
            else if(method == 3){
                path.addAll(nextRotate(state, face1, intFace, halfWidth));
            }
            if(method2 ==1){
                path.addAll(sideRotate(state, intFace, face2, halfWidth));
            }
            else if(method2 == 2){
                path.addAll(backRotate(state, intFace, face2, halfWidth));
            }
            else if(method2 == 3){
                path.addAll(nextRotate(state, intFace, face2, halfWidth));
            }
        }

        return path;
    }

    public boolean checkRect(State state, double x, double y, double w, double h){
        List<MovingBox> movingboxes = state.getMovingBoxes();
        List<MovingObstacle> movingObstacles = state.getMovingObstacles();

        x = x + 0.0001;
        y = y + 0.0001;
        w = w - 0.0002;
        h = h - 0.0002;

        if(x < 0 || y < 0 || x > 1 - h/2 || y > 1 - h/2){
            System.out.println("Wall");
            return false;
        }

        Rectangle2D rect = new Rectangle2D.Double(x, y, w, h);

        if(staticCollision(rect)){
            System.out.println("Static");
            return false;
        }

        for(MovingObstacle box : movingObstacles){
            if(box.getRect().intersects(rect)){
                System.out.println("obstacle");
                return false;
            }
        }
        return true;

        //for(MovingBox box : movingboxes){
        //    if(box.getRect().intersects(rect)){
        //        System.out.println("box");
        //        return false;
        //    }
        //}


        //return true;
    }

    public boolean checkSide(State state, double halfWidth, double centX, double centY, int side){
        double x;
        double y;

        if(side == 1){
            x = centX - halfWidth;
            y = centY + halfWidth;
        }
        else if(side == 2){
            x = centX + halfWidth;
            y = centY - halfWidth;
        }
        else if(side == 3){
            x = centX - halfWidth;
            y = centY - (3 * halfWidth);
        }
        else{
            x = centX - (3 * halfWidth);
            y = centY - halfWidth;
        }
        return checkRect(state, x, y, halfWidth * 2, halfWidth * 2);
    }


    public int checkRotation(State state, int face1, int face2){
        // 1 Top, 2 Right, 3 Bottom, 4 Left
        System.out.println("Face 1: " + face1 + ", Face 2: " +face2);
        double width = this.roboWidth;
        double halfWidth = width / 2;
        RobotConfig cur = state.getRobo();
        double y;
        double x;
        double centX;
        double centY;

        //Top
        //Top left
        if((face1 == 1 && face2 == 4) ||(face2 == 1 && face1 == 4) ){
            y = cur.getPos().getY();
            x = cur.getPos().getX() - halfWidth;
            if(checkRect(state, x, y, halfWidth, halfWidth)){
                y = y - halfWidth;
                x = x - halfWidth;
                if(checkRect(state, x, y, halfWidth, halfWidth)){
                    return 1;
                }
            }
            if(face1 == 1){
                centX = cur.getPos().getX();
                centY = cur.getPos().getY() - halfWidth;
                if(checkSide(state, halfWidth, centX, centY, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, centX, centY, face2)){
                    return 3;
                }
            }
            else{
                centX = cur.getPos().getX() + halfWidth;
                centY = cur.getPos().getY();
                if(checkSide(state, halfWidth, centX, centY, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, centX, centY, face2)){
                    return 3;
                }

            }

            return 0;
        }
        //Top right
        else if((face1 == 1 && face2 == 2)||(face2 == 1 && face1 == 2)){
            y = cur.getPos().getY();
            x = cur.getPos().getX();
            if(checkRect(state, x, y, halfWidth, halfWidth)){
                y = y - halfWidth;
                x = x + halfWidth;
                if(checkRect(state, x, y, halfWidth, halfWidth)){
                    return 1;
                }
            }
            if(face1 == 1){
                centX = cur.getPos().getX();
                centY = cur.getPos().getY() - halfWidth;
                if(checkSide(state, halfWidth, centX, centY, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, centX, centY, face2)){
                    return 3;
                }
            }
            else{
                centX = cur.getPos().getX() - halfWidth;
                centY = cur.getPos().getY();
                if(checkSide(state, halfWidth, centX, centY, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, centX, centY, face2)){
                    return 3;
                }

            }


            return 0;
        }
        //Bottom
        //Bottom left
        else if((face1 == 3 && face2 == 4)||(face2 == 3 && face1 ==4 )){
            y = cur.getPos().getY() - halfWidth;
            x = cur.getPos().getX() - halfWidth;
            if(checkRect(state, x, y, halfWidth, halfWidth)){
                y = y + halfWidth;
                x = x - halfWidth;
                if(checkRect(state, x, y, halfWidth, halfWidth)){
                    return 1;
                }
            }
            if(face1 == 3){
                centX = cur.getPos().getX();
                centY = cur.getPos().getY() + halfWidth;
                if(checkSide(state, halfWidth, centX, centY, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, centX, centY, face2)){
                    return 3;
                }
            }
            else{
                centX = cur.getPos().getX() + halfWidth;
                centY = cur.getPos().getY();
                if(checkSide(state, halfWidth, centX, centY, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, centX, centY, face2)){
                    return 3;
                }

            }


            return 0;
        }
        //Bottom right
        else if((face1 == 3 && face2 == 2)||(face2 == 3 && face1 == 2)){
            y = cur.getPos().getY() - halfWidth;
            x = cur.getPos().getX();
            if(checkRect(state, x, y, halfWidth, halfWidth)){
                y = y + halfWidth;
                x = x + halfWidth;
                if(checkRect(state, x, y, halfWidth, halfWidth)){
                    return 1;
                }
            }
            if(face1 == 3){
                centX = cur.getPos().getX();
                centY = cur.getPos().getY() + halfWidth;
                if(checkSide(state, halfWidth, centX, centY, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, centX, centY, face2)){
                    return 3;
                }
            }
            else{
                centX = cur.getPos().getX() - halfWidth;
                centY = cur.getPos().getY();
                if(checkSide(state, halfWidth, centX, centY, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, centX, centY, face2)){
                    return 3;
                }

            }

            return 0;
        }
        return -1;
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
                path.addAll(rotateBot(0, state, true));
            }else{
                path.addAll(rotateBot(180, state, true));
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


    public void stepObjectiveSampling(){
        State og = new State(robo, movingBoxes, movingObstacles);

        List<State> path = new ArrayList<>();
        path.add(og);

        posRoboConfig = sampleNewRobo(og, robo);

        //mapped moving Boxes at assumed states
        for(MovingBox mb : movingBoxes){

            //find the most accessible/closest position to "dock" the robot
            Point2D robDockPos = findDock(robo, mb);
            mb.setDockPos(robDockPos);

            posRoboConfig.add(new RobotConfig(robDockPos, robo.getOrientation()));

//            for(int i = 0; i < 1000; i++){
                sampleNewState(og, mb);
//            }

//            mb.addToNodeList(new MovingBox(mb.getEndPos(), mb.getEndPos(), mb.getWidth()));
        }

        State step = og;    //the last step

        for(MovingBox mbog : movingBoxes){ //hard limit placed

            MovingBox init = mbog;
            State intoNewBox = step;

            if(!mbog.getDockPos().equals(robo.getPos())){   //path to box
                path.addAll(pathBot(robo, intoNewBox, mbog.getDockPos()));
            }

            //orient robot based on position
            path.addAll(orientRobot(mbog, path.get(path.size() - 1)));
            List<MovingBox> boxPath = (List) findBoxPath(mbog, new MovingBox(mbog.getEndPos(), mbog.getEndPos(), mbog.getWidth()), (Set) mbog.getNodeList());
//            MovingBox next = boxPath.get(1);
            for (MovingBox next : boxPath) {

//              System.out.println(next.getPos().getX() + ", " + next.getPos().getY());

                // check path between origin and end (Rec around the origin and end points)
                Point2D mapOrigin = new Point2D.Double(min(init.getRect().getMinX(), next.getRect().getMinX()),
                        min(init.getRect().getMinY(), next.getRect().getMinY()));

                double width = Math.abs(mapOrigin.getX() - max(init.getRect().getMaxX(), next.getRect().getMaxX()));
                double height = Math.abs(mapOrigin.getY() - max(init.getRect().getMaxY(), next.getRect().getMaxY()));

                Rectangle2D pathSpace = new Rectangle2D.Double(
                        mapOrigin.getX(), mapOrigin.getY(), width, height);

                List<MovingObstacle> newMoPos = moveMovingObstacles(pathSpace, step);

                MovingBox last = init;
                MovingBox intermediate = (MovingBox) joinNodes(init, next);
//                    MovingBox sStep = buildStep(init, next).get(1);

                // Path to intermediate node
                for(MovingBox sStep : buildStep(init, intermediate)){
                    og = path.get(path.size() - 1);
                    path.addAll(refaceRobot(last, sStep, og));

                    og = path.get(path.size() - 1);
                    step = createNewState(intoNewBox, mbog, sStep, last, newMoPos, og.getRobo(), true);
                    path.add(step);

                    last = sStep;
                }
                // Path from intermediate node to next node
                for(MovingBox sStep : buildStep(intermediate, next)){
                    og = path.get(path.size() - 1);
                    path.addAll(refaceRobot(last, sStep, og));

                    og = path.get(path.size() - 1);
                    step = createNewState(intoNewBox, mbog, sStep, last, newMoPos, og.getRobo(), true);
                    path.add(step);

                    last = sStep;
                }
                init = next;

//              count++;
            }
        }

        printOutput(path);
    }

    /**
     * Return a set of neighbour nodes to mb from the given set
     * Neighbours are within a radius and have a path between one another
     *
     * @param samples
     * @param mb
     * @return
     */
    public Set<Box> getNeighbourNodes(Set<Box> samples, Box mb){

        Set<Box> neighbourNodes = new HashSet<>();

        for(Box node : samples){
            double dx = Math.abs(node.getPos().getX() - mb.getPos().getX());
            double dy = Math.abs(node.getPos().getY() - mb.getPos().getY());

            if(dx < 1 && dy < 1 && !node.getPos().equals(mb.getPos())){
                if (joinNodes(mb, node) != null) {
                    neighbourNodes.add(node);
                }
            }
        }
//        System.out.println("Number of neighbours: " + neighbourNodes.size() + " Number of nodes: " + focusBox.getNodeList().size());
        return neighbourNodes;
    }

    /**
     * Finds an intermediate position that link two goals without intersections
     *
     * @param start
     * @param goal
     * @return
     */
    private Box joinNodes(Box start, Box goal) {
        // Path 1
        Rectangle2D intermediatePos = new Rectangle2D.Double(start.getPos().getX(),
                goal.getPos().getY(), start.getWidth(), start.getWidth());
        if (!staticCollision(start.getRect().createUnion(intermediatePos))
                && !staticCollision(goal.getRect().createUnion(intermediatePos))) {
            return new MovingBox(
                    new Point2D.Double(intermediatePos.getMinX(), intermediatePos.getMinY()), start.getWidth());
        }
        // Path 2
        intermediatePos = new Rectangle2D.Double(goal.getPos().getX(),
                start.getPos().getY(), start.getWidth(), start.getWidth());
        if (!staticCollision(start.getRect().createUnion(intermediatePos))
                && !staticCollision(goal.getRect().createUnion(intermediatePos))) {
            return new MovingBox(
                    new Point2D.Double(intermediatePos.getMinX(), intermediatePos.getMinY()), start.getWidth());
        }

        return null;
    }

    /**
     * Find shortest path through set of samples to get from the initial position (mb) to the goal
     *
     * @param mb
     * @param goal
     * @param samples
     * @return
     */
    public List<Box> findBoxPath(Box mb, Box goal, Set<Box> samples) {
        // Initialise all the queues and sets to run the search
        Set<Node> queue = new HashSet<>();
        Set<Point2D> visited = new HashSet<>();
        Node current = new Node(mb, 0.0, heuristic(mb, goal), null); //how can you minimise if the initial weight is zero
        queue.add(current);

        // Continue search until there is no more options or the goal is reached
        while (!queue.isEmpty()) {
            current = getMinimumNode(queue);
            System.out.println("size: " + queue.size() + "  \tcur pos: "
                    + current.getBox().getPos().getX() + ", " + current.getBox().getPos().getY()
                    + "  \tcur weight: " + current.getWeight()
                    + " \theuristic: " + current.getHeuristic()
                    + " \tsum: " + (current.getWeight() + current.getHeuristic()));

            queue.remove(current);
            visited.add(current.getBox().getPos());
            if (current.getBox().equals(goal)) {
                break;
            }
            for (Box box : getNeighbourNodes(samples, current.getBox())) {
                if (visited.contains(box.getPos())) {
                    continue;
                }
                double weight = current.getWeight() +
                        + Math.abs(current.getBox().getPos().getX() - box.getPos().getX())
                        + Math.abs(current.getBox().getPos().getY() - box.getPos().getY());
                queue.add(new Node(box, weight, heuristic(box, goal), current));
            }
        }

        if (!current.getBox().equals(goal)) {
            System.out.println("Could not find a path to the goal");
        }

        // Create the list of boxes
        List<Box> boxPath = new ArrayList<>();
        while (current != null) {
            boxPath.add(0, current.getBox());
            current = current.getPrev();
        }

        System.out.println("final: " + boxPath.get(boxPath.size() - 1).getPos().getX()
                + ", " + boxPath.get(boxPath.size() - 1).getPos().getY());
        System.out.println("goal: " + goal.getPos().getX() + ", " + goal.getPos().getY());
        return boxPath;
    }

    /**
     * Return the Node with the smallest distance from the start node and expected path to goal
     *
     * @param queue
     * @return
     */
    public Node getMinimumNode(Set<Node> queue) {
        Node result = null;
        double minWeight = 0;
        for (Node node : queue) {
            if (result == null || (node.getWeight() + node.getHeuristic()) < minWeight) {
                result = node;
                minWeight = node.getWeight() + node.getHeuristic();
            }
        }
        return result;
    }

    /**
     * Return the value of the heuristic for getting from box to goal
     *
     * @param box
     * @param goal
     * @return
     */
    public double heuristic(Box box, Box goal) {
        return distanceToGoal(box, goal);
    }

    /**
     * Calculates the minimum distance to the goal
     *
     * @return
     */
    public double distanceToGoal(Box box, Box goal) {
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

        double goalX = end.getPos().getX();
        double goalY = end.getPos().getY();

        double curX = origin.getPos().getX();
        double curY = origin.getPos().getY();

        while(curX != goalX || curY != goalY){

            // Adjust position of x or y up to maximum step size
            if (curX != goalX) {
                if (Math.abs(curX - goalX) > minStepSize) {
                    curX = (curX - goalX > 0) ? curX - minStepSize : curX + minStepSize;
                } else {
                    curX = goalX;
                }
            } else if (curY != goalY) {
                if (Math.abs(curY - goalY) > minStepSize) {
                    curY = (curY - goalY > 0) ? curY - minStepSize : curY + minStepSize;
                } else {
                    curY = goalY;
                }
            }

            // need check to see if path intersects with moving object
            // if the path intersects with the moving object
            // move intersecting object ...new c-space from this point???
            // ideally want to move it out of the path
            // given the end and origin move obstacle away from end??? ...that would be origin though???

            MovingBox projMB = new MovingBox(
                    new Point2D.Double(curX, curY), roboWidth);

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
    private List<State> moveBot(Point2D end, State origin, RobotConfig marker){

        // build path
        List<State> path = new ArrayList<>();
        path.add(origin);

        double goalX = end.getX();
        double goalY = end.getY();

        double curX = origin.getRobo().getPos().getX();
        double curY = origin.getRobo().getPos().getY();

        while(curX != goalX || curY != goalY){

            // Adjust position of x or y up to maximum step size
            if (curX != goalX) {
                if (Math.abs(curX - goalX) > minStepSize) {
                    curX = (curX - goalX > 0) ? curX - minStepSize : curX + minStepSize;
                } else {
                    curX = goalX;
                }
            } else if (curY != goalY) {
                if (Math.abs(curY - goalY) > minStepSize) {
                    curY = (curY - goalY > 0) ? curY - minStepSize : curY + minStepSize;
                } else {
                    curY = goalY;
                }
            }

//            System.out.println("cur: " + curX + ", " + curY);
            RobotConfig newRoboConfig = new RobotConfig(
                    new Point2D.Double(curX, curY), origin.getRobo().getOrientation());

            // can check projected, if intersects with obstacle
            path.add(new State(newRoboConfig, origin.getMovingBoxes(), origin.getMovingObstacles()));
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
