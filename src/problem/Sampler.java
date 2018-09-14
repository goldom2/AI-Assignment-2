package problem;

import javax.sound.midi.SysexMessage;
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
     * Create a new state moving box from old position to current position
     *
     * @param state
     * @param new_pos
     * @param old_pos
     * @return
     */
    private State createNewState(State state, Box new_pos, Box old_pos){

        List<MovingBox> newMovingBoxes = new ArrayList<>();
        RobotConfig robotConfig = state.getRobo();

        for(MovingBox box : state.getMovingBoxes()){
            if(box.equals(old_pos)){
                robotConfig = new RobotConfig(new Point2D.Double(
                        robotConfig.getPos().getX() + (new_pos.getPos().getX() - old_pos.getPos().getX()),
                        robotConfig.getPos().getY() + (new_pos.getPos().getY() - old_pos.getPos().getY())),
                        robotConfig.getOrientation());
                newMovingBoxes.add((MovingBox) new_pos);
            }else{
                newMovingBoxes.add(box);
            }
        }
        List<MovingObstacle> newMovingObstacles = new ArrayList<>();
        for(MovingObstacle obstacle : state.getMovingObstacles()){
            if(obstacle.equals(old_pos)){
                robotConfig = new RobotConfig(new Point2D.Double(
                        robotConfig.getPos().getX() + (new_pos.getPos().getX() - old_pos.getPos().getX()),
                        robotConfig.getPos().getY() + (new_pos.getPos().getY() - old_pos.getPos().getY())),
                        robotConfig.getOrientation());
                newMovingObstacles.add((MovingObstacle) new_pos);
            }else{
                newMovingObstacles.add(obstacle);
            }
        }
        return new State(robotConfig, newMovingBoxes, newMovingObstacles);
    }

    private boolean sampleRoboStaticCollision(RobotConfig robo){

        Point2D origin;
        Point2D end;

        if(robo.getOrientation() == 0 || robo.getOrientation() == Math.PI){
            origin = new Point2D.Double(robo.getPos().getX() - roboWidth/2, robo.getPos().getY());
            end = new Point2D.Double(robo.getPos().getX() + roboWidth/2, robo.getPos().getY());
        }else{
            origin = new Point2D.Double(robo.getPos().getX(), robo.getPos().getY() - roboWidth/2);
            end = new Point2D.Double(robo.getPos().getX(), robo.getPos().getY() + roboWidth/2);
        }

        Line2D roboModel = new Line2D.Double(origin, end);

        for(StaticObstacle so : staticObstacles){
            if(roboModel.intersects(so.getRect())){
                return true;
            }
        }
        Rectangle2D board = new Rectangle2D.Double(0, 0, 1, 1);
        if (!board.contains(origin) || !board.contains(origin)) {
            return true;
        }

        return false;
    }

    private Set<RobotConfig> sampleNewRobo(RobotConfig origin){

        Set<RobotConfig> posRobo = new HashSet<>();
        RobotConfig temp;

        // Randomise samples around the board
        for (int i = 0; i < 100; i++) {
            do {
                temp = new RobotConfig(
                        new Point2D.Double(Math.random(), Math.random()), origin.getOrientation());
            } while (sampleRoboStaticCollision(temp));
            posRobo.add(temp);
        }

        // Add samples on static obstacle corners
        for (StaticObstacle staticObstacle : staticObstacles) {
            // Bottom left
            temp = new RobotConfig(
                    new Point2D.Double(staticObstacle.getRect().getMinX(), staticObstacle.getRect().getMinY()),
                    origin.getOrientation());
            if (!sampleRoboStaticCollision(temp)) {
                posRobo.add(temp);
            }
            // Bottom right
            temp = new RobotConfig(
                    new Point2D.Double(staticObstacle.getRect().getMaxX(), staticObstacle.getRect().getMinY()),
                    origin.getOrientation());
            if (!sampleRoboStaticCollision(temp)) {
                posRobo.add(temp);
            }
            // Top left
            temp = new RobotConfig(
                    new Point2D.Double(staticObstacle.getRect().getMinX(), staticObstacle.getRect().getMaxY()),
                    origin.getOrientation());
            if (!sampleRoboStaticCollision(temp)) {
                posRobo.add(temp);
            }
            // Top right
            temp = new RobotConfig(
                    new Point2D.Double(staticObstacle.getRect().getMaxX(), staticObstacle.getRect().getMinY()),
                    origin.getOrientation());
            if (!sampleRoboStaticCollision(temp)) {
                posRobo.add(temp);
            }
        }

        return posRobo;
    }

    /**
     * Samples a state for the given MovingBox leaving all other boxes where they are
     *
     * @param box
     * @return
     */
    private Set<Box> sampleNewState(Box box){
        Set<Box> samples = new HashSet<>();
        Box temp;
        if (box instanceof MovingBox) {
            MovingBox mb = (MovingBox) box;
            // Randomise samples around the board
            for (int i = 0; i < 100; i++) {
                do {
                    temp = new MovingBox(
                            new Point2D.Double(Math.random(), Math.random()), mb.getEndPos(), mb.getWidth());
                } while (staticCollision(temp.getRect()));
                samples.add(temp);
            }
            // Add samples on static obstacle corners
            for (StaticObstacle staticObstacle : staticObstacles) {
                // Bottom left
                temp = new MovingBox(
                        new Point2D.Double(staticObstacle.getRect().getMinX() - mb.getWidth(), staticObstacle.getRect().getMinY() - mb.getWidth()),
                        mb.getEndPos(), mb.getWidth());
                if (!staticCollision(temp.getRect())) {
                    samples.add(temp);
                }
                // Bottom right
                temp = new MovingBox(
                        new Point2D.Double(staticObstacle.getRect().getMaxX(), staticObstacle.getRect().getMinY() - mb.getWidth()),
                        mb.getEndPos(), mb.getWidth());
                if (!staticCollision(temp.getRect())) {
                    samples.add(temp);
                }
                // Top left
                temp = new MovingBox(
                        new Point2D.Double(staticObstacle.getRect().getMinX() - mb.getWidth(), staticObstacle.getRect().getMaxY()),
                        mb.getEndPos(), mb.getWidth());
                if (!staticCollision(temp.getRect())) {
                    samples.add(temp);
                }
                // Top right
                temp = new MovingBox(
                        new Point2D.Double(staticObstacle.getRect().getMaxX(), staticObstacle.getRect().getMinY()),
                        mb.getEndPos(), mb.getWidth());
                if (!staticCollision(temp.getRect())) {
                    samples.add(temp);
                }
            }
            // Add goal to samples
            temp = new MovingBox(mb.getEndPos(), mb.getEndPos(), mb.getWidth());
            samples.add(temp);
        } else {
            MovingObstacle mo = (MovingObstacle) box;
            // Randomise samples around the board
            for (int i = 0; i < 100; i++) {
                do {
                    temp = new MovingObstacle(
                            new Point2D.Double(Math.random(), Math.random()), mo.getWidth());
                } while (staticCollision(temp.getRect()));
                samples.add(temp);
            }
            // Add samples on static obstacle corners
            for (StaticObstacle staticObstacle : staticObstacles) {
                // Bottom left
                temp = new MovingObstacle(
                        new Point2D.Double(staticObstacle.getRect().getMinX() - mo.getWidth(),
                                staticObstacle.getRect().getMinY() - mo.getWidth()), mo.getWidth());
                if (!staticCollision(temp.getRect())) {
                    samples.add(temp);
                }
                // Bottom right
                temp = new MovingObstacle(
                        new Point2D.Double(staticObstacle.getRect().getMaxX(),
                                staticObstacle.getRect().getMinY() - mo.getWidth()), mo.getWidth());
                if (!staticCollision(temp.getRect())) {
                    samples.add(temp);
                }
                // Top left
                temp = new MovingObstacle(
                        new Point2D.Double(staticObstacle.getRect().getMinX() - mo.getWidth(),
                                staticObstacle.getRect().getMaxY()), mo.getWidth());
                if (!staticCollision(temp.getRect())) {
                    samples.add(temp);
                }
                // Top right
                temp = new MovingObstacle(
                        new Point2D.Double(staticObstacle.getRect().getMaxX(), staticObstacle.getRect().getMinY()),
                        mo.getWidth());
                if (!staticCollision(temp.getRect())) {
                    samples.add(temp);
                }
            }
        }
        return samples;
    }

    /**
     * Choose a goal node from the list of samples
     *
     * @param samples
     * @param path
     * @return
     */
    private Box chooseGoalNode(Set<Box> samples, Rectangle2D path) {
        for (Box box : samples) {
            if (!box.getRect().intersects(path)) {
                return box;
            }
        }
        return null;
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
    public void printOutput(String solutionFile, List<State> solution) {
        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(solutionFile));

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

    public Point2D findDock(Box mb, RobotConfig rc){
        Point2D center = new Point2D.Double(mb.getPos().getX() + mb.getWidth()/2, mb.getPos().getY() + mb.getWidth()/2);

        Set<Point2D> dockPos = new HashSet<>();
        dockPos.add(new Point2D.Double(center.getX() + roboWidth, center.getY()));    //right hand side ...robo width 0.01
        dockPos.add(new Point2D.Double(center.getX() - roboWidth, center.getY()));    //left hand side
        dockPos.add(new Point2D.Double(center.getX(), center.getY() + roboWidth));    //upper hand side
        dockPos.add(new Point2D.Double(center.getX(), center.getY() - roboWidth));    //under hand side

        double dist = Math.sqrt(2);
        Point2D bestDockPos = center;

        for(Point2D pos : dockPos){
            Rectangle2D proj = new Rectangle2D.Double(pos.getX(), pos.getY(), mb.getWidth(), mb.getWidth());
            if(!staticCollision(proj) && pos.distance(rc.getPos()) < dist){
                dist = pos.distance(rc.getPos());
                bestDockPos = pos;
            }
        }


        return bestDockPos;
    }

    public List<State> orientRobot(Box mb, State state){
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

    public List<State> rotateBot(double target, State state, boolean right){
        List<State> path = new ArrayList<>();
        RobotConfig robo = state.getRobo();

        double angle = robo.getOrientation()/(Math.PI/180);
        angle = getReducedDouble(angle, 8);
//        System.out.println(angle);

        if(angle == 0 && target == 360){
            path.add(new State(
                    new RobotConfig(robo.getPos(), 2*Math.PI),
                    state.getMovingBoxes(), state.getMovingObstacles()
            ));

        }else if(angle == 360 && target == 0){
            path.add(new State(
                    new RobotConfig(robo.getPos(), 0),
                    state.getMovingBoxes(), state.getMovingObstacles()
            ));

        }else if(right && angle == 360){
            angle = 0;
        }else if(!right && angle == 0){
            angle = 360;
        }

//        System.out.println("cur angle: " + angle + " target: " + target);

        double maxIncrementAngle = getReducedDouble((minStepSize / (Math.PI * (roboWidth / 2)) * 360) / 2, 8);
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
//        System.out.println(dist);

        while(step < getReducedDouble(dist, 3)){
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

        if(!updown){
            newRobo = new RobotConfig(
                    new Point2D.Double(newRobo.getPos().getX() + dir*(dist - step), newRobo.getPos().getY()), orien);
        }else{
            newRobo = new RobotConfig(
                    new Point2D.Double(newRobo.getPos().getX(), newRobo.getPos().getY() + dir*(dist - step)), orien);
        }

        path.add(new State(
                newRobo,
                state.getMovingBoxes(),
                state.getMovingObstacles()
        ));

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

    public List<State> refaceRobot(Box prev, Box next, State state){
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
        double x;
        double y;
//        System.out.println(state.getRobo().getOrientation());

        Point2D center = new Point2D.Double(prev.getPos().getX() + prev.getWidth()/2,
                prev.getPos().getY() + prev.getWidth()/2);
        Point2D nc = new Point2D.Double(next.getPos().getX() + next.getWidth()/2,
                next.getPos().getY() + next.getWidth()/2);

//        System.out.println("robot: " + cur.getPos().getX() + ", " + cur.getPos().getY());
//        System.out.println("center: " + center.getX() + ", " + center.getY() + " | " + prev.getWidth());
//        System.out.println("next: " + nc.getX() + ", " + nc.getY());

        x = cur.getPos().getX();
        y = cur.getPos().getY();

        deltaX = center.getX() - x;
        deltaY = center.getY() - y;

//        System.out.println("Delta: " + deltaX + ", " + deltaY);

        if(Math.abs(deltaX) < 0.004 && deltaX != 0){
            if(deltaX < 0){
                path.addAll(refaceRobotTransition(state, Math.abs(deltaX), -1, false));
            }
            else{
                path.addAll(refaceRobotTransition(state, Math.abs(deltaX), 1, false));
            }
            state = path.get(path.size() - 1);
        }


        if(Math.abs(deltaY) < 0.004 && deltaY != 0){
            if(deltaY < 0){
                path.addAll(refaceRobotTransition(state, Math.abs(deltaY), -1, true));
            }
            else{
                path.addAll(refaceRobotTransition(state, Math.abs(deltaY), 1, true));
            }
            state = path.get(path.size() - 1);
        }

        cur = state.getRobo();

        if(cur.getPos().getX() < center.getX()){    //left
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

        }else if(cur.getPos().getX() > center.getX()){  //right
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

        }else if(cur.getPos().getY() < center.getY()){  //down
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

        }else if(cur.getPos().getY() > center.getY()){  //up
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


        if(intFace == 0){
            method = checkRotation(state, face1, face2, center.getX(), center.getY());
//            System.out.println("method: " + method);
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
//                System.out.println("Shit");
            }

        }
        else{
//            System.out.println("Switch");
            method = checkRotation(state, face1, intFace, center.getX(), center.getY());
            method2 = checkRotation(state, intFace, face2, center.getX(), center.getY());
            if(method == 0 || method2 == 0){
                if(intFace == 2){
                    intFace = 4;
                }
                else{
                    intFace = 1;
                }
                method = checkRotation(state, face1, intFace, center.getX(), center.getY());
                method2 = checkRotation(state, intFace, face2, center.getX(), center.getY());
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

        if(x < 0 || y < 0 || x > 1 - h/2 || y > 1 - h/2){
//            System.out.println("Wall");
            return false;
        }

        Rectangle2D rect = new Rectangle2D.Double(x, y, w, h);

        if(staticCollision(rect)){
//            System.out.println("Static");
            return false;
        }

        for(MovingObstacle box : movingObstacles){
            if(box.getRect().intersects(rect)){
//                System.out.println("obstacle");
                return false;
            }
        }

        for(MovingBox box : movingboxes){
            if(box.getRect().intersects(rect)){
                System.out.println("box");
                return false;
            }
        }


        return true;
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


    public int checkRotation(State state, int face1, int face2, double X, double Y){
        // 1 Top, 2 Right, 3 Bottom, 4 Left
//        System.out.println("Face 1: " + face1 + ", Face 2: " +face2);
        double width = this.roboWidth;
        double halfWidth = width / 2;
        RobotConfig cur = state.getRobo();
        double y;
        double x;
        double centX;
        double centY;

        if(face1 == 1){
            x = X;
            y = Y + halfWidth;
        }
        else if(face1 == 2){
            x = X + halfWidth;
            y = Y;
        }
        else if(face1 == 3){
            x = X;
            y = Y - halfWidth;
        }
        else{
            x = X - halfWidth;
            y = Y;
        }
        //Top
        //Top left
        if((face1 == 1 && face2 == 4) ||(face2 == 1 && face1 == 4) ){
            x = x - halfWidth;
            if(checkRect(state, x, y, halfWidth, halfWidth)){
                y = y - halfWidth;
                x = x - halfWidth;
                if(checkRect(state, x, y, halfWidth, halfWidth)){
                    return 1;
                }
            }
            if(face1 == 1){
                if(checkSide(state, halfWidth, X, Y, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, X, Y, face2)){
                    return 3;
                }
            }
            else{
                if(checkSide(state, halfWidth, X, Y, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, X, Y, face2)){
                    return 3;
                }

            }

            return 0;
        }
        //Top right
        else if((face1 == 1 && face2 == 2)||(face2 == 1 && face1 == 2)){
            if(checkRect(state, x, y, halfWidth, halfWidth)){
                y = y - halfWidth;
                x = x + halfWidth;
                if(checkRect(state, x, y, halfWidth, halfWidth)){
                    return 1;
                }
            }
            if(face1 == 1){
                if(checkSide(state, halfWidth, X, Y, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, X, Y, face2)){
                    return 3;
                }
            }
            else{
                if(checkSide(state, halfWidth, X, Y, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, X, Y, face2)){
                    return 3;
                }

            }


            return 0;
        }
        //Bottom
        //Bottom left
        else if((face1 == 3 && face2 == 4)||(face2 == 3 && face1 ==4 )){
            y = y - halfWidth;
            x = x - halfWidth;
            if(checkRect(state, x, y, halfWidth, halfWidth)){
                y = y + halfWidth;
                x = x - halfWidth;
                if(checkRect(state, x, y, halfWidth, halfWidth)){
                    return 1;
                }
            }
            if(face1 == 3){
                if(checkSide(state, halfWidth, X, Y, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, X, Y, face2)){
                    return 3;
                }
            }
            else{
                if(checkSide(state, halfWidth, X, Y, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, X, Y, face2)){
                    return 3;
                }

            }


            return 0;
        }
        //Bottom right
        else if((face1 == 3 && face2 == 2)||(face2 == 3 && face1 == 2)){
            y = y - halfWidth;
            if(checkRect(state, x, y, halfWidth, halfWidth)){
                y = y + halfWidth;
                x = x + halfWidth;
                if(checkRect(state, x, y, halfWidth, halfWidth)){
                    return 1;
                }
            }
            if(face1 == 3){
                if(checkSide(state, halfWidth, X, Y, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, X, Y, face2)){
                    return 3;
                }
            }
            else{
                if(checkSide(state, halfWidth, X, Y, face1)){
                    return 2;
                }
                else if(checkSide(state, halfWidth, X, Y, face2)){
                    return 3;
                }

            }

            return 0;
        }
        return -1;
    }

    public void stepObjectiveSampling(String solutionFile){
        State og = new State(robo, movingBoxes, movingObstacles);

        List<State> path = new ArrayList<>();
        path.add(og);

        //horo robot
        posRoboConfig = sampleNewRobo(new RobotConfig(
                robo.getPos(), 0));

        //vert robot
        posRoboConfig.addAll(sampleNewRobo(new RobotConfig(
                robo.getPos(), Math.PI/2)));

        boolean complete = false;
        while (!complete) {
            for (MovingBox mbog : path.get(path.size() - 1).getMovingBoxes()) { //hard limit placed
                pathBox(mbog, new MovingBox(mbog.getEndPos(), mbog.getEndPos(), mbog.getWidth()),
                        sampleNewState(mbog), path);
            }
            complete = true;
            for (MovingBox mbog : path.get(path.size() - 1).getMovingBoxes()) {
                if (!mbog.getPos().equals(mbog.getEndPos())) {
                    complete = false;
                }
            }
        }

        printOutput(solutionFile, path);
    }

    /**
     * Create the path of the box and robot from box to goal
     * Updates path with new states
     *
     * @param box
     * @param goal
     * @param path
     */
    private void pathBox(Box box, Box goal, Set<Box> samples, List<State> path) {
        System.out.println("Gotta move " + ((box instanceof MovingBox) ? "MovingBox " : "MovingObstacle ") + box.toString() + " to " + goal.toString());
        Box init = box;
        List<State> offset = new ArrayList<>();

        //Find dock positions
        Point2D robDockPos = findDock(box, path.get(path.size() - 1).getRobo());
        posRoboConfig.add(new RobotConfig(robDockPos, 0));
        box.setDockPos(robDockPos);
        path.addAll(linkRobToObjective(path.get(path.size() - 1), posRoboConfig, box));

        System.out.println("Connected to box yay");
        List<Box> boxPath = findBoxPath(box, goal, samples);

        for (Box next : boxPath) {
            Box intermediate = joinNodes(init, next);
            // Path to intermediate node from initial node
            addBuildStepsToPath(path, init, intermediate);
            // Path from intermediate node to next node
            addBuildStepsToPath(path, intermediate, next);

            init = next;
        }

        //arrived at destination move bot off box to path to next box
        Point2D center = new Point2D.Double(
                goal.getPos().getX() + goal.getWidth()/2,
                goal.getPos().getY() + goal.getWidth()/2
        );

        RobotConfig cur = path.get(path.size() - 1).getRobo();
        State curState = path.get(path.size() - 1);

        if(cur.getPos().getX() < center.getX()) {    //left
            path.addAll(moveBot(new Point2D.Double(
                    center.getX() - roboWidth,
                    center.getY()
            ), curState));

        }else if(cur.getPos().getX() > center.getX()){  //right
            path.addAll(moveBot(new Point2D.Double(
                    center.getX() + roboWidth,
                    center.getY()
            ), curState));

        }else if(cur.getPos().getY() < center.getY()){  //below
            path.addAll(moveBot(new Point2D.Double(
                    center.getX(),
                    center.getY() - roboWidth
            ), curState));

        }else if(cur.getPos().getY() > center.getY()){  //top
            path.addAll(moveBot(new Point2D.Double(
                    center.getX(),
                    center.getY() + roboWidth
            ), curState));

        }
    }

    private void addBuildStepsToPath(List<State> path, Box init, Box goal) {
        Rectangle2D union = init.getRect().createUnion(goal.getRect());
            for (MovingObstacle mo : path.get(path.size() - 1).getMovingObstacles()) {
                if (mo.getRect().intersects(union) && !mo.equals(init)) {
                    System.out.println("Oh my it appears there's a MovingObstacle in our path");
                    Set<Box> nodes = sampleNewState(mo);
                    Box target = chooseGoalNode(nodes, union);
                    if (target == null) {
                        System.out.println("Could not decide upon a goal node");
                    }
                    System.out.println("Time to move it");
                    pathBox(mo, target, nodes, path);
                    System.out.println("At least I can path it haha");

                    Point2D robDockPos = findDock(init, path.get(path.size() - 1).getRobo());
                    posRoboConfig.add(new RobotConfig(robDockPos, 0));
                    init.setDockPos(robDockPos);
                    path.addAll(linkRobToObjective(path.get(path.size() - 1), posRoboConfig, init));
                    System.out.println("And I can make it back home yay");
                }
            }
            for (MovingBox mb : path.get(path.size() - 1).getMovingBoxes()) {
                if (mb.getRect().intersects(union) && !mb.equals(init)) {
                    System.out.println("Oh my it appears there's a MovingBox in our path");
                    Set<Box> nodes = sampleNewState(mb);
                    Box target = chooseGoalNode(nodes, union);
                    if (target == null) {
                        System.out.println("Could not decide upon a goal node");
                    }
                    System.out.println("Time to move it");
                    pathBox(mb, target, nodes, path);
                    System.out.println("At least I can path it haha");
                    Point2D robDockPos = findDock(init, path.get(path.size() - 1).getRobo());
                    posRoboConfig.add( new RobotConfig(robDockPos, 0));
                    init.setDockPos(robDockPos);
                    path.addAll(linkRobToObjective(path.get(path.size() - 1), posRoboConfig, init));
                    System.out.println("And I can make it back home yay");
                }
            }

        State state;
        State step;
        Box last = init;
        for(Box sStep : buildStep(init, goal)){
            state = path.get(path.size() - 1);
            path.addAll(refaceRobot(last, sStep, state));

            state = path.get(path.size() - 1);
            step = createNewState(state, sStep, last);

            path.add(step);
            last = sStep;
        }
    }

    public List<State> linkRobToObjective(State prev, Set<RobotConfig> samples, Box focus){
        List<State> path = new ArrayList<>();
        path.add(prev);

        RobotConfig cur = prev.getRobo();

//        for(RobotConfig sample : samples){
//            if(focus.getDockPos().equals(sample.getPos())){
//                System.out.println("we did it");
//            }
//        }

        List<RobotConfig> rp = pathBot(cur, samples, focus);

        RobotConfig last = cur;
        State curState;
        boolean cc;

        for(RobotConfig r : rp) {

            RobotConfig intermediate = validatePath(last.getPos(), r.getPos(), focus);

//            System.out.println("-->" +last.getPos().getX() + ", " + last.getPos().getY());
//            System.out.println(intermediate.getPos().getX() + ", " + intermediate.getPos().getY());
//            System.out.println(r.getPos().getX() + ", " + r.getPos().getY());
//
//            System.out.println("-->" + last.getOrientation());
//            System.out.println(intermediate.getOrientation());
//            System.out.println(r.getOrientation());

            cc = (last.getOrientation() < intermediate.getOrientation());

            curState =  path.get(path.size() - 1);
            path.addAll(rotateBot(intermediate.getOrientation() * (180/Math.PI), curState, cc));

            curState =  path.get(path.size() - 1);
            path.addAll(moveBot(intermediate.getPos(), curState));

            cc = (last.getOrientation() < intermediate.getOrientation());

            curState =  path.get(path.size() - 1);
            path.addAll(rotateBot(r.getOrientation() * (180/Math.PI), curState, cc));

            curState =  path.get(path.size() - 1);
            path.addAll(moveBot(r.getPos(), curState));

            last = r;
        }

        curState =  path.get(path.size() - 1);
        path.addAll(orientRobot(focus, curState));

        return path;
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
        if (start instanceof MovingBox) {
            // Path 1
            Rectangle2D intermediatePos = new Rectangle2D.Double(start.getPos().getX(),
                    goal.getPos().getY(), start.getWidth(), start.getWidth());
            if (!staticCollision(start.getRect().createUnion(intermediatePos))
                    && !staticCollision(goal.getRect().createUnion(intermediatePos))) {
                return new MovingBox(
                        new Point2D.Double(intermediatePos.getMinX(), intermediatePos.getMinY()),
                        ((MovingBox)start).getEndPos(), start.getWidth());
            }
            // Path 2
            intermediatePos = new Rectangle2D.Double(goal.getPos().getX(),
                    start.getPos().getY(), start.getWidth(), start.getWidth());
            if (!staticCollision(start.getRect().createUnion(intermediatePos))
                    && !staticCollision(goal.getRect().createUnion(intermediatePos))) {
                return new MovingBox(
                        new Point2D.Double(intermediatePos.getMinX(), intermediatePos.getMinY()),
                        ((MovingBox)start).getEndPos(), start.getWidth());
            }
        } else {
            // Path 1
            Rectangle2D intermediatePos = new Rectangle2D.Double(start.getPos().getX(),
                    goal.getPos().getY(), start.getWidth(), start.getWidth());
            if (!staticCollision(start.getRect().createUnion(intermediatePos))
                    && !staticCollision(goal.getRect().createUnion(intermediatePos))) {
                return new MovingObstacle(
                        new Point2D.Double(intermediatePos.getMinX(), intermediatePos.getMinY()), start.getWidth());
            }
            // Path 2
            intermediatePos = new Rectangle2D.Double(goal.getPos().getX(),
                    start.getPos().getY(), start.getWidth(), start.getWidth());
            if (!staticCollision(start.getRect().createUnion(intermediatePos))
                    && !staticCollision(goal.getRect().createUnion(intermediatePos))) {
                return new MovingObstacle(
                        new Point2D.Double(intermediatePos.getMinX(), intermediatePos.getMinY()), start.getWidth());
            }
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
//            System.out.println("size: " + queue.size() + "  \tcur pos: "
//                    + current.getBox().getPos().getX() + ", " + current.getBox().getPos().getY()
//                    + "  \tcur weight: " + current.getWeight()
//                    + " \theuristic: " + current.getHeuristic()
//                    + " \tsum: " + (current.getWeight() + current.getHeuristic()));

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

//        System.out.println("final: " + boxPath.get(boxPath.size() - 1).getPos().getX()
//                + ", " + boxPath.get(boxPath.size() - 1).getPos().getY());
//        System.out.println("goal: " + goal.getPos().getX() + ", " + goal.getPos().getY());
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

    public List<Box> buildStep(Box origin, Box end){

        // build path
        List<Box> path = new ArrayList<>();
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

            Box projMB;
            if (origin instanceof MovingBox) {
                projMB = new MovingBox(
                        new Point2D.Double(curX, curY), ((MovingBox) origin).getEndPos(), roboWidth);
            } else {
                projMB = new MovingObstacle(
                        new Point2D.Double(curX, curY), roboWidth);
            }


            // can check projected, if intersects with obstacle

            path.add(projMB);

        }

        return path;
    }

    /**
     * Return the RoboPos with the smallest distance from the start pos and expected path to goal
     *
     * @param queue
     * @return
     */
    public RoboPos getNextRobo(Set<RoboPos> queue) {
        RoboPos result = null;
        double minWeight = 10;
        for (RoboPos pos : queue) {
            if (result == null || (pos.getWeight() + pos.getHeuristic()) < minWeight) {
                result = pos;
                minWeight = pos.getWeight() + pos.getHeuristic();
            }
        }
        return result;
    }

    public RobotConfig validatePath(Point2D start, Point2D end, Box focus){
        //        |----
        // Path 1 |

        Point2D p = new Point2D.Double(start.getX(), end.getY());

        Line2D l1 = new Line2D.Double(start, p);
        Line2D l2 = new Line2D.Double(p, end);
        Rectangle2D intermediatePos = new Rectangle2D.Double(
                p.getX() - roboWidth/2, p.getY() - roboWidth/2, roboWidth, roboWidth);

//        System.out.println("intersects with y aixs: " + !l0.intersects(focus.getRect()));

        if (!lineIntoStatic(l1) && !lineIntoStatic(l2) && !staticCollision(intermediatePos)) {

            return new RobotConfig(p, Math.PI/2);
        }

        //            |
        // Path 2 ----|

        p = new Point2D.Double(start.getX(), end.getY());

        l1 = new Line2D.Double(start, p);
        l2 = new Line2D.Double(p, end);
        intermediatePos = new Rectangle2D.Double(
                p.getX() - roboWidth/2, p.getY() - roboWidth/2, roboWidth, roboWidth);

//        System.out.println("intersects with x aixs: " + l0.intersects(focus.getRect()));

        if (!lineIntoStatic(l1) && !lineIntoStatic(l2) && !staticCollision(intermediatePos)) {

            return new RobotConfig(p, 0);
        }

        return null;
    }


    public Set<RobotConfig> getRoboNeighbours(Set<RobotConfig> samples, RobotConfig cur, Box focus) {
        Set<RobotConfig> neighbours = new HashSet<>();

        for(RobotConfig rb : samples){
            if(validatePath(cur.getPos(), rb.getPos(), focus) != null){
                neighbours.add(rb);
            }
        }
        return neighbours;
    }

    private List<RobotConfig> pathBot(RobotConfig start, Set<RobotConfig> samples, Box focus) {
        // Initialise all the queues and sets to run the search
        Set<RoboPos> queue = new HashSet<>();
        Set<Point2D> visited = new HashSet<>();

        Point2D end = focus.getDockPos();
//        System.out.println("goal: " + end.getX() + ", " + end.getY());

        RoboPos current = new RoboPos(start, 0.0, start.getPos().distance(end), null);
        queue.add(current);

        // Continue search until there is no more options or the goal is reached
        int count = 0;
        while (!queue.isEmpty()) {

            for(RoboPos pos : queue){
                if(pos.getRobo().getPos().equals(end)){
                    System.out.println("we found goal from neighbours");
                }
//                System.out.println(pos.getRobo().getPos().getX() + ", " + pos.getRobo().getPos().getY());
            }

            current = getNextRobo(queue);
            System.out.println("size: " + queue.size() + "  \tcur pos: "
                    + current.getRobo().getPos().getX() + ", " + current.getRobo().getPos().getY()
                    + "  \tcur weight: " + current.getWeight()
                    + " \theuristic: " + current.getHeuristic()
                    + " \tsum: " + (current.getWeight() + current.getHeuristic()));

            queue.remove(current);
            visited.add(current.getRobo().getPos());

            if (end.equals(current.getRobo().getPos())) {
                break;
            }

            for (RobotConfig rb : getRoboNeighbours(samples, current.getRobo(), focus)) {
                if (visited.contains(rb.getPos()) || current.getRobo().equals(rb)) {
                    continue;
                }

//                System.out.println("-> " + rb.getPos().getX() + ", " + rb.getPos().getY());
//                System.out.println(current.getRobo().getPos().getX() + ", " + current.getRobo().getPos().getY());

                double weight = current.getWeight() +
                        + Math.abs(current.getRobo().getPos().getX() - rb.getPos().getX())
                        + Math.abs(current.getRobo().getPos().getY() - rb.getPos().getY());
                queue.add(new RoboPos(rb, weight, rb.getPos().distance(end), current));
            }

            count++;
        }

        if (end.equals(current.getRobo().getPos())) {
            System.out.println("Could not find a path to the goal");
        }

        // Create the list of boxes
        List<RobotConfig> botPath = new ArrayList<>();
        while (current != null) {
            if(!current.getRobo().equals(start)){
                botPath.add(0, current.getRobo());
            }
            current = current.getPrev();
        }

        return botPath;
    }

    private boolean abridgedContains(Set<Point2D> nodes, Point2D target){

        for(Point2D node : nodes){
//            System.out.println("x: " + node.getX() + " - " + target.getX()+ "\ty: " + node.getX() + " - " + target.getY());

            if(node.getX() == target.getX()
                && node.getY() == target.getY()){

                return  true;
            }
        }

        return false;
    }

    private List<State> moveBot(Point2D end, State origin){

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

    private boolean lineIntoStatic(Line2D path){
        for(StaticObstacle so : staticObstacles){
            if(path.intersects(so.getRect())){
                return true;
            }
        }

        return false;
    }
}
