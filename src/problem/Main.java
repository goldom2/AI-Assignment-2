package problem;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Main {

    public static void main(String[] args) throws IOException {
        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem("input2.txt");
            ps.loadSolution("solution1.txt");
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
        }

        System.out.println("Finished loading!");


        /** generate initial test space

         Assumptions:
         -the moving object is always the same width as the robot
         -the moving object is always square
         */
        genGridOverlay(ps);

    }

    private static void genGridOverlay(ProblemSpec ps) throws IOException {

        List<Box> objectiveBoxes = ps.getMovingBoxes();

        Box movingBox = objectiveBoxes.iterator().next();
        double gridDiv = movingBox.getWidth(); //assign grid division equal to the size of the moving box

        ArrayList<Point2D> grid = new ArrayList<>();

        for (int i = 0; i < (1 / gridDiv); i++) {
            for (int j = 0; j < 1 / gridDiv; j++) {
                Point2D pos = new Point2D.Double(i, j);
                grid.add(pos);
            }
        }

        List<StaticObstacle> staticObjects = ps.getStaticObstacles();
        List<Box> movingObjects = ps.getMovingObstacles();

        List<Point2D> discard = new ArrayList<>();

        for (int i = 0; i < grid.size(); i++) {
            Point2D pos = grid.get(i);

            for (int j = 0; j < staticObjects.size(); j++) {
                if (staticObjects.get(j).getRect().intersects(pos.getX() * gridDiv
                        , pos.getY() * gridDiv, gridDiv, gridDiv)) {

                    discard.add(pos);
                }
            }

            for (int j = 0; j < movingObjects.size(); j++) {
                if (movingObjects.get(j).getRect().intersects(pos.getX() * gridDiv
                        , pos.getY() * gridDiv, gridDiv, gridDiv)) {
                    discard.add(pos);
                }
            }
        }

        //revised grid
        grid.removeAll(discard);

        //link start/end nodes to objective and goal objects
        Point2D origin = ps.getInitialRobotConfig().getPos();
        System.out.println(ps.getInitialRobotConfig().getOrientation());

        //pick the first objective box
        Box objective = objectiveBoxes.get(0);
        Point2D goal = objective.getPos();

        List<RobotConfig> path = new ArrayList<>();
        path.add(ps.getInitialRobotConfig());

        RobotConfig base = path.get(path.size() - 1);
        Line2D projection = new Line2D.Double(base.getPos(), goal);

        double distance = goal.distance(base.getPos());
        double orientation = Math.atan2(goal.getY() - origin.getY(), goal.getX() - origin.getX());

        if (projection.intersects(objective.getRect())) {
            distance -= objective.getWidth();
        }

        while (distance > 0) {
            RobotConfig next = path.get(path.size() - 1);
            RobotConfig step = new RobotConfig(new Point2D.Double(
                    next.getPos().getX() + 0.001 * Math.cos(orientation),
                    next.getPos().getY() + 0.001 * Math.sin(orientation)),
                    next.getOrientation());

            distance -= 0.001;
            path.add(step);
        }

        // check for placement on the moving object

        RobotConfig here = path.get(path.size() - 1);
        if (here.getPos().getX() > objective.getPos().getX()) { //to the right

        }
        if (here.getPos().getX() < objective.getPos().getX()) { //to the left

        }
        if (here.getPos().getY() > objective.getPos().getY()) { //on top

        }
        if (here.getPos().getY() < objective.getPos().getY()) { //below

        }

        writeToOutput(path, null, null);
    }

    private static void writeToOutput(
            List<RobotConfig> roboPath, List<MovingBox> movingBoxes,
            List<MovingObstacle> movingObstacles) throws IOException {

        int length = roboPath.size();
        int currentLine = 0;

        //assume that all lists are the same size
        BufferedWriter writer = new BufferedWriter(new FileWriter("solution1.txt"));

        writer.write(Integer.toString(length));
        writer.newLine();
        while(currentLine < length) {

            writer.write(Double.toString(roboPath.get(currentLine).getPos().getX()) + " "
                    + Double.toString(roboPath.get(currentLine).getPos().getY()) + " "
                    + Double.toString(roboPath.get(currentLine).getOrientation()) + " "
                    + 0.15 + " "
                    + 0.15 + " "
                    + 0.25 + " "
                    + 0.25 + " "
                    + 0.4 + " "
                    + 0.4);
            writer.newLine();

            currentLine++;
        }
        writer.close();
    }
}