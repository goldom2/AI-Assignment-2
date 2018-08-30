package problem;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Sampler {

    private RobotConfig robo;
    private List<MovingBox> movingBoxes;
    private List<StaticObstacle> staticObstacles;
    private List<MovingObstacle> movingObstacles;
    private double movingBoxWidth;

    public Sampler(RobotConfig robo,
                   List<MovingBox> movingBoxes,
                   List<StaticObstacle> staticObstacles,
                   List<MovingObstacle> movingObstacles){

        this.robo = robo;
        this.movingBoxes = movingBoxes;
        this.staticObstacles = staticObstacles;
        this.movingObstacles = movingObstacles;

        //Assume there is always at least one moving box
        this.movingBoxWidth = movingBoxes.get(0).getWidth();

    }

    /**
     * The goal of this sampling strategy is to isolate obstacles my placing nodes
     * on the edges of illegal space and creating a dispersed network around these anchor nodes
     */
    public void objectBasedSampling() throws IOException {

        Set<Point2D> anchorNodes = new HashSet<>();

        //add the starting point of the robo
        anchorNodes.add(robo.getPos());

        //add nodes for static obstacles
        if(staticObstacles.size() > 0){
            for(StaticObstacle so : staticObstacles){
                anchorNodes.addAll(findCornerNodes(so.getRect()));
            }
        }

        //add nodes for movingBoxes
        if(movingBoxes.size() > 0){
            for(MovingBox mb : movingBoxes){

                Rectangle2D endRect = new Rectangle2D.Double(mb.getEndPos().getX(),
                        mb.getEndPos().getY(),
                        mb.getWidth(), mb.getWidth());

                anchorNodes.addAll(findCornerNodes((Rectangle2D) endRect.clone()));
                anchorNodes.addAll(findCornerNodes(mb.getRect()));
            }
        }

        //add nodes for MovingObstacles
        if(movingObstacles.size() > 0){
            for(MovingObstacle mo : movingObstacles){
                anchorNodes.addAll(findCornerNodes(mo.getRect()));
            }
        }

        anchorNodes.add(robo.getPos());
        visualiseNodes(linkNodes(anchorNodes));
    }

    private Set<Point2D> findCornerNodes(Rectangle2D object){
        Set<Point2D> currNodes = new HashSet<>();

        //On object
        currNodes.add(new Point2D.Double(object.getMinX(),object.getMinY()));
        currNodes.add(new Point2D.Double(object.getMaxX(),object.getMinY()));
        currNodes.add(new Point2D.Double(object.getMinX(),object.getMaxY()));
        currNodes.add(new Point2D.Double(object.getMaxX(),object.getMaxY()));

        //Offset
        Point2D llc = new Point2D.Double((object.getMinX()
                - movingBoxWidth), (object.getMinY() - movingBoxWidth));
        Point2D lrc = new Point2D.Double(object.getMaxX(),
                (object.getMinY() - movingBoxWidth));
        Point2D ulc = new Point2D.Double((object.getMinX()
                - movingBoxWidth), object.getMaxY());

        if(checkIfLegal(llc)){
            currNodes.add(llc);
        }
        if(checkIfLegal(lrc)){
            currNodes.add(lrc);
        }
        if(checkIfLegal(ulc)){
            currNodes.add(ulc);
        }

        return currNodes;
    }

    /**
     * Check the proposed node against edges (walls) and placement under static objects
     *
     * @param currNode
     * @return
     */
    private boolean checkIfLegal(Point2D currNode) {
        if (currNode.getX() > (1 - movingBoxWidth*1.5)
                || currNode.getX() < (0 + movingBoxWidth/2)
                || currNode.getY() > (1 - movingBoxWidth*1.5)
                || currNode.getY() < (0 + movingBoxWidth/2)){
            return false;
        }
        for (StaticObstacle staticObstacle : staticObstacles) {
            if (staticObstacle.getRect().contains(currNode)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Create nodes linking each current node in vertical and horizontal lines.
     * Return a list of edges and nodes.
     *
     *
     */
    private Set<Point2D> linkNodes(Set<Point2D> nodes) {
        Set<Point2D> newNodes = new HashSet<>();
        Set<Edge> edges = new HashSet<>();

        for (Point2D i : nodes) {
            for (Point2D j : nodes) {
                if (i.equals(j)) {
                    continue;
                }

                // Add new nodes joining the two existing nodes using horizontal and vertical lines
                // Create lines between nodes to test for edges
                Edge e;
                Point2D x = new Point2D.Double(i.getX(), j.getY());
                if (checkIfLegal(x)) {
                    newNodes.add(x);
                    e = testEdge(x, i);
                    if (e != null) {
                        edges.add(e);
                    }
                    e = testEdge(x, j);
                    if (e != null) {
                        edges.add(e);
                    }
                }
                Point2D y = new Point2D.Double(j.getX(), i.getY());
                if (checkIfLegal(y)) {
                    newNodes.add(y);
                    e = testEdge(y, i);
                    if (e != null) {
                        edges.add(e);
                    }
                    e = testEdge(y, j);
                    if (e != null) {
                        edges.add(e);
                    }
                }
            }
        }
        newNodes.addAll(nodes);
        return newNodes;
    }

    /**
     *
     * @param x
     * @param y
     * @return the Edge or NULL if it collides
     */
    private Edge testEdge(Point2D x, Point2D y) {
        Edge e = new Edge(x, y);
        for (StaticObstacle o : staticObstacles) {
            if (o.getRect().intersectsLine(e)) {
                return null;
            }
        }
        return e;
    }

    /**
     * Debugging function to draw nodes as movable objects on the visualiser
     */
    public void visualiseNodes(Set<Point2D> nodes) throws IOException {
        //assume that all lists are the same size
        BufferedWriter writer = new BufferedWriter(new FileWriter("solution1.txt"));

        writer.write(movingBoxWidth + " " + robo.getPos().getX()
                + " " + robo.getPos().getY() + " " + robo.getOrientation());
        writer.newLine();
        writer.write(movingBoxes.size() + " " + (movingObstacles.size() + nodes.size())
                + " " + staticObstacles.size());
        writer.newLine();

        for (MovingBox box : movingBoxes) {
            writer.write((box.getPos().getX() + movingBoxWidth / 2) + " "
                    + (box.getPos().getY() + movingBoxWidth / 2) + " "
                    + (box.getEndPos().getX() + movingBoxWidth / 2) + " "
                    + (box.getEndPos().getY() + movingBoxWidth / 2));
            writer.newLine();
        }

        for (MovingObstacle ob: movingObstacles) {
            writer.write((ob.getPos().getX() + ob.getWidth() / 2) + " "
                    + (ob.getPos().getY() + ob.getWidth() / 2) + " "
                    + ob.getWidth());
            writer.newLine();
        }
        for (Point2D node : nodes) {
            writer.write(node.getX() + " " + node.getY() + " " + "0.01");
            writer.newLine();
        }

        for (StaticObstacle ob : staticObstacles) {
            writer.write(ob.getRect().getMinX() + " " + ob.getRect().getMinY()
                    + " " + ob.getRect().getMaxX() + " " + ob.getRect().getMaxY());
            writer.newLine();
        }

        writer.close();
    }
}
