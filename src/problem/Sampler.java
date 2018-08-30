package problem;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

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
    public void objectBasedSampling(){

        List<Point2D> anchorNodes = new ArrayList<>();

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
    }

    private List<Point2D> findCornerNodes(Rectangle2D object){
        List<Point2D> currNodes = new ArrayList<>();

        //On object
        currNodes.add(new Point2D.Double(object.getMinX(),object.getMinY()));
        currNodes.add(new Point2D.Double(object.getMaxX(),object.getMinY()));
        currNodes.add(new Point2D.Double(object.getMinX(),object.getMaxY()));
        currNodes.add(new Point2D.Double(object.getMaxX(),object.getMaxY()));

        //Offset
        Point2D llc = new Point2D.Double((object.getMinX()
                - movingBoxWidth), (object.getMinY() - movingBoxWidth));
        Point2D lrc = new Point2D.Double((object.getMaxX()
                + movingBoxWidth), (object.getMinY() - movingBoxWidth));
        Point2D ulc = new Point2D.Double((object.getMinX()
                - movingBoxWidth), object.getMaxY());
        Point2D urc = new Point2D.Double(object.getMaxX(),
                object.getMaxY());

        if(checkIfLegal(llc)){
            currNodes.add(llc);
        }
        if(checkIfLegal(lrc)){
            currNodes.add(lrc);
        }
        if(checkIfLegal(ulc)){
            currNodes.add(ulc);
        }
        if(checkIfLegal(urc)){
            currNodes.add(urc);
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
        if (currNode.getX() > (1 - movingBoxWidth) || currNode.getX() < 0
                || currNode.getY() > (1 - movingBoxWidth) || currNode.getY() < 0) {
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
    private void linkNodes(List<Point2D> nodes) {
        List<Point2D> newNodes = new ArrayList<Point2D>();
        List<Edge> edges = new ArrayList<Edge>();

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
                if (checkIfLegal(x)) {
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
    public void visualiseNodes(List<Point2D> nodes) {
        String output = "";
        output += movingBoxWidth + " " + robo.getPos().getX()
                + " " + robo.getPos().getY() + " " + robo.getOrientation() + "\n";
        output += movingBoxes.size() + " " + (movingObstacles.size() + nodes.size())
                + " " + staticObstacles.size() + "\n";

        for (MovingBox box : movingBoxes) {
            output += box.getPos().getX() + " " + box.getPos().getY() + " "
                    + box.getEndPos().getX() + " " + box.getEndPos().getY() + "\n";
        }

        for (MovingObstacle ob: movingObstacles) {
            output += ob.getPos().getX() + " " + ob.getPos().getY() + " " + ob.getWidth() + "\n";
        }
        for (Point2D node : nodes) {
            output += node.getX() + " " + node.getY() + " " + "0.01\n";
        }

        for (StaticObstacle ob : staticObstacles) {
            output += ob.getRect().getMinX() + " " + ob.getRect().getMinY()
                    + " " + ob.getRect().getMaxX() + " " + ob.getRect().getMaxY() + "\n";
        }
    }
}
