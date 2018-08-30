package problem;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

public class Edge extends Line2D.Double {


    public Edge(Point2D start, Point2D end) {
        super(start, end);
    }

//    public boolean equals(Object obj) {
//        if (obj instanceof Edge) {
//            Edge other = (Edge) obj;
//            if (other.getP1()().equals(this.getStart())
//                    && other.getEnd().equals(this.getEnd())
//                    || other.getStart().equals(this.getEnd())
//                    && other.getEnd().equals(this.getStart())) {
//                return true;
//            }
//        }
//        return false;
//    }

}
