package problem;

public class Node {

    MovingBox box;
    double weight;
    double heuristic;
    Node prev;

    public Node(MovingBox box, double weight, double heuristic, Node prev) {
        this.box = box;
        this.weight = weight;
        this.heuristic = heuristic;
        this.prev = prev;
    }

    public MovingBox getBox() {
        return box;
    }

    public Node getPrev() {
        return prev;
    }

    public Double getHeuristic() {
        return heuristic;
    }

    public double getWeight() {
        return weight;
    }

    @Override
    public boolean equals(Object o) {
        if (o instanceof Node) {
            Node other = (Node) o;
            return this.getBox().getPos().equals(other.getBox().getPos());
//            return this.getBox().equals(other.getBox())
//                    && this.getWeight() == other.getWeight()
//                    && (this.getPrev() == null
//                    ? other.getPrev() == null : this.getPrev().equals(other.getPrev()));
//            }
        }
        return false;
    }
}
