package problem;

public class Node {

    MovingBox box;
    double weight;
    Node prev;

    public Node(MovingBox box, double weight, Node prev) {
        this.box = box;
        this.weight = weight;
        this.prev = prev;
    }

    public MovingBox getBox() {
        return box;
    }

    public Node getPrev() {
        return prev;
    }

    public double getWeight() {
        return weight;
    }

    @Override
    public boolean equals(Object o) {
        if (o instanceof Node) {
            Node other = (Node) o;
            if (this.getPrev() == null || other.getPrev() == null) {
                return this.getBox().equals(other.getBox())
                        && this.getWeight() == other.getWeight()
                        && this.getPrev() == null
                        && other.getPrev() == null;
            } else {
                return this.getBox().equals(other.getBox())
                        && this.getWeight() == other.getWeight()
                        && this.getPrev().equals(other.getPrev());
            }
        }
        return false;
    }
}
