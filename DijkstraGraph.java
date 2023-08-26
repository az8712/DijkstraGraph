                                        // --== CS400 File Header Information ==--
// Name: <your full name>
// Email: <your @wisc.edu email address>
// Group and Team: <your group name: two letters, and team color>
// Group TA: <name of your group's ta>
// Lecturer: <name of your lecturer>
// Notes to Grader: <optional extra notes>

import java.util.PriorityQueue;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Hashtable;
import java.util.List;
import java.util.LinkedList;
import java.util.NoSuchElementException;
import org.junit.Before;
import org.junit.Test;
/**
 * This class extends the BaseGraph data structure with additional methods for
 * computing the total cost and list of node data along the shortest path
 * connecting a provided starting to ending nodes.  This class makes use of
 * Dijkstra's shortest path algorithm.
 */
public class DijkstraGraph<NodeType, EdgeType extends Number>
    extends BaseGraph<NodeType,EdgeType>
    implements GraphADT<NodeType, EdgeType> {

    /**
     * While searching for the shortest path between two nodes, a SearchNode
     * contains data about one specific path between the start node and another
     * node in the graph.  The final node in this path is stored in it's node
     * field.  The total cost of this path is stored in its cost field.  And the
     * predecessor SearchNode within this path is referened by the predecessor
     * field (this field is null within the SearchNode containing the starting
     * node in it's node field).
     *
     * SearchNodes are Comparable and are sorted by cost so that the lowest cost
     * SearchNode has the highest priority within a java.util.PriorityQueue.
     */
    protected class SearchNode implements Comparable<SearchNode> {
        public Node node;
        public double cost;
        public SearchNode predecessor;
        public SearchNode(Node node, double cost, SearchNode predecessor) {
            this.node = node;
            this.cost = cost;
            this.predecessor = predecessor;
        }
        public int compareTo(SearchNode other) {
            if( cost > other.cost ) return +1;
            if( cost < other.cost ) return -1;
            return 0;
        }
    }

    /**
     * This helper method creates a network of SearchNodes while computing the
     * shortest path between the provided start and end locations.  The
     * SearchNode that is returned by this method is represents the end of the
     * shortest path that is found: it's cost is the cost of that shortest path,
     * and the nodes linked together through predecessor references represent
     * all of the nodes along that shortest path (ordered from end to start).
     *
     * @param start the data item in the starting node for the path
     * @param end the data item in the destination node for the path
     * @return SearchNode for the final end node within the shortest path
     * @throws NoSuchElementException when no path from start to end is found
     *         or when either start or end data do not correspond to a graph node
     */
    protected SearchNode computeShortestPath(NodeType start, NodeType end) {
        // TODO: implement in step 6
        Node startNode = nodes.get(start);
        Node endNode = nodes.get(end);

        if (startNode == null || endNode == null) {
            throw new NoSuchElementException("cannot find shortest path between null nodes");
        }

        Hashtable<Node, SearchNode> visited = new Hashtable<>();

        PriorityQueue<SearchNode> pq = new PriorityQueue<>();

        pq.add(new SearchNode(startNode, 0, null));

        while (!pq.isEmpty()) {
            SearchNode c = pq.remove();
            if (c.node.data.equals(end)) {
                return c;
            }

            if (!visited.containsKey(c.node)) {
                visited.put(c.node, c);
                List<Edge> edges = c.node.edgesLeaving;

                for (int i = 0; i < edges.size(); i++) {
                    if (!visited.containsKey(edges.get(i).successor)) {
                        pq.add(new SearchNode(edges.get(i).successor, c.cost + edges.get(i).data.doubleValue(), c));
                    }
                }
            }
        }
        throw new NoSuchElementException("No path");
    }

    /**
     * Returns the list of data values from nodes along the shortest path
     * from the node with the provided start value through the node with the
     * provided end value.  This list of data values starts with the start
     * value, ends with the end value, and contains intermediary values in the
     * order they are encountered while traversing this shorteset path.  This
     * method uses Dijkstra's shortest path algorithm to find this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end the data item in the destination node for the path
     * @return list of data item from node along this shortest path
     */
    public List<NodeType> shortestPathData(NodeType start, NodeType end) {
        // TODO: implement in step 7
        List<NodeType> reversedPath = new LinkedList<>();
        SearchNode c = computeShortestPath(start, end);
        while (c != null) {
            reversedPath.add(c.node.data);
            c = c.predecessor;
        }
        
        List<NodeType> output = new LinkedList<>();
        for (int i = reversedPath.size() - 1; i >= 0; i--) {
            output.add(reversedPath.get(i));
        }
       

        return output;
    }

    /**
     * Returns the cost of the path (sum over edge weights) of the shortest
     * path freom the node containing the start data to the node containing the
     * end data.  This method uses Dijkstra's shortest path algorithm to find
     * this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end the data item in the destination node for the path
     * @return the cost of the shortest path between these nodes
     */
    public double shortestPathCost(NodeType start, NodeType end) {
        // TODO: implement in step 7
        SearchNode c = computeShortestPath(start, end);
        return c.cost;
    }

    // TODO: implement 3+ tests in step 8.
    public class DijkstraGraphTests {
        DijkstraGraph<String, Integer> graph;
        @Before
        public void testSetup() {
            graph = new DijkstraGraph<>();
            graph.insertNode("A");
            graph.insertNode("B");
            graph.insertNode("C");
            graph.insertNode("D");
            graph.insertNode("E");
            graph.insertNode("F");
            graph.insertNode("G");
            graph.insertNode("H");
            graph.insertEdge("A", "B", 4);
            graph.insertEdge("A", "E", 15);
            graph.insertEdge("A", "C", 2);
            graph.insertEdge("B", "E", 10);
            graph.insertEdge("B", "D", 1);
            graph.insertEdge("C", "D", 5);
            graph.insertEdge("D", "E", 3);
            graph.insertEdge("D", "F", 0);
            graph.insertEdge("F", "D", 2);
            graph.insertEdge("F", "H", 4);
            graph.insertEdge("G", "H", 4);
        }

        @Test
        public void test1() {
            double cost = graph.shortestPathCost("A", "D");
            assertEquals(5.0, cost, .01);
        }

        @Test
        public void test2() {
            double cost = graph.shortestPathCost("A", "E");
            assertEquals(8.0, cost, .01);
            List<String> path = graph.shortestPathData("A", "E");
            assertTrue(path.contains("A"));
            assertTrue(path.contains("B"));
            assertTrue(path.contains("D"));
            assertTrue(path.contains("E"));
        }

        @Test(expected = NoSuchElementException.class)
        public void test3() {
            DijkstraGraph<String, Integer> testNoEdge = new DijkstraGraph<>();
            testNoEdge.insertNode("A");
            testNoEdge.insertNode("B");
            double cost = testNoEdge.shortestPathCost("A", "B");
        }
    

    }
    
}