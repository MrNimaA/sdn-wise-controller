package com.github.sdnwiselab.sdnwise;

import com.github.sdnwiselab.sdnwise.utils.ClusteringDijkstra;
import org.graphstream.algorithm.Dijkstra;
import org.graphstream.algorithm.flow.FordFulkersonAlgorithm;
import org.graphstream.graph.*;
import org.graphstream.graph.implementations.Graphs;
import org.graphstream.graph.implementations.SingleGraph;

import java.util.*;

public class MakeCluster {
    private static final HashMap<Integer, ArrayList<Integer>> BASE_ADJACENCY_LIST = new HashMap<Integer, ArrayList<Integer>>() {{
        put(0, new ArrayList<>(Arrays.asList(1, 2)));
        put(1, new ArrayList<>(Arrays.asList(2, 3)));
        put(2, new ArrayList<>(Arrays.asList(4)));
        put(3, new ArrayList<>(Arrays.asList(5)));
        put(4, new ArrayList<>(Arrays.asList(5, 6)));
        put(5, new ArrayList<>(Arrays.asList(6)));
    }};
    private static Dijkstra dijkstra;
    private static final HashMap<String, String> nodesToClusterNode = new HashMap<>();
    private static final HashMap<String, HashSet<String>> clusterNodesToBoarderNodes = new HashMap<>();

    public static void main(String[] args) {
        System.setProperty("org.graphstream.ui", "swing");
        Graph networkGraph = createGraphFromAdjList("Network Graph");
        SingleGraph clusteringGraph = new SingleGraph("Clustering Graph");
        updateVisualGraph(clusteringGraph);
        // This is a Toff to have a good visualization of the network.
        Graph copyNetworkGraph = Graphs.clone(networkGraph);
        updateVisualGraph(copyNetworkGraph);

        // Initialize dijkstra on network graph.
        dijkstra = new Dijkstra(Dijkstra.Element.EDGE, null, "length");
        dijkstra.init(networkGraph);
        makeCluster(
                clusteringGraph,
                networkGraph,
                getHeadNodes(networkGraph, new String[]{String.valueOf(0), String.valueOf(6)}, null)
        );

        Node source = updateOrCreateNode(copyNetworkGraph, String.valueOf(0));
        Node target = updateOrCreateNode(copyNetworkGraph, String.valueOf(6));

        // Remove Node 4 from cluster 6 boarder nodes. So the final path should be [0, 1, 3, 5, 6] besides the main dijkstra.
        clusterNodesToBoarderNodes.get("6").remove("4");
        System.out.println("CTRL: boarder nodes are : " + clusterNodesToBoarderNodes);
        System.out.println("CTRL: nodes to cluster node are : " + nodesToClusterNode);

        ClusteringDijkstra clusteringDijkstra = new ClusteringDijkstra(
                Dijkstra.Element.EDGE,
                null,
                "length",
                nodesToClusterNode,
                clusterNodesToBoarderNodes
        );
        clusteringDijkstra.init(copyNetworkGraph);
        clusteringDijkstra.setSource(source);
        clusteringDijkstra.compute();
        LinkedList<Node> path = new LinkedList<>();
        for (Node node : clusteringDijkstra.getPathNodes(target)) {
            path.push(node);
        }
        System.out.println("CTRL: path from " + source + " to " + target + " is " + path);
    }

    public static Graph createGraphFromAdjList(String graphId) {
        SingleGraph networkGraph = new SingleGraph(graphId);
        for (Map.Entry<Integer, ArrayList<Integer>> entry : BASE_ADJACENCY_LIST.entrySet()) {
            Integer value = entry.getKey();
            ArrayList<Integer> adjacencyList = entry.getValue();
            Node currentNode = updateOrCreateNode(networkGraph, String.valueOf(value));
            for (Integer neighbor : adjacencyList) {
                Node neighborNode = updateOrCreateNode(networkGraph, String.valueOf(neighbor));
                updateOrCreateEdge(networkGraph, currentNode, neighborNode, 1, 1);
            }
        }
        return networkGraph;
    }

    public static HashSet<Node> getHeadNodes(Graph networkGraph, String[] initialHeadNodes, Integer clusterCount) {
        /*
          Return a predefined set of Head Nodes if the initialHeadNodes in provided, else return a random set of
          head nodes.
         */
        if (initialHeadNodes.length > 0) {
            HashSet<Node> headNodes = new HashSet<>();
            for (String headNode : initialHeadNodes) {
                headNodes.add(updateOrCreateNode(networkGraph, headNode));
            }
            System.out.println("Predefined Head Nodes : " + headNodes);
            return headNodes;
        }
        return getRandomHeadNodes(networkGraph, clusterCount);
    }

    private static HashSet<Node> getRandomHeadNodes(Graph networkGraph, Integer clusterCount) {
        /*
           Return a set of random nodes as head nodes.
         */
        HashSet<Node> headNodes = new HashSet<>();
        int totalNodeCount = networkGraph.getNodeCount();
        while (headNodes.size() < clusterCount) {
            Node candidate = networkGraph.getNode(new Random().nextInt(totalNodeCount));
            boolean isNeighbor = headNodes.stream().anyMatch(head -> head.hasEdgeBetween(candidate));
            if (!isNeighbor) {
                headNodes.add(candidate);
            }
        }
        System.out.println("Random Head Nodes: " + headNodes);
        return headNodes;
    }

    public static void makeCluster(Graph clusteringGraph, Graph networkGraph, HashSet<Node> headNodes) {
        HashMap<Node, HashSet<Node>> clustering = new HashMap<>();

        // Create clustering graph for all nodes except last head node.
        Node lastHeadNode = headNodes.stream().skip(new Random().nextInt(headNodes.size())).findFirst().orElse(null);
        for (Node source : headNodes) {
            if (source.equals(lastHeadNode))
                continue;
            dijkstra.clear();
            dijkstra.init(networkGraph);
            dijkstra.setSource(source);
            dijkstra.compute();
            HashSet<Node> prev = new HashSet<>();
            for (Node sink : headNodes) {
                if (sink.equals(source) || clustering.containsKey(sink))
                    continue;
                clusteringGraph.clear();
                HashSet<Node> sourceCluster = partitionIntoTwoClusters(source, sink, clusteringGraph, networkGraph);
                if (prev.isEmpty())
                    prev = sourceCluster;
                else
                    prev.retainAll(sourceCluster);
            }
            for (Node node : prev) {
                networkGraph.removeNode(node);
            }
            clustering.put(source, prev);
            System.out.println("Final Cluster of " + source + ": " + prev);
        }

        // Create border nodes for the last head node.
        dijkstra.clear();
        dijkstra.init(networkGraph);
        dijkstra.setSource(lastHeadNode);
        dijkstra.compute();

        HashSet<Node> nodes = new HashSet<>();
        Queue<Node> queue = new LinkedList<>();
        queue.add(lastHeadNode);
        nodes.add(lastHeadNode);
        while (!queue.isEmpty()) {
            Node node = queue.poll();
            nodesToClusterNode.put(node.getId(), lastHeadNode.getId());
            boolean hadAddedToNodes = false;
            for (Edge e : node.getEdgeSet()) {
                Node neighbor = e.getOpposite(node);
                if (nodes.contains(neighbor))
                    continue;
                queue.add(neighbor);
                nodes.add(neighbor);
                hadAddedToNodes = true;
            }
            if (!hadAddedToNodes) {
                HashSet<String> boarderNodes = clusterNodesToBoarderNodes.getOrDefault(lastHeadNode.getId(), new HashSet<>());
                boarderNodes.add(node.getId());
                clusterNodesToBoarderNodes.put(lastHeadNode.getId(), boarderNodes);
            }
        }
        clustering.put(lastHeadNode, nodes);
        System.out.println("Final Cluster of " + lastHeadNode + ": " + nodes);
    }

    public static HashSet<Node> partitionIntoTwoClusters(
            Node source, Node sink, Graph clusteringGraph, Graph networkGraph
    ) {
        /*
           Partition the networkGraph into two clusters with Sink and Source nodes.
         */
        clusteringGraph.clear();
        // Run Dijkstra from source before we start exploring its neighbor nodes.
        dijkstra.setSource(source);
        dijkstra.compute();
        // Create source and sink nodes for the clustering graph.
        Node clusteringSource = updateOrCreateNode(clusteringGraph, source.getId());
        Node clusteringSink = updateOrCreateNode(clusteringGraph, sink.getId());

        for (Node currentNode : networkGraph.getEachNode()) {
            if (currentNode == source || currentNode == sink)
                continue;
            // Create or update the source and sink related nodes for clustering graph.
            Node currentNodeSource = updateOrCreateNode(clusteringGraph, currentNode.getId() + "_s");
            Node currentNodeSink = updateOrCreateNode(clusteringGraph, currentNode.getId() + "_t");
            updateOrCreateEdge(clusteringGraph, currentNodeSource, currentNodeSink, 1, 1);
            // Connect the created new nodes to the according proper nodes in clustering graph.
            for (Edge e : currentNode.getEachEdge()) {
                Node neighbor = e.getOpposite(currentNode);
                if (neighbor == sink) {
                    updateOrCreateEdge(clusteringGraph, currentNodeSink, clusteringSink, 1, networkGraph.getNodeCount());
                } else if (neighbor == source) {
                    updateOrCreateEdge(clusteringGraph, clusteringSource, currentNodeSource, 1, networkGraph.getNodeCount());
                } else {
                    if (clusteringGraph.getNode(neighbor.getId() + "_s") != null && clusteringGraph.getNode(neighbor.getId() + "_t") != null){

                        Node neighborNodeSource = updateOrCreateNode(clusteringGraph, neighbor.getId() + "_s");
                        Node neighborNodeSink = updateOrCreateNode(clusteringGraph, neighbor.getId() + "_t");

                        if (dijkstra.getPathLength(currentNode) > dijkstra.getPathLength(neighbor)) {
                            updateOrCreateEdge(clusteringGraph, neighborNodeSink, currentNodeSource, 1, networkGraph.getNodeCount());
                        } else if (dijkstra.getPathLength(currentNode) < dijkstra.getPathLength(neighbor)) {
                            updateOrCreateEdge(clusteringGraph, currentNodeSink, neighborNodeSource, 1, networkGraph.getNodeCount());
                        } else {
                            updateOrCreateEdge(clusteringGraph, currentNodeSource, neighborNodeSource, 1, networkGraph.getNodeCount());
                        }
                    }
                }
            }
        }

        // Run the Min-Cut algorithm to split the graph into two clusters.
        FordFulkersonAlgorithm fd = new FordFulkersonAlgorithm();
        fd.setCapacityAttribute("capacity");
        fd.init(clusteringGraph, clusteringSource.getId(), clusteringSink.getId());
        fd.compute();

        // Find cut edges in order to find out which nodes are border nodes.
        Set<Node> sourceCluster = bfs(fd, clusteringSource);
        HashSet<Node> cluster = new HashSet<>();
        for (Node node : sourceCluster) {
            Node actualNode = updateOrCreateNode(networkGraph, node.getId().split("_")[0]);
            cluster.add(actualNode);
            nodesToClusterNode.put(actualNode.getId(), source.getId());
            for (Edge edge : node.getEachEdge()) {
                Node neighbor = edge.getOpposite(node);
                if (!sourceCluster.contains(neighbor)){
                    HashSet<String> boarderNodes = clusterNodesToBoarderNodes.getOrDefault(source.getId(), new HashSet<>());
                    boarderNodes.add(actualNode.getId());
                    clusterNodesToBoarderNodes.put(source.getId(), boarderNodes);
                }
            }
        }
        return cluster;
    }

    private static Set<Node> bfs(FordFulkersonAlgorithm fd, Node source) {
        Set<Node> visited = new HashSet<>();
        Queue<Node> queue = new LinkedList<>();
        queue.add(source);
        visited.add(source);
        while (!queue.isEmpty()) {
            Node node = queue.poll();
            for (Edge e : node.getEachLeavingEdge()) {
                Node neighbor = e.getOpposite(node);
                if (visited.contains(neighbor))
                    continue;
                visited.add(neighbor);
                double capacity = fd.getCapacity(node, neighbor);
                double flow = fd.getFlow(node, neighbor);
                // Only consider edges that have remaining residual flow.
                if (capacity - flow > 0) {
                    queue.add(neighbor);
                }
            }
        }
        return visited;
    }

    private static void updateVisualGraph(Graph graph) {
        graph.addAttribute("ui.quality");
        graph.addAttribute("ui.antialias");
        graph.display();
    }

    private static Edge updateOrCreateEdge(Graph networkGraph, Node sourceNode, Node targetNode, int length, int capacity) {
        Edge e = networkGraph.getEdge(sourceNode.getId() + "-" + targetNode.getId());
        if (e == null) {
            e = networkGraph.addEdge(sourceNode.getId() + "-" + targetNode.getId(), sourceNode, targetNode);
        }
        Integer[] capacities = {capacity, capacity};
        e.setAttribute("length", length);
        e.setAttribute("capacity", (Object[]) capacities);
        return e;
    }

    private static Node updateOrCreateNode(Graph networkGraph, String value) {
        Node currentNode = networkGraph.getNode(value);
        if (currentNode == null) {
            currentNode = networkGraph.addNode(value);
            currentNode.setAttribute("ui.label", value);
            currentNode.setAttribute(
                    "ui.style",
                    "size: 20px, 20px; " +
                            "fill-color: rgb(43,128,123),rgb(3,82,77); " +
                            "fill-mode:gradient-diagonal2; " +
                            "shadow-mode:gradient-radial; " +
                            "shadow-color: gray,rgba(255,255,255,0); " +
                            "text-background-mode:rounded-box; " +
                            "text-style:italic; " +
                            "text-alignment:under; " +
                            "text-padding:2; " +
                            "text-offset:0,5;"
            );
        }
        return currentNode;
    }
}
