package com.github.sdnwiselab.sdnwise;

import org.graphstream.algorithm.Dijkstra;
import org.graphstream.algorithm.flow.FordFulkersonAlgorithm;
import org.graphstream.graph.*;
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
    private static final HashMap<Node, Node> nodesToClusterNode = new HashMap<>();
    private static final HashMap<Node, HashSet<Node>> clusterNodesToBoarderNodes = new HashMap<>();

    public static void main(String[] args) {
        System.setProperty("org.graphstream.ui", "swing");
        SingleGraph networkGraph = new SingleGraph("Network Graph");
        updateVisualGraph(networkGraph);
        SingleGraph clusteringGraph = new SingleGraph("Clustering Graph");
        updateVisualGraph(clusteringGraph);
        for (Map.Entry<Integer, ArrayList<Integer>> entry : BASE_ADJACENCY_LIST.entrySet()) {
            Integer value = entry.getKey();
            ArrayList<Integer> adjacencyList = entry.getValue();
            Node currentNode = updateOrCreateNode(networkGraph, String.valueOf(value));
            for (Integer neighbor : adjacencyList) {
                Node neighborNode = updateOrCreateNode(networkGraph, String.valueOf(neighbor));
                updateOrCreateEdge(networkGraph, currentNode, neighborNode, 1, 1);
            }
        }

        // Run dijkstra on network graph
        dijkstra = new Dijkstra(Dijkstra.Element.EDGE, null, "length");
        dijkstra.init(networkGraph);
        partitionIntoTwoClusters(
                updateOrCreateNode(networkGraph, String.valueOf(0)),
                updateOrCreateNode(networkGraph, String.valueOf(6)),
                clusteringGraph,
                networkGraph
        );
        System.out.println("CTRL: boarder nodes are " + clusterNodesToBoarderNodes);
        System.out.println(nodesToClusterNode);
//        makeCluster(clusteringGraph, networkGraph, 4);
    }

//        public static void makeCluster(Graph clusteringGraph, Graph networkGraph, int clusterNumber) {
//        int nodeNumber = networkGraph.getNodeCount();
//        Random random = new Random();
//        HashSet<Node> headNodes = new HashSet<>();
//        while (headNodes.size() < clusterNumber) {
//            Node candidate = networkGraph.getNode(random.nextInt(nodeNumber));
//            boolean isNeighbor = headNodes.stream().anyMatch(head -> head.hasEdgeBetween(candidate));
//            if (!isNeighbor) {
//                headNodes.add(candidate);
//            }
//        }
//        HashMap<String, HashSet<String>> clustering = new HashMap<>();
//        Node last = null;
//        for (Node s : headNodes) {
//            dijkstra.clear();
//            dijkstra.init(networkGraph);
//            dijkstra.setSource(s);
//            dijkstra.compute();
//            HashSet<String> prev = new HashSet<>();
//            System.out.println(s.getId());
//            System.out.println(headNodes);
//            for (Node t : headNodes) {
//                if (t.getId().equals(s.getId()) || clustering.containsKey(t.getId())) continue;
//                clusteringGraph.clear();
//                HashSet<String> clusterS = partitionIntoTwoClusters(s, t, clusteringGraph, networkGraph);
//                if (prev.isEmpty()) prev = clusterS;
//                else prev.retainAll(clusterS);
//            }
//            for (String nodeId : prev) {
//                networkGraph.removeNode(nodeId);
//            }
//            clustering.put(s.getId(), prev);
//            System.out.println("Final Cluster of " + s.getId() + ": " + prev);
//            last = s;
//        }
//        HashSet<String> nodeIds = new HashSet<>();
//        assert last != null;
//        nodeIds.add(last.getId());
//        for (Node n : networkGraph.getEachNode())
//            nodeIds.add(n.getId());
//        clustering.put(last.getId(), nodeIds);
//        System.out.println("Final Cluster of " + last.getId() + ": " + nodeIds);
//        }
//    }

    public static void partitionIntoTwoClusters(
            Node source, Node sink, Graph clusteringGraph, Graph networkGraph
    ) {
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
        System.out.println("CTRL: Running Min-Cut Algorithm on the clustering graph...");
        FordFulkersonAlgorithm fd = new FordFulkersonAlgorithm();
        fd.setCapacityAttribute("capacity");
        fd.init(clusteringGraph, clusteringSource.getId(), clusteringSink.getId());
        fd.compute();
        System.out.println("CTRL: Ford Fulkerson Algorithm completed with max flow " + fd.getMaximumFlow() + ".");
        // Find cut edges in order to find out which nodes are border nodes.
        Set<Node> sourceCluster = bfs(fd, clusteringSource);
        System.out.println("CTRL: Cluster graph of node "+ source.getId() +" is " + sourceCluster);
        for (Node node : sourceCluster) {
            Node actualNode = updateOrCreateNode(networkGraph, node.getId().split("_")[0]);
            nodesToClusterNode.put(actualNode, source);
            for (Edge edge : node.getEachEdge()) {
                Node neighbor = edge.getOpposite(node);
                if (!sourceCluster.contains(neighbor)){
                    HashSet<Node> boarderNodes = clusterNodesToBoarderNodes.getOrDefault(source, new HashSet<>());
                    boarderNodes.add(actualNode);
                    clusterNodesToBoarderNodes.put(source, boarderNodes);
                }
            }
        }
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
