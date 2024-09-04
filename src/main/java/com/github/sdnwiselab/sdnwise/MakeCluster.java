package com.github.sdnwiselab.sdnwise;

import org.graphstream.algorithm.Dijkstra;
import org.graphstream.algorithm.flow.FordFulkersonAlgorithm;
import org.graphstream.graph.*;
import org.graphstream.graph.implementations.MultiGraph;
import org.graphstream.graph.implementations.SingleGraph;

import java.util.*;

public class MakeCluster {
    private static final HashMap<Integer, ArrayList<Integer>> BASE_ADJACENCY_LIST = new HashMap<Integer, ArrayList<Integer>>() {{
        put(0, new ArrayList<>(Arrays.asList(1, 2)));
        put(1, new ArrayList<>(Arrays.asList(3)));
        put(2, new ArrayList<>(Arrays.asList(4)));
        put(3, new ArrayList<>(Arrays.asList(5)));
        put(4, new ArrayList<>(Arrays.asList(5, 6)));
        put(5, new ArrayList<>(Arrays.asList(6)));
    }};
    private static Dijkstra dijkstra;

    public static void main(String[] args) {
        System.setProperty("org.graphstream.ui", "swing");
        SingleGraph networkGraph = new SingleGraph("Network Graph");
        updateVisualGraph(networkGraph);
        MultiGraph clusteringGraph = new MultiGraph("Clustering Graph");
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
        HashSet<Node> boarderNodes = partitionIntoTwoClusters(
                updateOrCreateNode(networkGraph, String.valueOf(0)),
                updateOrCreateNode(networkGraph, String.valueOf(6)),
                clusteringGraph,
                networkGraph
        );
        for (Node node : boarderNodes) {
            System.out.println(node);
        }
//        makeCluster(clusteringGraph, networkGraph, 4);
    }

    public static HashSet<Node> partitionIntoTwoClusters(
            Node source, Node sink, Graph clusteringGraph, Graph networkGraph
    ) {
        clusteringGraph.clear();
        Node clusteringSource = updateOrCreateNode(clusteringGraph, source.getId());
        Node clusteringSink = updateOrCreateNode(clusteringGraph, sink.getId());

        for (Node currentNode : networkGraph.getEachNode()) {
            if (currentNode == source || currentNode == sink)
                continue;
            // Run Dijkstra for current node before we start exploring its neighbor nodes.
            dijkstra.setSource(currentNode);
            dijkstra.compute();
            // Create or update the source and sink related nodes for clustering graph.
            Node currentNodeSource = updateOrCreateNode(clusteringGraph, currentNode.getId() + "_s");
            Node currentNodeSink = updateOrCreateNode(clusteringGraph, currentNode.getId() + "_t");
            updateOrCreateEdge(clusteringGraph, currentNodeSink, currentNodeSource, 1, 1);
            // Connect the created new nodes to the according proper nodes in clustering graph.
            for (Edge e : currentNode.getEachEdge()) {
                Node neighbor = e.getOpposite(currentNode);
                if (neighbor == sink) {
                    updateOrCreateEdge(clusteringGraph, clusteringSink, currentNodeSink, 1, networkGraph.getNodeCount());
                } else if (neighbor == source) {
                    updateOrCreateEdge(clusteringGraph, clusteringSource, currentNodeSource, 1, networkGraph.getNodeCount());
                } else {
                    if (clusteringGraph.getNode(neighbor.getId() + "_s") != null && clusteringGraph.getNode(neighbor.getId() + "_t") != null){

                        Node neighborNodeSource = updateOrCreateNode(clusteringGraph, neighbor.getId() + "_s");
                        Node neighborNodeSink = updateOrCreateNode(clusteringGraph, neighbor.getId() + "_t");

                        if (dijkstra.getPathLength(sink) > dijkstra.getPathLength(source)) {
                            updateOrCreateEdge(clusteringGraph, currentNodeSink, neighborNodeSource, 1, networkGraph.getNodeCount());
                        } else {
                            updateOrCreateEdge(clusteringGraph, currentNodeSource, neighborNodeSink, 1, networkGraph.getNodeCount());
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
        System.out.println("CTRL: Ford Fulkerson Algorithm completed.");
        // Find cut edges in order to find out which nodes are border nodes.
        Set<Node> sourceCluster = bfs(fd, clusteringSource);
        HashSet<Node> boarderNodes = new HashSet<>();
        for (Edge e : clusteringGraph.getEachEdge()) {
            Node sourceNode = e.getSourceNode();
            Node targetNode = e.getTargetNode();
            if (sourceCluster.contains(sourceNode) && !sourceCluster.contains(targetNode)){
                String actualNodId = sourceNode.getId().split("_")[0];
                Node n = updateOrCreateNode(networkGraph, actualNodId);
                boarderNodes.add(n);
            }
        }
        return boarderNodes;
    }

//    public static void makeCluster(Graph clusteringGraph, Graph networkGraph, int clusterNumber) {
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
//    }

    private static Set<Node> bfs(FordFulkersonAlgorithm fd, Node source) {
        Set<Node> visited = new HashSet<>();
        Queue<Node> queue = new LinkedList<>();
        queue.add(source);
        visited.add(source);
        while (!queue.isEmpty()) {
            Node node = queue.poll();
            for (Edge e : node.getEachEdge()) {
                Node neighbor = e.getOpposite(node);
                double capacity = fd.getCapacity(node, neighbor);
                System.out.println("CTRL: Edge from " + node.getId() + " to " + neighbor.getId() + " has capacity " + capacity);
                // Only consider edges that have remaining capacity.
                if (!visited.contains(neighbor) && capacity >= 0) {
                    visited.add(neighbor);
                    queue.add(neighbor);
                }
            }
        }
        return visited;
    }

    private static String getEdgeIDBasedOnNodes(Node n1, Node n2) {
        String id1 = n1.getId();
        String id2 = n2.getId();

        return id1 + "-" + id2;
    }

    private static void updateVisualGraph(Graph graph) {
        graph.addAttribute("ui.quality");
        graph.addAttribute("ui.antialias");
        graph.display();
    }

    private static Edge updateOrCreateEdge(Graph networkGraph, Node currentNode, Node neighborNode, int length, int capacity) {
        Edge e = networkGraph.getEdge(getEdgeIDBasedOnNodes(currentNode, neighborNode));
        if (e == null) {
            e = networkGraph.addEdge(getEdgeIDBasedOnNodes(currentNode, neighborNode), currentNode, neighborNode);
        }
        e.setAttribute("length", length);
        e.setAttribute("capacity", capacity);
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
