package com.github.sdnwiselab.sdnwise;

import org.graphstream.algorithm.Dijkstra;
import org.graphstream.algorithm.flow.FordFulkersonAlgorithm;
import org.graphstream.graph.*;
import org.graphstream.graph.implementations.MultiGraph;
import org.graphstream.graph.implementations.SingleGraph;

import java.util.*;

public class MakeCluster {
    private static Dijkstra dijkstra;

    public static void main(String[] args) {
        System.setProperty("org.graphstream.ui", "swing");
        SingleGraph clusteringGraph = new SingleGraph("cluster");
        MultiGraph networkGraph = new MultiGraph("cluster2");
        for (int i = 1; i <= 20; i++) {
            setupNewNode(String.valueOf(i), networkGraph);
        }

        setupNewEdge("1", "3", networkGraph, 1, "Edge 1-3");
        setupNewEdge("3", "5", networkGraph, 1, "Edge 3-5");
        setupNewEdge("5", "4", networkGraph, 1, "Edge 5-4");
        setupNewEdge("2", "4", networkGraph, 1, "Edge 2-4");
        setupNewEdge("1", "2", networkGraph, 1, "Edge 1-2");
        setupNewEdge("5", "7", networkGraph, 1, "Edge 5-7");
        setupNewEdge("4", "7", networkGraph, 1, "Edge 4-7");
        setupNewEdge("6", "1", networkGraph, 1, "Edge 6-1");
        setupNewEdge("6", "2", networkGraph, 1, "Edge 6-2");

        setupNewEdge("8", "6", networkGraph, 1, "Edge 8-6");
        setupNewEdge("8", "9", networkGraph, 1, "Edge 8-9");
        setupNewEdge("9", "10", networkGraph, 1, "Edge 9-10");
        setupNewEdge("10", "11", networkGraph, 1, "Edge 10-11");
        setupNewEdge("11", "12", networkGraph, 1, "Edge 11-12");
        setupNewEdge("12", "8", networkGraph, 1, "Edge 12-8");
        setupNewEdge("7", "8", networkGraph, 1, "Edge 7-8");
        setupNewEdge("3", "10", networkGraph, 1, "Edge 3-10");
        setupNewEdge("2", "11", networkGraph, 1, "Edge 2-11");

        setupNewEdge("1", "5", networkGraph, 1, "Edge 1-5");
        setupNewEdge("3", "7", networkGraph, 1, "Edge 3-7");
        setupNewEdge("10", "15", networkGraph, 1, "Edge 10-15");
        setupNewEdge("12", "18", networkGraph, 1, "Edge 12-18");
        setupNewEdge("8", "16", networkGraph, 1, "Edge 8-16");
        setupNewEdge("4", "11", networkGraph, 1, "Edge 4-11");
        setupNewEdge("2", "17", networkGraph, 1, "Edge 2-17");

        networkGraph.display();
        clusteringGraph.display(true);

        // Run dijkstra on network graph
        dijkstra = new Dijkstra(Dijkstra.Element.EDGE, null, "length");
        dijkstra.init(networkGraph);
//        dijkstra.setSource(s);
//        dijkstra.compute();
        makeCluster(clusteringGraph, networkGraph, 4);
    }

    public static void setupNewEdge(String from, String to, Graph graph, int capacity, String label) {
        try {
            Edge e = graph.addEdge(from + "-" + to, from, to);
            e.setAttribute("ui.style", "fill-color: gray;");
            e.setAttribute("ui.label", label);
            e.setAttribute("length", 1);
            e.setAttribute("capacity", capacity);
        } catch (Exception ignored) {}
    }

    public static void setupNewNode(String nodeId, Graph graph) {
        Node n = graph.addNode(nodeId);
        n.setAttribute("ui.label", nodeId);
        n.setAttribute("ui.style", "fill-color: blue; size: 20px;");
    }

    public static HashSet<String> partitionIntoTwoClusters(Node s, Node t, SingleGraph clusteringGraph, MultiGraph networkGraph, int nodeNumber, HashSet<Node> headNodes) {
        try {
            clusteringGraph.addNode(s.getId()).addAttribute("ui.label", "h_" + s.getId());
            clusteringGraph.addNode(t.getId()).addAttribute("ui.label", "h_" + t.getId());
        } catch (Exception ignored) {
        }
        for (Node n : networkGraph.getEachNode()) {
            if (headNodes.contains(n))
                continue;
            String ns = n.getId() + "_s";
            String nt = n.getId() + "_t";

            setupNewNodeForClustering(ns, nt, clusteringGraph);
            for (Edge e : n.getEachEdge()) {
                Node neighbor = e.getNode1();
                String edgeLabel;
                if (neighbor.getId().equals(n.getId()))
                    neighbor = e.getNode0();
                if (neighbor.getId().equals(t.getId())) {
                    edgeLabel = nt + "-" + t.getId();
                    setupNewEdge(nt, t.getId(), clusteringGraph, nodeNumber, edgeLabel);
                } else if (neighbor.getId().equals(s.getId())) {
                    edgeLabel = s.getId() + "-" + ns;
                    setupNewEdge(s.getId(), ns, clusteringGraph, nodeNumber, edgeLabel);
                } else
                    connectingTwoNonHeadNodesNeighbor(neighbor, n, clusteringGraph, ns, nt, nodeNumber);
            }
        }
        FordFulkersonAlgorithm fd = new FordFulkersonAlgorithm();
        fd.setCapacityAttribute("capacity");
        fd.init(clusteringGraph, s.getId(), t.getId());
        fd.compute();
        HashSet<String> clusterS = bfs(fd, clusteringGraph.getNode(s.getId()));
//        HashSet<String> boarderNodes = new HashSet<>();
        HashSet<String> cluster = new HashSet<>();
        for (String nodeId : clusterS) {
            if (nodeId.endsWith("_t") || nodeId.endsWith("_s")) {
                String actualNodeId = nodeId.split("_")[0];
//                boarderNodes.add(actualNodeId);
                cluster.add(actualNodeId);
            } else cluster.add(nodeId);
        }
        return cluster;
    }

    public static void connectingTwoNonHeadNodesNeighbor
            (Node neighbor, Node n, SingleGraph clusteringGraph, String ns, String nt, int nodeNumber) {
        String neighbor_s = neighbor.getId() + "_s";
        String neighbor_t = neighbor.getId() + "_t";
        setupNewNodeForClustering(neighbor_s, neighbor_t, clusteringGraph);
        Pair<Double> distances = getDistance(n, neighbor);
        double d1 = distances.getFirst();
        double d2 = distances.getSecond();
        try {
            if (d1 == d2)
                setupNewEdge(neighbor_s, ns, clusteringGraph, nodeNumber, neighbor_s + "-" + ns);
            else if (d1 < d2)
                setupNewEdge(nt, neighbor_s, clusteringGraph, nodeNumber, nt + "-" + neighbor_s);
            else
                setupNewEdge(neighbor_t, ns, clusteringGraph, nodeNumber, neighbor_t + "-" + ns);
        } catch (IdAlreadyInUseException | EdgeRejectedException ignored) {
        }
    }

    public static void makeCluster(SingleGraph clusteringGraph, MultiGraph networkGraph, int clusterNumber) {
        int nodeNumber = networkGraph.getNodeCount();
        Random random = new Random();
        HashSet<Node> headNodes = new HashSet<>();
        while (headNodes.size() < clusterNumber) {
            Node candidate = networkGraph.getNode(random.nextInt(nodeNumber));
            boolean isNeighbor = headNodes.stream().anyMatch(head -> head.hasEdgeBetween(candidate));
            if (!isNeighbor) {
                headNodes.add(candidate);
            }
        }
        HashMap<String, HashSet<String>> clustering = new HashMap<>();
        Node last = null;
        for (Node s : headNodes) {
            dijkstra.clear();
            dijkstra.init(networkGraph);
            dijkstra.setSource(s);
            dijkstra.compute();
            HashSet<String> prev = new HashSet<>();
            System.out.println(s.getId());
            System.out.println(headNodes);
            for (Node t: headNodes) {
                if (t.getId().equals(s.getId()) || clustering.containsKey(t.getId())) continue;
                clusteringGraph.clear();
                HashSet <String> clusterS = partitionIntoTwoClusters(s, t, clusteringGraph, networkGraph, nodeNumber, headNodes);
                if (prev.isEmpty()) prev = clusterS;
                else prev.retainAll(clusterS);
            }
            for (String nodeId: prev) {
                networkGraph.removeNode(nodeId);
            }
            clustering.put(s.getId(), prev);
            System.out.println("Final Cluster of " + s.getId() + ": " + prev);
            last = s;
        }
        HashSet<String> nodeIds = new HashSet<>();
        assert last != null;
        nodeIds.add(last.getId());
        for (Node n: networkGraph.getEachNode())
            nodeIds.add(n.getId());
        clustering.put(last.getId(), nodeIds);
        System.out.println("Final Cluster of " + last.getId() + ": " + nodeIds);
    }

    private static HashSet<String> bfs(FordFulkersonAlgorithm fd, Node source) {
        HashSet<String> visited = new HashSet<>();
        Queue<Node> queue = new LinkedList<>();
        queue.add(source);
        visited.add(source.getId());
        while (!queue.isEmpty()) {
            Node node = queue.poll();
            for (Edge e : node.getEachEdge()) {
                Node neighbor = e.getOpposite(node);
                if ((int) e.getAttribute("capacity") == fd.getFlow(node, neighbor))
                    continue;
                if (visited.contains(neighbor.getId()))
                    continue;
                visited.add(neighbor.getId());
                queue.add(neighbor);
            }
        }
        return visited;
    }

    private static void setupNewNodeForClustering(String nodes, String nodet, SingleGraph clusteringGraph) {
        if (clusteringGraph.getNode(nodes) != null)
            return;
        clusteringGraph.addNode(nodes).addAttribute("ui.label", nodes);
        clusteringGraph.addNode(nodet).addAttribute("ui.label", nodet);
        Edge e = clusteringGraph.addEdge(nodes + "-" + nodet, nodes, nodet);
        e.setAttribute("length", 1);
        e.setAttribute("capacity", 1);
    }

    private static Pair<Double> getDistance(Node target1, Node target2) {
        return new Pair<>(dijkstra.getPathLength(target1), dijkstra.getPathLength(target2));
    }
}

class Pair<Double> {
    private final Double first;
    private final Double second;

    public Pair(Double first, Double second) {
        this.first = first;
        this.second = second;
    }

    public Double getFirst() {
        return first;
    }

    public Double getSecond() {
        return second;
    }
}