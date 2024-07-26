package com.github.sdnwiselab.sdnwise;

import org.graphstream.algorithm.Dijkstra;
import org.graphstream.algorithm.flow.FordFulkersonAlgorithm;
import org.graphstream.graph.*;
import org.graphstream.graph.implementations.MultiGraph;
import org.graphstream.graph.implementations.SingleGraph;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;

public class MakeCluster {
    public static void main(String[] args) {
        System.setProperty("org.graphstream.ui", "swing");
        SingleGraph clusteringGraph = new SingleGraph("cluster");
        MultiGraph networkGraph = new MultiGraph("cluster2");
        for (int i = 1; i <= 5; i++) {
            setupNewNode(String.valueOf(i), networkGraph);
        }
        Node sink = setupNewNode("6", networkGraph);
        Node source = setupNewNode("7", networkGraph);

        setupNewEdge("1", "3", networkGraph, "");
        setupNewEdge("3", "5", networkGraph, "");
        setupNewEdge("5", "4", networkGraph, "");
        setupNewEdge("2", "4", networkGraph, "");
        setupNewEdge("1", "2", networkGraph, "");
        setupNewEdge("5", "7", networkGraph, "");
        setupNewEdge("4", "7", networkGraph, "");
        setupNewEdge("6", "1", networkGraph, "");
        setupNewEdge("6", "2", networkGraph, "");

        networkGraph.display();
        clusteringGraph.display(true);
        makeCluster(clusteringGraph, networkGraph, sink, source);
    }


    public static Edge setupNewEdge(String from, String to, Graph graph, String capacity) {
        Edge e = graph.addEdge(from + "-" + to, from, to);
        e.setAttribute("ui.style", "fill-color: gray;");
        e.setAttribute("length", "1");
        if (!capacity.isEmpty()) {
            e.setAttribute("capacity", capacity);
        }
        return e;
    }

    public static Node setupNewNode(String nodeId, Graph graph) {
        Node n = graph.addNode(nodeId);
        n.setAttribute("ui.label", nodeId);
        n.setAttribute("ui.style", "fill-color: blue; size: 20px;");
        return n;
    }

    public static void makeCluster(SingleGraph clusteringGraph, MultiGraph networkGraph, Node s, Node t) {
        clusteringGraph.clear();
        int nodeNumber = networkGraph.getNodeCount();
        clusteringGraph.addNode(s.getId()).addAttribute("ui.label", "hs");
        clusteringGraph.addNode(t.getId()).addAttribute("ui.label", "ht");
        Dijkstra dj = new Dijkstra(Dijkstra.Element.EDGE, null, "length");
        for (Node n : networkGraph.getEachNode()) {
            if (n.getId().equals(s.getId()) || n.getId().equals(t.getId()))
                continue;
            String ns = n.getId() + "_s";
            String nt = n.getId() + "_t";
            setupNewNodeForClustering(ns, nt, clusteringGraph);
            for (Edge e : n.getEachEdge()) {
                Node neighbor = e.getNode1();
                if (neighbor.getId().equals(n.getId()))
                    neighbor = e.getNode0();
                if (neighbor.getId().equals(t.getId())) {
                    String edgeLabel = nt + "-" + t.getId();
                    Edge edge = clusteringGraph.addEdge(edgeLabel, nt, t.getId());
                    edge.setAttribute("ui.label", edgeLabel);
                    edge.setAttribute("length", 1);
                    edge.setAttribute("capacity", nodeNumber);
                } else if (neighbor.getId().equals(s.getId())) {
                    String edgeLabel = s.getId() + "-" + ns;
                    Edge edge = clusteringGraph.addEdge(edgeLabel, s.getId(), ns);
                    edge.setAttribute("ui.label", edgeLabel);
                    edge.setAttribute("length", 1);
                    edge.setAttribute("capacity", nodeNumber);
                } else {
                    String neighbor_s = neighbor.getId() + "_s";
                    String neighbor_t = neighbor.getId() + "_t";
                    setupNewNodeForClustering(neighbor_s, neighbor_t, clusteringGraph);
                    Pair<Double> distances = getDistance(networkGraph, s, n, neighbor);
                    double d1 = distances.getFirst();
                    double d2 = distances.getSecond();
                    try {
                        Edge edge;
                        if (d1 == d2)
                            edge = clusteringGraph.addEdge(neighbor_s + "-" + ns, ns, neighbor_s);
                        else if (d1 < d2)
                            edge = clusteringGraph.addEdge(nt + "-" + neighbor_s, nt, neighbor_s);
                        else
                            edge = clusteringGraph.addEdge(neighbor_t + "-" + ns, ns, neighbor_t);
                        edge.setAttribute("length", 1);
                        edge.setAttribute("capacity", nodeNumber);
                    } catch (IdAlreadyInUseException | EdgeRejectedException ignored) {
                    }
                }
            }
        }
        FordFulkersonAlgorithm fd = new FordFulkersonAlgorithm();
        fd.setCapacityAttribute("capacity");
        fd.init(clusteringGraph, s.getId(), t.getId());
        fd.compute();
        bfs(fd, clusteringGraph.getNode(s.getId()));
        System.out.println("Make Cluster End -- Max Flow: " + fd.getMaximumFlow());
    }

    private static void bfs(FordFulkersonAlgorithm fd, Node source) {
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
        System.out.println("hs Cluster: " + visited);
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

    private static Pair<Double> getDistance(Graph graph, Node source, Node target1, Node target2) {
        Dijkstra dijkstra = new Dijkstra(Dijkstra.Element.EDGE, null, "length");
        dijkstra.init(graph);
        dijkstra.setSource(source);
        dijkstra.compute();
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