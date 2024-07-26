package com.github.sdnwiselab.sdnwise;

import org.graphstream.algorithm.Dijkstra;
import org.graphstream.graph.*;
import org.graphstream.graph.implementations.MultiGraph;
import org.graphstream.graph.implementations.SingleGraph;
import org.graphstream.algorithm.flow.FordFulkersonAlgorithm;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Random;

public class MakeCluster {
    public static void main(String[] args) {
        System.setProperty("org.graphstream.ui", "swing");
        SingleGraph clusteringGraph = new SingleGraph("cluster");
        MultiGraph networkGraph = new MultiGraph("cluster2");
        for (int i = 1; i <= 5; i++) {
            networkGraph.addNode(String.valueOf(i)).setAttribute("ui.label", String.valueOf(i));
        }
        networkGraph.addNode("6").setAttribute("ui.label", "ht");
        networkGraph.addNode("7").setAttribute("ui.label", "hs");

        networkGraph.addEdge("1-3", "1", "3");
        networkGraph.addEdge("3-5", "3", "5");
        networkGraph.addEdge("5-4", "5", "4");
        networkGraph.addEdge("2-4", "2", "4");
        networkGraph.addEdge("1-2", "1", "2");
        networkGraph.addEdge("5-7", "5", "7");
        networkGraph.addEdge("4-7", "4", "7");
        networkGraph.addEdge("6-1", "6", "1");
        networkGraph.addEdge("6-2", "6", "2");

        for (int i = 1; i <= 7; i++) {
            networkGraph.getNode(String.valueOf(i)).setAttribute("ui.style", "fill-color: blue; size: 20px;");
        }
        networkGraph.getEachEdge().forEach(e -> e.setAttribute("ui.style", "fill-color: gray;"));
        networkGraph.getEachEdge().forEach(e -> e.setAttribute("length", "1"));

        networkGraph.display();
        clusteringGraph.display(true);
        makeCluster(clusteringGraph, networkGraph);
    }

    public static void makeCluster(SingleGraph clusteringGraph, MultiGraph networkGraph) {
        clusteringGraph.clear();
        int nodeNumber = networkGraph.getNodeCount();
        Random rand = new Random();
        Node h1 = networkGraph.getNode("6");
        Node h2 = networkGraph.getNode("7");
        clusteringGraph.addNode(h1.getId()).addAttribute("ui.label","hs");
        clusteringGraph.addNode(h2.getId()).addAttribute("ui.label","ht");
        Dijkstra dj = new Dijkstra(Dijkstra.Element.EDGE, null, "length");
        for (Node n: networkGraph.getEachNode()) {
            if (n.getId().equals(h1.getId()) || n.getId().equals(h2.getId()))
                continue;
            String ns = n.getId()+"_s";
            String nt = n.getId()+"_t";
            setupNewNodeForClustering(ns, nt, clusteringGraph);
            for (Edge e:n.getEachEdge()){
                Node neighbor = e.getNode1();
                if (neighbor.getId().equals(n.getId()))
                    neighbor = e.getNode0();
                if (neighbor.getId().equals(h2.getId())) {
                    Edge edge = clusteringGraph.addEdge(nt + "-" + h2.getId(), nt, h2.getId());
                    edge.setAttribute("length", 1);
                    edge.setAttribute("capacity", nodeNumber);
                }
                else if (neighbor.getId().equals(h1.getId())) {
                    Edge edge = clusteringGraph.addEdge(h1.getId() + "-" + ns  , h1.getId(), ns);
                    edge.setAttribute("length", 1);
                    edge.setAttribute("capacity", nodeNumber);
                }
                else {
                    String neighbors = neighbor.getId()+"_s";
                    String neighbort = neighbor.getId()+"_t";
                    setupNewNodeForClustering(neighbors, neighbort, clusteringGraph);
                    int d1 = getDistance(dj, n ,h1, networkGraph);
                    int d2 = getDistance(dj, neighbor, h1, networkGraph);
//                    System.out.println(n + " - " + h1 + " - " + d1);
//                    System.out.println(neighbor + " - " + h1 + " - " + d2);
                    try {
                        Edge edge;
                        if (d1 == d2)
                            edge = clusteringGraph.addEdge(neighbors + "-" + ns, ns, neighbors);
                        else if (d1 < d2)
                            edge = clusteringGraph.addEdge(nt + "-" + neighbors, nt, neighbors);
                        else
                            edge = clusteringGraph.addEdge(neighbort + "-" + ns, ns, neighbort);
                        edge.setAttribute("length", 1);
                        edge.setAttribute("capacity", nodeNumber);
                    } catch (IdAlreadyInUseException|EdgeRejectedException ignored){}
                }
            }
        }
        FordFulkersonAlgorithm fd = new FordFulkersonAlgorithm();
        fd.setCapacityAttribute("capacity");
        fd.init(clusteringGraph, h1.getId(), h2.getId());
        fd.compute();
        bfs(fd, clusteringGraph.getNode(h1.getId()));
        System.out.println("Make Cluster End -- Max Flow: " + fd.getMaximumFlow());
    }

    private static void bfs(FordFulkersonAlgorithm fd, Node source) {
        HashSet<String> visited = new HashSet<>();
        Queue<Node> queue = new LinkedList<>();
        queue.add(source);
        visited.add(source.getId());
        while (!queue.isEmpty()) {
            Node node = queue.poll();
            for (Edge e: node.getEachEdge()){
                Node neighbor = e.getOpposite(node);
                if ((int)e.getAttribute("capacity") == fd.getFlow(node, neighbor))
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
        Edge e = clusteringGraph.addEdge(nodes+"-"+nodet, nodes, nodet);
        e.setAttribute("length", 1);
        e.setAttribute("capacity", 1);
    }

    private static int getDistance(Dijkstra d, Node source, Node target, MultiGraph networkGraph) {
        d.setSource(source);
        d.init(networkGraph);
        d.compute();
        int distance = 0;
        for (Node n: d.getPathNodes(target)) {
//            System.out.println(n);
            distance++;
        }
//        System.out.println(d.getPath(networkGraph.getNode("2")).size());
        d.clear();
        return distance;
    }
}
