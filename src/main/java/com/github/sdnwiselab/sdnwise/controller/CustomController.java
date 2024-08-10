/* 
 * Copyright (C) 2015 SDN-WISE
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package com.github.sdnwiselab.sdnwise.controller;

import com.github.sdnwiselab.sdnwise.adapter.Adapter;
import com.github.sdnwiselab.sdnwise.packet.NetworkPacket;
import com.github.sdnwiselab.sdnwise.topology.NetworkGraph;
import com.github.sdnwiselab.sdnwise.util.NodeAddress;

import java.util.*;

import org.graphstream.algorithm.Dijkstra;
import org.graphstream.algorithm.flow.FordFulkersonAlgorithm;
import org.graphstream.graph.*;
import org.graphstream.graph.implementations.MultiGraph;
import org.graphstream.graph.implementations.SingleGraph;

/**
 * This class implements the Controller class using the Dijkstra routing
 * algorithm in order to find the shortest path between nodes. When a request
 * from the network is sent, this class sends a SDN_WISE_OPEN_PATH message with
 * the shortest path. No action is taken if the topology of the network changes.
 *
 * @author Sebastiano Milardo
 * @version 0.1
 */
public class CustomController extends Controller {

    private final Dijkstra dijkstra;
    private String lastSource = "";
    private long lastModification = -1;
    private final HashMap<String, HashSet<String>> cluster;
    private final HashMap<String, HashMap<String, HashSet<String>>> boarderNodes;
    private final SingleGraph clusteringGraph;
    private final Random rand = new Random();
    /*
     * Constructor method fo ControllerDijkstra.
     * 
     * @param id ControllerId object.
     * @param lower Lower Adpater object.
     * @param networkGraph NetworkGraph object.
     */
    public CustomController(Adapter lower, NetworkGraph networkGraph) {
        super(lower, networkGraph);
        dijkstra = new Dijkstra(Dijkstra.Element.EDGE, null, "length");
        cluster = new HashMap<>();
        boarderNodes = new HashMap<>();
        clusteringGraph = new SingleGraph("clustering");
    }

    @Override
    public final void graphUpdate() {
        System.out.println("Graph update");
        for (Node n: networkGraph.getGraph()) {
            System.out.println(n.toString());
        }
        System.out.println("[CTRL]: " + networkGraph.getGraph());
        System.out.println("End");

        System.out.println("Calling make cluster");
        int clusterNumber = 4;
        makeCluster(clusterNumber);
    }

    public void setupNewEdge(String from, String to, Graph graph, int capacity, String label) {
        try {
            Edge e = graph.addEdge(from + "-" + to, from, to);
            e.setAttribute("ui.style", "fill-color: gray;");
            e.setAttribute("ui.label", label);
            e.setAttribute("length", 1);
            e.setAttribute("capacity", capacity);
        } catch (Exception ignored) {}
    }

    public void setupNewNode(String nodeId, Graph graph) {
        Node n = graph.addNode(nodeId);
        n.setAttribute("ui.label", nodeId);
        n.setAttribute("ui.style", "fill-color: blue; size: 20px;");
    }

    public List<HashSet<String>> partitionIntoTwoClusters(Node s, Node t, SingleGraph clusteringGraph, MultiGraph networkGraph, int nodeNumber, HashSet<Node> headNodes) {
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
        return getClusterAndBoarderNodes(clusterS);
    }

    private List<HashSet<String>> getClusterAndBoarderNodes(HashSet<String> clusterS) {
        HashSet<String> boarderNodes = new HashSet<>();
        HashSet<String> cluster = new HashSet<>();
        for (String nodeId : clusterS) {
            if (nodeId.endsWith("_t") || nodeId.endsWith("_s")) {
                String actualNodeId = nodeId.split("_")[0];
                boarderNodes.add(actualNodeId);
                cluster.add(actualNodeId);
            } else cluster.add(nodeId);
        }
        List<HashSet<String>> l = new ArrayList<>();
        l.add(cluster);
        l.add(boarderNodes);
        return l;
    }

    public void connectingTwoNonHeadNodesNeighbor
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

    private MultiGraph copyNetworkGraph(Graph networkGraph) {
        MultiGraph graph = new MultiGraph("copy");
        for (Node n : networkGraph) {
            graph.addNode(n.getId());
        }
        for (Edge e : networkGraph.getEachEdge()) {
            graph.addEdge(e.getId(), e.getNode0().getId(), e.getNode1().getId());
        }
        return graph;
    }

    public void makeCluster(int clusterNumber) {
        int nodeNumber = networkGraph.getGraph().getNodeCount();
        MultiGraph graph = copyNetworkGraph(networkGraph.getGraph());
        HashSet<Node> headNodes = new HashSet<>();
        while (headNodes.size() < clusterNumber) {
            Node candidate = graph.getNode(rand.nextInt(nodeNumber));
            boolean isNeighbor = headNodes.stream().anyMatch(head -> head.hasEdgeBetween(candidate));
            if (!isNeighbor) {
                headNodes.add(candidate);
            }
        }
//        HashMap<String, HashSet<String>> clustering = new HashMap<>();
        Node last = null;
        for (Node s : headNodes) {
            dijkstra.clear();
            dijkstra.init(graph);
            dijkstra.setSource(s);
            dijkstra.compute();
            HashSet<String> prev = new HashSet<>();
//            System.out.println(s.getId());
//            System.out.println(headNodes);
            for (Node t: headNodes) {
                if (t.getId().equals(s.getId()) || cluster.containsKey(t.getId())) continue;
                clusteringGraph.clear();
                List<HashSet<String>> l = partitionIntoTwoClusters(s, t, clusteringGraph, graph, nodeNumber, headNodes);
                if (prev.isEmpty())
                    prev = l.get(0);
                else
                    prev.retainAll(l.get(0));
                boarderNodes.getOrDefault(s.getId(), new HashMap<>()).put(t.getId(), l.get(1));
                boarderNodes.getOrDefault(t.getId(), new HashMap<>()).put(s.getId(), l.get(1));
            }
            for (String nodeId: prev) {
                graph.removeNode(nodeId);
            }
            cluster.put(s.getId(), prev);
            System.out.println("Final Cluster of " + s.getId() + ": " + prev);
            last = s;
        }
        HashSet<String> nodeIds = new HashSet<>();
        assert last != null;
        nodeIds.add(last.getId());
        for (Node n: graph.getEachNode())
            nodeIds.add(n.getId());
        cluster.put(last.getId(), nodeIds);
    }

    private HashSet<String> bfs(FordFulkersonAlgorithm fd, Node source) {
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

    private void setupNewNodeForClustering(String nodes, String nodet, SingleGraph clusteringGraph) {
        if (clusteringGraph.getNode(nodes) != null)
            return;
        clusteringGraph.addNode(nodes).addAttribute("ui.label", nodes);
        clusteringGraph.addNode(nodet).addAttribute("ui.label", nodet);
        Edge e = clusteringGraph.addEdge(nodes + "-" + nodet, nodes, nodet);
        e.setAttribute("length", 1);
        e.setAttribute("capacity", 1);
    }

    private Pair<Double> getDistance(Node target1, Node target2) {
        return new Pair<>(dijkstra.getPathLength(target1), dijkstra.getPathLength(target2));
    }

    private Node getHeadClusterNode(Node n) {
        if (cluster.containsKey(n.getId()))
            return n;
        String headID = String.valueOf(cluster.keySet().stream().filter(k -> cluster.get(k).contains(n.getId())).findFirst());
        if (headID == null) {
            System.err.println("The given node is not in any cluster");
            return null;
        }
        return networkGraph.getNode(headID);
    }
    @Override
    public final void manageRoutingRequest(NetworkPacket data) {

        String destination = data.getNetId() + "." + data.getDst();
        String source = data.getNetId() + "." + data.getSrc();

        if (!source.equals(destination)) {

            Node sourceNode = networkGraph.getNode(source);
            Node destinationNode = networkGraph.getNode(destination);

            Node sourceHead = getHeadClusterNode(sourceNode);
            Node destHead = getHeadClusterNode(sourceNode);

            LinkedList<NodeAddress> path = null;

            if (sourceNode != null && destinationNode != null) {

                if (!lastSource.equals(source) || lastModification != networkGraph.getLastModification()) {
                    results.clear();
                    dijkstra.init(networkGraph.getGraph());
                    dijkstra.setSource(networkGraph.getNode(source));
                    dijkstra.compute();
                    lastSource = source;
                    lastModification = networkGraph.getLastModification();
                } else {
                    path = results.get(data.getDst());
                }
                if (path == null) {
                    path = new LinkedList<>();
                    for (Node node : dijkstra.getPathNodes(networkGraph.getNode(destination))) {
                        path.push((NodeAddress) node.getAttribute("nodeAddress"));
                    }
                    System.out.println("[CTRL]: " + path);
                    results.put(data.getDst(), path);
                }
                if (path.size() > 1) {
                    sendPath((byte) data.getNetId(), path.getFirst(), path);
                    data.unsetRequestFlag();
                    data.setSrc(getSinkAddress());
                    sendNetworkPacket(data);

                } else {
                    // TODO send a rule in order to say "wait I dont have a path"
                    //sendMessage(data.getNetId(), data.getDst(),(byte) 4, new byte[10]);
                }
            }
        }
    }

    @Override
    public void setupNetwork() {
    }

    static class Pair<Double> {
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
}

