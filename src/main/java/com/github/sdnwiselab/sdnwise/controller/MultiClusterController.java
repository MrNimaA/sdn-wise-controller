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
import org.graphstream.graph.implementations.Graphs;
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
public class MultiClusterController extends Controller {

    private final Dijkstra dijkstra;
    private String lastSource = "";
    private long lastModification = -1;
    private HashMap<Node, Node> nodesToClusterNode = new HashMap<>();
    private HashMap<Node, HashSet<Node>> clusterNodesToBoarderNodes = new HashMap<>();
    private Graph networkGraphCopy;

    /**
     * Constructor method fo ControllerDijkstra.
     * 
     * @param lower Lower Adpater object.
     * @param networkGraph NetworkGraph object.
     */
    public MultiClusterController(Adapter lower, NetworkGraph networkGraph) {
        super(lower, networkGraph);
        dijkstra = new Dijkstra(Dijkstra.Element.EDGE, null, "length");
        nodesToClusterNode = new HashMap<>();
        clusterNodesToBoarderNodes = new HashMap<>();
        networkGraphCopy = Graphs.clone(networkGraph.getGraph());
    }

    /**
     *  Return a predefined set of Head Nodes if the initialHeadNodes in provided, else return a random set of head
     *  nodes.
     * @param networkGraph the network graph to get its head nodes.
     * @param initialHeadNodes the initial head nodes if any.
     * @param clusterCount the number of clusters to be produced.
     * @return a set of head nodes.
     */
    private HashSet<Node> getHeadNodes(Graph networkGraph, String[] initialHeadNodes, Integer clusterCount) {
        if (initialHeadNodes.length > 0) {
            HashSet<Node> headNodes = new HashSet<>();
            for (String headNode : initialHeadNodes) {
                headNodes.add(networkGraph.getNode(headNode));
            }
            System.out.println("[CTRL]: Predefined Head Nodes : " + headNodes);
            return headNodes;
        }
        return getRandomHeadNodes(networkGraph, clusterCount);
    }

    /**
     * Return a set of random nodes as head nodes.
     *
     * @param networkGraph the network graph to get its head nodes.
     * @param clusterCount the number of clusters to be produced.
     * @return a set of head nodes.
     */
    private HashSet<Node> getRandomHeadNodes(Graph networkGraph, Integer clusterCount) {
        HashSet<Node> headNodes = new HashSet<>();
        int totalNodeCount = networkGraph.getNodeCount();
        while (headNodes.size() < clusterCount) {
            Node candidate = networkGraph.getNode(new Random().nextInt(totalNodeCount));
            boolean isNeighbor = headNodes.stream().anyMatch(head -> head.hasEdgeBetween(candidate));
            if (!isNeighbor) {
                headNodes.add(candidate);
            }
        }
        System.out.println("[CTRL]: Random Head Nodes: " + headNodes);
        return headNodes;
    }

    public void makeCluster(Graph networkGraph, HashSet<Node> headNodes) {
        Graph clusteringGraph = new SingleGraph("SDN-WISE Clustering Network");
        HashMap<Node, HashSet<Node>> clusters = new HashMap<>();

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
                if (sink.equals(source) || clusters.containsKey(sink))
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
            clusters.put(source, prev);
            System.out.println("[CTRL]: Final Cluster of " + source + ": " + prev);
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
            nodesToClusterNode.put(node, lastHeadNode);
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
                HashSet<Node> boarderNodes = clusterNodesToBoarderNodes.getOrDefault(lastHeadNode, new HashSet<>());
                boarderNodes.add(node);
                clusterNodesToBoarderNodes.put(lastHeadNode, boarderNodes);
            }
        }
        clusters.put(lastHeadNode, nodes);
        System.out.println("[CTRL]: Final Cluster of " + lastHeadNode + ": " + nodes);
    }

    public HashSet<Node> partitionIntoTwoClusters(
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
        return cluster;
    }

    private Set<Node> bfs(FordFulkersonAlgorithm fd, Node source) {
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

    private void updateOrCreateEdge(Graph networkGraph, Node sourceNode, Node targetNode, int length, int capacity) {
        Edge e = networkGraph.getEdge(sourceNode.getId() + "-" + targetNode.getId());
        if (e == null) {
            e = networkGraph.addEdge(sourceNode.getId() + "-" + targetNode.getId(), sourceNode, targetNode);
        }
        Integer[] capacities = {capacity, capacity};
        e.setAttribute("length", length);
        e.setAttribute("capacity", (Object[]) capacities);
    }

    private Node updateOrCreateNode(Graph networkGraph, String value) {
        Node currentNode = networkGraph.getNode(value);
        if (currentNode == null) {
            currentNode = networkGraph.addNode(value);
        }
        return currentNode;
    }

    @Override
    public final void graphUpdate() {
        System.out.println("[CTRL]: Graph is being updated...");
        nodesToClusterNode = new HashMap<>();
        clusterNodesToBoarderNodes = new HashMap<>();
        networkGraphCopy = Graphs.clone(networkGraph.getGraph());
        System.out.println("[CTRL]: Calling make cluster...");

        makeCluster(
                networkGraphCopy,
                getRandomHeadNodes(networkGraphCopy, 4)
        );
        System.out.println("[CTRL]: Finished make cluster.");
        System.out.println("[CTRL]: boarder nodes are : " + clusterNodesToBoarderNodes);
        System.out.println("[CTRL]: nodes to cluster node are : " + nodesToClusterNode);
    }

    @Override
    public final void manageRoutingRequest(NetworkPacket data) {

        String destination = data.getNetId() + "." + data.getDst();
        String source = data.getNetId() + "." + data.getSrc();

        if (!source.equals(destination)) {

            Node sourceNode = networkGraph.getNode(source);
            Node destinationNode = networkGraph.getNode(destination);

//            Node sourceHead = getHeadClusterNode(sourceNode);
//            Node destHead = getHeadClusterNode(sourceNode);

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
}

