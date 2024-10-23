package com.github.sdnwiselab.sdnwise.utils;

import org.graphstream.algorithm.Dijkstra;
import org.graphstream.algorithm.util.FibonacciHeap;
import org.graphstream.graph.Edge;
import org.graphstream.graph.Node;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;

public class ClusteringDijkstra extends Dijkstra {
    protected final HashMap<String, String> nodesToClusterNode;
    protected final HashMap<String, HashSet<String>> clusterNodesToBoarderNodes;


    public ClusteringDijkstra(Element element,
                              String resultAttribute,
                              String lengthAttribute,
                              HashMap<String, String> nodesToClusterNode,
                              HashMap<String, HashSet<String>> clusterNodesToBoarderNodes
    ) {
        super(element, resultAttribute, lengthAttribute);
        this.nodesToClusterNode = nodesToClusterNode;
        this.clusterNodesToBoarderNodes = clusterNodesToBoarderNodes;
    }

    public Node getClusterNode(String nodeID){
        return this.graph.getNode(this.nodesToClusterNode.get(nodeID));
    }

    public HashSet<Node> getBoarderNodes(String clusterNodeID){
        HashSet<Node> nodes = new HashSet<>();
        for (String nodeID : this.clusterNodesToBoarderNodes.get(clusterNodeID)) {
            nodes.add(this.graph.getNode(nodeID));
        }
        return nodes;
    }

    @Override
    protected void makeTree() {
        FibonacciHeap<Double, Node> heap = new FibonacciHeap<>();

        for (Node node : this.graph) {
            Data data = new Data();
            double v = node == this.source ? this.getSourceLength() : Double.POSITIVE_INFINITY;
            data.fn = heap.add(v, node);
            data.edgeFromParent = null;
            node.addAttribute(this.resultAttribute, data);
        }

        while(!heap.isEmpty()) {
            Node u = heap.extractMin();
            Node headU = getClusterNode(u.getId());
            Data dataU = u.getAttribute(this.resultAttribute);
            dataU.distance = dataU.fn.getKey();
            dataU.fn = null;
            if (dataU.edgeFromParent != null) {
                this.edgeOn(dataU.edgeFromParent);
            }

            for (Edge e : u.getEachEdge()) {
                Node v = e.getOpposite(u);
                Node headV = getClusterNode(v.getId());
                Data dataV = v.getAttribute(this.resultAttribute);
                if (dataV.fn != null) {
                    if (
                            headU.equals(headV) ||
                            (getBoarderNodes(headU.getId()).contains(u) && getBoarderNodes(headV.getId()).contains(v))
                    ) {
                        double tryDist = dataU.distance + this.getLength(e, v);
                        if (tryDist < dataV.fn.getKey()) {
                            dataV.edgeFromParent = e;
                            heap.decreaseKey(dataV.fn, tryDist);
                        }
                    }
                }
            }
        }
    }

    public void clear() {
        super.clear();
        Node node;
        for(Iterator<Node> i$ = this.graph.iterator(); i$.hasNext(); node.removeAttribute(this.resultAttribute)) {
            node = i$.next();
            Data data = node.getAttribute(this.resultAttribute);
            if (data != null) {
                data.fn = null;
                data.edgeFromParent = null;
            }
        }
    }

    public <T extends Edge> T getEdgeFromParent(Node target) {
        return (T) ((Data)target.getAttribute(this.resultAttribute)).edgeFromParent;
    }

    public double getPathLength(Node target) {
        return ((Data)target.getAttribute(this.resultAttribute)).distance;
    }


    protected static class Data extends Dijkstra.Data {
        FibonacciHeap<Double, Node>.Node fn;
        Edge edgeFromParent;
        double distance;

        protected Data() {
        }
    }
}
