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

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Random;

import org.apache.commons.math3.analysis.function.Sin;
import org.graphstream.algorithm.Dijkstra;
import org.graphstream.graph.Edge;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.implementations.MultiGraph;
import org.graphstream.graph.implementations.SingleGraph;
import scala.util.parsing.combinator.testing.Str;

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
    private HashMap<String, ArrayList<NodeAddress>> cluster;
    private SingleGraph clusteringGraph;
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
        this.dijkstra = new Dijkstra(Dijkstra.Element.EDGE, null, "length");
        this.cluster = new HashMap<>();
        this.clusteringGraph = new SingleGraph("clustering");
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
        makeCluster();
    }

    public final void makeCluster() {
        clusteringGraph.clear();
        int nodeNumber = networkGraph.getGraph().getNodeCount();
        Node h1 = networkGraph.getGraph().getNode(rand.nextInt(nodeNumber));
        Node h2 = networkGraph.getGraph().getNode(rand.nextInt(nodeNumber));
        clusteringGraph.addNode(h1.getId()).addAttribute("ui.label",h1.getId());
        clusteringGraph.addNode(h2.getId()).addAttribute("ui.label",h2.getId());
        Dijkstra dj = new Dijkstra(Dijkstra.Element.EDGE, null, "length");
        for (Node n: networkGraph.getGraph().getEachNode()) {
            if (n.getId().equals(h1.getId()) || n.getId().equals(h2.getId()))
                continue;
            String ns = n.getId()+"_s";
            String nt = n.getId()+"_t";
            setupNewNodeForClustering(ns, nt);
            for (Edge e:n.getEachEdge()){
                Node neighbor = e.getNode1();
                if (neighbor.getId().equals(n.getId()))
                    neighbor = e.getNode0();
                if (neighbor.getId().equals(h2.getId())) {
                    Edge edge = clusteringGraph.addEdge(nt + "-" + h2.getId(), nt, h2.getId());
                    edge.setAttribute("length", 1);
                }
                else if (neighbor.getId().equals(h2.getId())) {
                    Edge edge = clusteringGraph.addEdge(ns + "-" + h1.getId(), ns, h1.getId());
                    edge.setAttribute("length", 1);
                }
                else {
                    String neighbors = neighbor.getId()+"_s";
                    String neighbort = neighbor.getId()+"_t";
                    setupNewNodeForClustering(neighbors, neighbort);
                    int d1 = getDistance(dj, n ,h1);
                    int d2 = getDistance(dj, neighbor, h1);
                    if (d1 == d2)
                        clusteringGraph.addEdge(nt+"-"+neighbors, nt, neighbors).setAttribute("length", 1);
                    else if (d1 < d2)
                        clusteringGraph.addEdge(nt+"-"+neighbors, nt, neighbors).setAttribute("length", 1);
                    else
                        clusteringGraph.addEdge(ns+"-"+neighbors, ns, neighbors).setAttribute("length", 1);
                }
            }
        }
        clusteringGraph.display(true);
        System.out.println("Make Cluster End");
    }

    private void setupNewNodeForClustering(String nodes, String nodet) {
        clusteringGraph.addNode(nodes).addAttribute("ui.label", nodes);
        clusteringGraph.addNode(nodet).addAttribute("ui.label", nodet);
        clusteringGraph.addEdge(nodes+"-"+nodet,
                nodes, nodet).setAttribute("length", 1);
    }

    private int getDistance(Dijkstra d, Node source, Node target) {
        d.clear();
        d.setSource(source);
        d.init(networkGraph.getGraph());
        d.compute();
        int distance = 0;
        for (Node n: d.getPathNodes(target))
            distance++;
        return distance;
    }


    @Override
    public final void manageRoutingRequest(NetworkPacket data) {

        String destination = data.getNetId() + "." + data.getDst();
        String source = data.getNetId() + "." + data.getSrc();

        if (!source.equals(destination)) {

            Node sourceNode = networkGraph.getNode(source);
            Node destinationNode = networkGraph.getNode(destination);
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
