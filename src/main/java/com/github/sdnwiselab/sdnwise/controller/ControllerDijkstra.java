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
import com.github.sdnwiselab.sdnwise.topology.Edge;
import com.github.sdnwiselab.sdnwise.topology.NetworkGraph;
import com.github.sdnwiselab.sdnwise.topology.Node;
import com.github.sdnwiselab.sdnwise.util.NodeAddress;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;

import java.util.LinkedList;

/**
 * This class implements the Controller class using the Dijkstra routing
 * algorithm in order to find the shortest path between nodes. When a request
 * from the network is sent, this class sends an SDN_WISE_OPEN_PATH message with
 * the shortest path. No action is taken if the topology of the network changes.
 *
 * @author Sebastiano Milardo
 * @version 0.1
 */
public class ControllerDijkstra extends Controller {

    private String lastSource = "";
    private long lastModification = -1;

    /**
     * Constructor Method for the Controller Class.
     *
     * @param lower        Lower Adapter object.
     * @param networkGraph NetworkGraph object.
     */
    ControllerDijkstra(Adapter lower, NetworkGraph networkGraph) {
        super(lower, networkGraph);
    }

    @Override
    public final void graphUpdate() {

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
                    lastSource = source;
                    lastModification = networkGraph.getLastModification();
                } else {
                    path = results.get(data.getDst());
                }
                if (path == null) {
                    path = new LinkedList<>();
                    DijkstraShortestPath<Node, Edge> dijkstra = new DijkstraShortestPath<>(this.getNetworkGraph().getGraph());
                    for (Node node : dijkstra.getPath(sourceNode, destinationNode).getVertexList()){
                        path.push(node.getNodeAddress());
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
