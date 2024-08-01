package com.github.sdnwiselab.sdnwise.topology;

import org.jgrapht.graph.DefaultWeightedEdge;

public class Edge extends DefaultWeightedEdge implements EdgeInterface {
    private String id;
    private Integer length;
    private Node firstNode;
    private Node secondNode;

    public Edge(String id, Node firstNode, Node secondNode) {
        super();
        this.id = id;
        this.firstNode = firstNode;
        this.secondNode = secondNode;
    }

    @Override
    public void setId(String id) {
        this.id = id;
    }

    @Override
    public String getId() {
        return this.id;
    }

    public void setLength(Integer length) {
        this.length = length;
    }

    public Integer getLength() {
        return length;
    }

    public void setFirstNode(Node firstNode) {
        this.firstNode = firstNode;
    }

    public Node getFirstNode() {
        return firstNode;
    }

    public void setSecondNode(Node secondNode) {
        this.secondNode = secondNode;
    }

    public Node getSecondNode() {
        return secondNode;
    }
}
