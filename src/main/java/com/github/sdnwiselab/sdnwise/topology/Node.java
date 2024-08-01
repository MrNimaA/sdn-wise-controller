package com.github.sdnwiselab.sdnwise.topology;

import com.github.sdnwiselab.sdnwise.util.NodeAddress;


public class Node implements NodeInterface {
    private String id;
    private Long lastSeen = Long.MAX_VALUE;
    private Integer battery = 0;
    private Integer netId = 0;
    private NodeAddress nodeAddress = null;

    public Node(String id, Long lastSeen, Integer battery, Integer netId, NodeAddress nodeAddress) {
        this.id = id;
        this.lastSeen = lastSeen;
        this.battery = battery;
        this.netId = netId;
        this.nodeAddress = nodeAddress;
    }

    public Node(String id){
        this.id = id;
    }

    public void setBattery(Integer battery) {
        this.battery = battery;
    }

    public void setId(String id) {
        this.id = id;
    }

    public void setLastSeen(Long lastSeen) {
        this.lastSeen = lastSeen;
    }

    public void setNetId(Integer netId) {
        this.netId = netId;
    }

    public void setNodeAddress(NodeAddress nodeAddress) {
        this.nodeAddress = nodeAddress;
    }

    public Integer getBattery() {
        return battery;
    }

    public Integer getNetId() {
        return netId;
    }

    public Long getLastSeen() {
        return lastSeen;
    }

    public NodeAddress getNodeAddress() {
        return nodeAddress;
    }

    public String getId() {
        return id;
    }

    @Override
    public String toString() {
        return this.id;
    }

    @Override
    public int hashCode() {
        return this.id.hashCode();
    }
}