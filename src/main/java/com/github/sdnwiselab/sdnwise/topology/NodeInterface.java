package com.github.sdnwiselab.sdnwise.topology;

/**
 * The NodeInterface defines the basic operations for a node in the SDN-WISE topology.
 * It provides methods to get and set the identifier of a node.
 */
public interface NodeInterface {

    /**
     * Gets the unique identifier of the node.
     *
     * @return the ID of the node as a String.
     */
    String getId();

    /**
     * Sets the unique identifier of the node.
     *
     * @param id the new ID of the node as a String.
     */
    void setId(String id);

    /**
     * As far as org.jgrapht.Graph needs to hash nodes, we need to implement hashCode function.
     *
     * @return the hashCode of this Node object.
     */
    int hashCode();
}
