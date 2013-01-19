/**
 * Fixed topology interface helps to fix topology to static one for CTP router.
 * Using this interface user can manually set parent to particular node id.
 * Setting topology to fixed, no routing messages (broadcasts) are sent.
 */
interface FixedTopology {
    /**
     * Sets fixed parent for topology, topology gets fixed.
     * Engine should disable all routing messages and fix topology
     * with parent node.
     * 
     * @param parentNode - id of parent to send data to. If AM_BROADCAST_ADDR then
     * this node is set as root.   
     */
    command void setFixedTopologyParent(am_addr_t parentNode);
    
    /**
     * Disables fixed topology, routing is enabled again, tables are recomputed again
     */
    command void disableFixedTopology();
    
    /**
     * Returns whether topology is fixed
     */
    command bool isTopologyFixed();
}

