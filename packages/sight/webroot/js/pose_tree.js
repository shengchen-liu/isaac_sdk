/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
// A temporal pose tree to store relative coordinate system transformations over time
// This implementation does not support multiple paths between the same coordinate systems. It does
// however allow for multiple "roots". In fact the transformation relationships form an acylic,
// bi-directional, not necessarily fully-connected graph. (In other words: it's a forest).
class PoseTreeImpl {
  constructor() {
    // Dictionary of node_name -> Node
    this.nodes_ = {};
    // Latest time
    this.now_ = 0.0;
    // Create a window displaying the PoseTree
    this.graph_view_ = document.createElement("div");
    this.graph_view_.id = "__win-posetree-graph-view";
    // Create a new window with a callback to call resize() on the cytoscape object.
    let that = this;
    WindowManager().createWindow(this.graph_view_, "PoseTree",
                                 {resize: true, onresize: function (obj) {
      that.graph_view_.style.width = (obj.width-10) + "px";
      that.graph_view_.style.height = "100%";
      if (that.graph_view_.hasOwnProperty("cy")) that.graph_view_.cy.resize();
    }});
    this.cy_graph_ = cytoscape({
      container: this.graph_view_,
      boxSelectionEnabled: false,
      autounselectify: true,
      style: cytoscape.stylesheet()
        .selector('node')
          .css({
            'width': 150,
            'height': 40,
            'shape': 'roundrectangle',
            'border-width': 1,
            'border-color': '#333',
            'border-style': 'solid',
            'content': 'data(name)',
            'color': '#000',
            'font-weight': 'normal',
            'text-valign': 'center',
          })
        .selector('edge')
          .css({
            'curve-style': 'bezier',
            'target-arrow-shape': 'triangle',
            'source-arrow-shape': 'triangle',
            'width': 5,
            'line-color': '#666',
            'target-arrow-color': '#666',
          })
    });
    this.graph_view_.cy = this.cy_graph_;
    this.edge_set_expansion_callbacks_ = [];
    this.edge_set_clear_callbacks_ = [];
  }

  // Returns a defaultFrame() that no one is using
  defaultFrame() {
    return "####";
  }

  // Returns the current application time
  now() {
    return this.now_;
  }

  // Resets the PoseTree
  reset() {
    try {
      this.edge_set_clear_callbacks_.forEach((c) => c());
    } catch (e) {
      console.error(e);
    }
    this.cy_graph_.remove(this.cy_graph_.elements('edge'));
    this.cy_graph_.remove(this.cy_graph_.elements('node'));
    this.nodes_ = {};
  }

  // Loads/updates a pose tree from a config
  loadFromConfig(data) {
    // Data is:
    // {
    //   time: 0.123,
    //   nodes: ["a", "b", ...],
    //   edges: [[lhs_idx, rhs_idx, time, [Pose]], ...]
    // }
    const kLhs = 0;
    const kRhs = 1;
    const kTimestamp = 2;
    const kPose = 3;
    for (let i in data.edges) {
      const edge = data.edges[i];
      const lhs = data.nodes[edge[kLhs]];
      const rhs = data.nodes[edge[kRhs]];
      const time = edge[kTimestamp];
      const pose = edge[kPose];
      const qw = pose[0];
      const qx = pose[1];
      const qy = pose[2];
      const qz = pose[3];
      const tx = pose[4];
      const ty = pose[5];
      const tz = pose[6];
      const quat = new THREE.Quaternion(qx, qy, qz, qw);  //THREE.js format: Quaternion(x,y,z,w)
      const mat = new THREE.Matrix4().makeRotationFromQuaternion(quat)
                                     .setPosition(new THREE.Vector3(tx, ty, tz));
      this.set(lhs, rhs, mat, time);
    }
    this.now_ = data.time;
  }

  registerEdgeSetClearCallback(callback) {
    this.edge_set_clear_callbacks_.push(callback);
  }

  // Register a callback to be called whenever a new edge is created.
  registerEdgeSetExpansionCallback(callback) {
    this.edge_set_expansion_callbacks_.push(callback);
  }

  // Set a transformation (THREE.Matrix4) between two nodes
  set(lhs, rhs, pose, timestamp) {
    if (lhs === this.defaultFrame() || rhs === this.defaultFrame()) {
      console.log("No connection to the default frame can be made");
      return;
    }
    let node_a = this._getNode(lhs);
    let node_b = this._getNode(rhs);
    if (node_a.edges[node_b.name] === undefined) {
      // Check there is no connection between the nodes
      if (this._areConnectedImpl(node_a, node_b)) {
        console.error('There is already a path between  ' + lhs + ' and ' + rhs + '');
        return;
      }
      node_a.edges[node_b.name] = {
          timeseries: [{ pose: pose, timestamp: timestamp}],
          latest() { return this.timeseries[this.timeseries.length-1]; }
      };
      node_b.edges[node_a.name] = {
          timeseries: [{ pose: new THREE.Matrix4().getInverse(pose), timestamp: timestamp}],
          latest() { return this.timeseries[this.timeseries.length-1]; }
      };
      // Connect the 2 components
      PoseTreeImpl._getNodeParent(node_a).parent = PoseTreeImpl._getNodeParent(node_b);
      this.edge_set_expansion_callbacks_.forEach((c) => c());
      // Update the graph
      this.cy_graph_.add({group: "edges", data: { source: rhs, target: lhs }});
      this._updateCyGraph();
      return;
    }
    // if timestamp comes out of order, just ignore it
    if (timestamp <= node_a.edges[node_b.name].latest().timestamp) return;
    node_a.edges[node_b.name].timeseries.push({
      pose: pose, timestamp: timestamp
    });
    node_b.edges[node_a.name].timeseries.push({
      pose: new THREE.Matrix4().getInverse(pose), timestamp: timestamp
    });
    const kMinHistory = 128;
    // Keep between 128 to 256 poses
    if (node_a.edges[node_b.name].timeseries.length > 2*kMinHistory) {
      node_a.edges[node_b.name].timeseries =
          node_a.edges[node_b.name].timeseries.slice(-kMinHistory);
      node_b.edges[node_a.name].timeseries =
          node_b.edges[node_a.name].timeseries.slice(-kMinHistory);
    }
  }

  // Returns the transformation at a given timestamp between two nodes.
  // If no transformation exists, then null will be returned.
  get(lhs, rhs, timestamp) {
    if (lhs === rhs) return new THREE.Matrix4();
    if (lhs === this.defaultFrame() || rhs === this.defaultFrame()) {
      return null;
    }
    let node_a = this._getNode(lhs);
    let node_b = this._getNode(rhs);
    if (!this._areConnectedImpl(node_a, node_b)) {
      console.warn(lhs + ' and ' + rhs + ' are not connected');
      return null;
    }
    const path = this._getPath(node_a, node_b);
    if (path === null) return null;
    let ans = new THREE.Matrix4();
    if (timestamp === null || timestamp === undefined) {
      timestamp = this.now();
    }
    for (let i = 1; i < path.length; i++) {
      ans.multiply(this._getInterpolatedPose(path[i-1].edges[path[i].name], timestamp));
    }
    return ans;
  }

  // Returns whether there is a connection between two nodes (using name)
  areConnected(lhs, rhs) {
    // Check if the name are the same (needed in case defaultFrame is passed)
    if (lhs === rhs) return true;
    if (!this.hasNode(lhs) || !this.hasNode(rhs)) return false;
    return this._areConnectedImpl(this.nodes_[lhs], this.nodes_[rhs]);
  }

  // Returns whether or not the node exist in the graph
  hasNode(node) {
    if (this.nodes_[node]) return true;
    return false;
  }

  // Returns the list of names of the node present in the graph
  getNodeNames() {
    let ans = [this.defaultFrame()];
    for (let node in this.nodes_) {
      ans.push(node);
    }
    return ans;
  }

  // Returns an array of names of the edges in this PoseTree
  getEdgesNames() {
    let edges = [];
    for (let n in this.nodes_) {
      for (let e in this.nodes_[n].edges) {
        // Lexicographical comparison to avoid duplicate edges
        if (n <= e) {
          const edge = {
            lhs: n,
            rhs: e,
            name: n + '_T_' + e
          }
          edges.push(edge);
        }
      }
    }
    return edges;
  }

  // get the length of the path between two nodes in this PoseTree
  getPathLength(lhs, rhs) {
    if (lhs === rhs) return 0;
    if (!this.hasNode(lhs) || !this.hasNode(rhs) || !this.areConnected(lhs, rhs)) return -1;
    const node_a = this._getNode(lhs);
    const node_b = this._getNode(rhs);
    const path = this._getPath(node_a, node_b);
    return path.length;
  }

  // Find the component parent
  static _getNodeParent(node) {
    if (node.parent === null) return node;
    node.parent = PoseTreeImpl._getNodeParent(node.parent);
    return node.parent;
  }

  // Returns the pose of an edge at a given time
  _getInterpolatedPose(edge, timestamp) {
    if (edge.timeseries[0].timestamp >= timestamp) return edge.timeseries[0].pose;
    if (edge.latest().timestamp <= timestamp) return edge.latest().pose;
    // TODO impement a binary search
    let i = 1;
    while (edge.timeseries[i].timestamp <= timestamp) i++;
    let p = (timestamp - edge.timeseries[i-1].timestamp) /
            (edge.timeseries[i].timestamp - edge.timeseries[i-1].timestamp);
    return poseInterpolation(edge.timeseries[i-1].pose, edge.timeseries[i].pose, p);
  }

  // Returns whether there is a connection between two nodes (using the node directly)
  _areConnectedImpl(node_a, node_b) {
    return PoseTreeImpl._getNodeParent(node_a) === PoseTreeImpl._getNodeParent(node_b);
  }

  // Adds a new node if it does not exist or return the existing one.
  _getNode(node) {
    if (!this.hasNode(node)) {
      const new_node = {
        name: node,    // Name of the node
        parent: null,  // To know whih connected component it belongs to
        edges: {},     // List of edges
        path: {}       // Cache for the path between this node and other node
      };
      this.nodes_[node] = new_node;
      // Update the graph
      this.cy_graph_.add({
        group: "nodes",
        data: {id: node, name: node},
        style: {'background-color': 'rgb(0, 182, 0)'}
      });
      this._updateCyGraph();
    }
    return this.nodes_[node];
  }

  // Returns a path from a to b and cache it
  _getPath(node_a, node_b) {
    if (node_a.path[node_b.name] !== undefined && node_a.path[node_b.name] !== null) {
      return node_a.path[node_b.name];
    }
    // Keep the direction where a node came from (to recover the path)
    let direction = {};
    // Keep track of the node which were visited
    let visited = {};
    // List of open node to visit
    let open = [node_a.name];
    visited[node_a.name] = true;
    // While we have open node we process them (in dfs order)
    while (open.length > 0) {
      let current = this.nodes_[open.pop()];
      // Go through the list of neighbors of the current node
      for (let node in current.edges) {
        if (visited[node]) continue;
        visited[node] = true;
        direction[node] = current.name;
        open.push(node);
        // If we reached or target, we backtrack to find the path
        if (node === node_b.name) {
          let ans = [];
          while (node !== undefined && node !== null) {
            ans.push(this._getNode(node));
            node = direction[node];
          }
          // Cache the path
          node_b.path[node_a.name] = ans;
          node_a.path[node_b.name] = ans.slice().reverse();
          return node_a.path[node_b.name];
        }
      }
    }
    return null;
  }

  _updateCyGraph() {
    this.cy_graph_.layout({name: 'breadthfirst'}).run();
  }
};

// Returns the unique PoseTree
let pose_tree_ = null;
function PoseTree() {
  if (pose_tree_ === null) {
    pose_tree_ = new PoseTreeImpl();
  }
  return pose_tree_;
}
