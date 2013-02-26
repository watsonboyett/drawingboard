/* 
 * Transient API.js
 * This is an API for John Hardy's Transient Displays Ph.D projcet.
 * 
 * 
 * 
 * 
 * Created: 17th August 2012
 * 
 */

 
/**
 * @brief Converts x pixels to meters.
 * @param f The number of pixels.
 * @param axis "w" or "h".
 * @return f pixels in meters.
 */
function Convert_Pixels2Meters(f, axis) {
	if (axis == "w") return (f * Surface.Width) / window.innerWidth
	if (axis == "h") return (f * Surface.Height) / window.innerHeight
}

/**
 * @brief Converts x meters to pixels.
 * @param f The number of meters.
 * @param axis "w" or "h".
 * @return f meters in pixels.
 */
function Convert_Meters2Pixels(f, axis) {
	if (axis == "w") return (f * (window.innerWidth / Surface.Width))
	if (axis == "h") return (f * (window.innerHeight / Surface.Height))
}

/**
 * @brief Round up a number to a given number of decimal places.
 * @param num The number to round up.
 * @param dec The number of decimal places to round too.
 */
function round(num, dec) {
	return Math.round(num*Math.pow(10,dec))/Math.pow(10,dec);
}


/* Simple JavaScript Inheritance
 * By John Resig http://ejohn.org/
 * MIT Licensed.
 */
// Inspired by base2 and Prototype
(function(){
  var initializing = false, fnTest = /xyz/.test(function(){xyz;}) ? /\b_super\b/ : /.*/;
  // The base Class implementation (does nothing)
  this.Class = function(){};
  
  // Create a new Class that inherits from this class
  Class.extend = function(prop) {
	var _super = this.prototype;
	
	// Instantiate a base class (but only create the instance,
	// don't run the init constructor)
	initializing = true;
	var prototype = new this();
	initializing = false;
	
	// Copy the properties over onto the new prototype
	for (var name in prop) {
	  // Check if we're overwriting an existing function
	  prototype[name] = typeof prop[name] == "function" && 
		typeof _super[name] == "function" && fnTest.test(prop[name]) ?
		(function(name, fn){
		  return function() {
			var tmp = this._super;
			
			// Add a new ._super() method that is the same method
			// but on the super-class
			this._super = _super[name];
			
			// The method only need to be bound temporarily, so we
			// remove it when we're done executing
			var ret = fn.apply(this, arguments);        
			this._super = tmp;
			
			return ret;
		  };
		})(name, prop[name]) :
		prop[name];
	}
	
	// The dummy class constructor
	function Class() {
	  // All construction is actually done in the init method
	  if ( !initializing && this.init )
		this.init.apply(this, arguments);
	}
	
	// Populate our constructed prototype object
	Class.prototype = prototype;
	
	// Enforce the constructor to be what we expect
	Class.prototype.constructor = Class;

	// And make this class extendable
	Class.extend = arguments.callee;
	
	return Class;
  };
})();

/**
 * @brief  KalmanDouble is a float value which is filtered with a KalmanFilter.
 * Usage:
 *
 *    var k = new KalmanDouble(start_value)
 *    k.push(new_value_1)
 *    k.push(new_value_2)
 *    var filtered_value = k.get()
 *
 * @author John Hardy
 */
var KalmanDouble = Class.extend({
	
	A : 1.0,	// Factor of real value to previous real value
	B : 0.0,	// Factor of real value to real control signal
	H : 1.0,	// Factor of measured value to real value
	Q : 0.01,
	
	P : 1.0,		// 
	noise : 0.4,	// Enviromental noise.
	
	value : 0.0,	// The value .
	last : 0.0,		// The last value.
	
	/**
	 * @brief Create a new Kalman filtered double.
	 * @param fStartingValue The starting value for the filter.
	 */
	init: function(fStartingValue)
	{
		this.reset(fStartingValue || 0);
	},
	
	/**
	 * @brief Reset this Kalman filtered double.
	 * @param fStartingValue The starting value for the filter.
	 */
	reset : function(fStartingValue)
	{
		this.LP = 0.1;
		this.value = fStartingValue;
		this.last = fStartingValue;
	},
	
	/**
	 * @brief Push a value into the filter to modify it by it.
	 * @param fValue The value to filter.
	 */
	push : function(fValue)
	{
		// time update - prediction
		this.last = this.A * this.last;
		this.LP = this.A * this.LP * this.A + this.Q;
		
		// measurement update - correction
		var K = this.LP * this.H / (this.H * this.LP * this.H + this.noise);
		this.last = this.last + K * (fValue - this.H * this.last);
		this.LP = (1.0 - K * this.H) * this.LP;
		
		// Store the update.
		this.value = this.last;
	},
	
	/**
	 * @brief Return the current filtered value.
	 */
	get : function()
	{
		return this.value;
	},
});

/**
 * @brief MovingVariance is an average with a moving window of x values.
 * Usage:
 *
 *    var k = new MovingVariance(length)
 *    k.push(new_value_1)
 *    k.push(new_value_2)
 *    var mean = k.mean()
 *    var std = k.std()
 *
 * @author John Hardy
 */
var MovingVariance  = Class.extend({
	
	data : [],
	
	count: 0,
	length: 10,
	idx : 0,
	
	total : 0,
	
	/**
	 * @brief Create a new running variance tracker.
	 */
	init: function(length)
	{
		this.length = length || 25;
		this.reset(this.length);
	},
	
	/**
	 * @brief Reset this running variance tracker.
	 */
	reset : function(length)
	{
		this.data = [];
		for (var i = 0; i < this.length; ++i)
			this.data.push(0);
		this.count = 0;
	},
	
	/**
	 * @brief Push a value so that it is accounted for in this variance tracker.
	 * @param x The value to account for.
	 */
	push : function(x)
	{
		// If we have reached the max number of samples.
		if (this.count == this.length)
		{
			// Subtract the value from the running total.
			//this.vartotal -= Math.pow((this.data[this.idx] - this.mean()), 2);
			this.total -= this.data[this.idx];
		}
		
		// Insert the new value.
		this.data[this.idx] = x;
		
		// Calculate some totals.
		this.total += x;
		//this.vartotal += Math.pow((this.data[this.idx] - this.mean()), 2);
		
		// Increment the index.
		++this.idx;
		
		// Increment the counter.
		++this.count;
		if (this.count > this.length)
			this.count = this.length;
		
		// If we have hit the end, reset the index.
		if (this.idx == this.length)
			this.idx = 0;
	},
	
	/**
	 * @brief Return the number of values used to compute this average.
	 * @return An integer.
	 */
	numvalues : function() { return this.count; },
	
	/**
	 * @brief Return the mean of the values pushed to this running average.
	 * @return A float.
	 */
	mean : function() { return (this.count > 0) ? this.total / this.count : 0.0; },
	
	/**
	 * @brief Return the mean of the values pushed to this running average.
	 * @return A float.
	 */
	variance : function()
	{
		// TODO: Optimise this.
		// return this.vartotal / this.count;
		
		var fMean = this.mean();
		var i = this.count;
		var v = 0;
		while (i--)
		{
			v += Math.pow((this.data[i] - fMean), 2);
		}
		v /= this.count;
		return v;
	},
	
	/**
	 * @brief Return the mean of the values pushed to this running average.
	 * @return A float.
	 */
	std : function() { return Math.sqrt(this.variance()); },
	
});

/**
 * @brief KnuthVariance is a continuious fixed-memory running average.
 * Based on this: http://www.johndcook.com/standard_deviation.html
 * Usage:
 *
 *    var k = new MovingVariance(length)
 *    k.push(new_value_1)
 *    k.push(new_value_2)
 *    var mean = k.mean()
 *    var std = k.std()
 *
 * @author John Hardy
 */
var KnuthVariance = Class.extend({
	
	n : 0,		// The number of values considered.
	
	_oldM : 0,	// The old mean.
	_newM : 0,	// The new mean.
	_oldS : 0,	// The old variance.
	_newS : 0,	// The new variance.
	
	/**
	 * @brief Create a new running variance tracker.
	 */
	init: function()
	{
		this.reset();
	},
	
	/**
	 * @brief Reset this running variance tracker.
	 */
	reset : function()
	{
		this.n = 0;
	},
	
	/**
	 * @brief Push a value so that it is accounted for in this variance tracker.
	 * @param x The value to account for.
	 */
	push : function(x)
	{
		this.n++;
		
		// See Knuth TAOCP vol 2, 3rd edition, page 232
		if (this.n == 1)
		{
			this._oldM = this._newM = x;
			this._oldS = 0.0;
		}
		else
		{
			this._newM = this._oldM + (x - this._oldM) / this.n;
			this._newS = this._oldS + (x - this._oldM) * (x - this._newM);
			
			// set up for next iteration
			this._oldM = this._newM; 
			this._oldS = this._newS;
		}
	},
	
	/**
	 * @brief Return the number of values used to compute this average.
	 * @return An integer.
	 */
	numvalues : function() { return this.n; },
	
	/**
	 * @brief Return the mean of the values pushed to this running average.
	 * @return A float.
	 */
	mean : function() { return (this.n > 0) ? this._newM : 0.0; },
	
	/**
	 * @brief Return the mean of the values pushed to this running average.
	 * @return A float.
	 */
	variance : function() { return ( (this.n > 1) ? this._newS / (this.n - 1) : 0.0 ); },
	
	/**
	 * @brief Return the mean of the values pushed to this running average.
	 * @return A float.
	 */
	std : function() { return Math.sqrt(this.variance()); },
	
});

/**
 * @brief kdTree is a basic but super fast JavaScript implementation of
 * the k-dimensional tree data structure.
 * @see https://github.com/ubilabs/kd-tree-javascript/blob/master/Readme.md
 * 
 * Copyright (c) 2012 Ubilabs
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
function kdTree(points, metric, dimensions) {

  function Node(obj, d, parent) {
    this.obj = obj;
    this.left = null;
    this.right = null;
    this.parent = parent;
    this.dimension = d;
  }

  var self = this;
  this.root = buildTree(points, 0, null);

  function buildTree(points, depth, parent) {
    var dim = depth % dimensions.length;
    if(points.length == 0) return null;
    if(points.length == 1) return new Node(points[0], dim, parent);

    points.sort(function(a,b){ return a[dimensions[dim]] - b[dimensions[dim]]; });

    var median = Math.floor(points.length/2);
    var node = new Node(points[median], dim, parent);
    node.left = buildTree(points.slice(0,median), depth+1, node);
    node.right = buildTree(points.slice(median+1), depth+1, node);
    return node;
  }

  this.insert = function(point) {
    var insertPosition = innerSearch(self.root, null);

    if(insertPosition == null) {
      self.root = new Node(point, 0, null);
      return;
    }

    var newNode = new Node(point, (insertPosition.dimension+1)%dimensions.length, insertPosition);
    var dimension = dimensions[insertPosition.dimension];
    if(point[dimension] < insertPosition.obj[dimension]) {
      insertPosition.left = newNode;
    } else {
      insertPosition.right = newNode;
    }

    function innerSearch(node, parent) {
      if(node == null) return parent;

      var dimension = dimensions[node.dimension];
      if(point[dimension] < node.obj[dimension]) {
        return innerSearch(node.left, node);
      } else {
        return innerSearch(node.right, node);
      }
    }
  }

  this.remove = function(point) {
    var node = nodeSearch(self.root);
    if(node == null) return;

    removeNode(node);
    function nodeSearch(node) {
      if(node == null) return null;
      if(node.obj === point) return node;

      var dimension = dimensions[node.dimension];
      if(point[dimension] < node.obj[dimension]) {
        return nodeSearch(node.left, node);
      } else {
        return nodeSearch(node.right, node);
      }
    }

    function removeNode(node) {
      if(node.left == null && node.right == null) {
        if(node.parent == null) {
          self.root = null;
          return;
        }
        var pDimension = dimensions[node.parent.dimension];
        if(node.obj[pDimension] < node.parent.obj[pDimension]) {
          node.parent.left = null;
        } else {
          node.parent.right = null;
        }
        return;
      }

      if(node.left != null) {
        var nextNode = findMax(node.left, node.dimension);
      } else {
        var nextNode = findMin(node.right, node.dimension);
      }
      var nextObj = nextNode.obj;
      removeNode(nextNode);
      node.obj = nextObj;

      function findMax(node, dim) {
        if(node == null) return null;

        var dimension = dimensions[dim];
        if(node.dimension == dim) {
          if(node.right != null) return findMax(node.right, dim);
          return node;
        }

        var own = node.obj[dimension]
        var left = findMax(node.left, dim);
        var right = findMax(node.right, dim);
        var max = node;
        if(left != null && left.obj[dimension] > own) max = left;
        if(right != null && right.obj[dimension] > max.obj[dimension]) max = right;
        return max;
      }

      function findMin(node, dim) {
        if(node == null) return null;

        var dimension = dimensions[dim];
        if(node.dimension == dim) {
          if(node.left != null) return findMin(node.left, dim);
          return node;
        }

        var own = node.obj[dimension]
        var left = findMin(node.left, dim);
        var right = findMin(node.right, dim);
        var min = node;
        if(left != null && left.obj[dimension] < own) min = left;
        if(right != null && right.obj[dimension] < min.obj[dimension]) min = right;
        return min;
      }
    }
  }

  this.nearest = function(point, maxNodes, maxDistance) {
    bestNodes = new BinaryHeap(function(e){ return -e[1]; });
	
    if(maxDistance) {
      for(var i=0; i<maxNodes; i++) {
        bestNodes.push([null, maxDistance]);
      }
    }
    nearestSearch(self.root);

    function nearestSearch(node) {
      var bestChild;
      var dimension = dimensions[node.dimension];
      var ownDistance = metric(point, node.obj);

      var linearPoint = {};
      for(var i=0; i<dimensions.length; i++) {
        if(i == node.dimension) {
          linearPoint[dimensions[i]] = point[dimensions[i]];
        } else {
          linearPoint[dimensions[i]] = node.obj[dimensions[i]];
        }
      }
      var linearDistance = metric(linearPoint, node.obj);

      if(node.right == null && node.left == null) {
        if(bestNodes.size() < maxNodes || ownDistance < bestNodes.peek()[1]) {
          saveNode(node, ownDistance);
        }
        return;
      }

      if(node.right == null) {
        bestChild = node.left;
      } else if(node.left == null) {
        bestChild = node.right;
      } else {
        if(point[dimension] < node.obj[dimension]) {
          bestChild = node.left;
        } else {
          bestChild = node.right;
        }
      }

      nearestSearch(bestChild);

      if(bestNodes.size() < maxNodes || ownDistance < bestNodes.peek()[1]) {
        saveNode(node, ownDistance);
      }

      if(bestNodes.size() < maxNodes || Math.abs(linearDistance) < bestNodes.peek()[1]) {
        var otherChild;
        if(bestChild == node.left) {
          otherChild = node.right;
        } else {
          otherChild = node.left;
        }
        if(otherChild != null) nearestSearch(otherChild);
      }

      function saveNode(node, distance) {
        bestNodes.push([node, distance]);
        if(bestNodes.size() > maxNodes) {
          bestNodes.pop();
        }
      }
    }

    var result = [];
    for(var i=0; i<maxNodes; i++) {
		//if (bestNodes.content[i] == undefined) // TODO FIXME
		//	continue;
      if(bestNodes.content[i][0]) { //if(bestNodes.content[i][0]) 
        result.push([bestNodes.content[i][0].obj, bestNodes.content[i][1]]);
		//console.log("wo");
      }
    }
    return result;
  }

  this.balanceFactor = function() {
    return height(self.root)/(Math.log(count(self.root))/Math.log(2));

    function height(node) {
      if(node == null) return 0;
      return Math.max(height(node.left), height(node.right)) + 1;
    }

    function count(node) {
      if(node == null) return 0;
      return count(node.left) + count(node.right) + 1;
    }
  }

  // Binary heap implementation from:
  // http://eloquentjavascript.net/appendix2.html
  function BinaryHeap(scoreFunction){
    this.content = [];
    this.scoreFunction = scoreFunction;
  }

  BinaryHeap.prototype = {
    push: function(element) {
      this.content.push(element);
      this.bubbleUp(this.content.length - 1);
    },

    pop: function() {
      var result = this.content[0];
      var end = this.content.pop();
      if (this.content.length > 0) {
        this.content[0] = end;
        this.sinkDown(0);
      }
      return result;
    },

    peek: function() {
      return this.content[0];
    },

    remove: function(node) {
      var len = this.content.length;
      for (var i = 0; i < len; i++) {
        if (this.content[i] == node) {
          var end = this.content.pop();
          if (i != len - 1) {
            this.content[i] = end;
            if (this.scoreFunction(end) < this.scoreFunction(node))
              this.bubbleUp(i);
            else
              this.sinkDown(i);
          }
          return;
        }
      }
      throw new Error("Node not found.");
    },

    size: function() {
      return this.content.length;
    },

    bubbleUp: function(n) {
      var element = this.content[n];
      while (n > 0) {
        var parentN = Math.floor((n + 1) / 2) - 1,
        parent = this.content[parentN];
        if (this.scoreFunction(element) < this.scoreFunction(parent)) {
          this.content[parentN] = element;
          this.content[n] = parent;
          n = parentN;
        }
        else {
          break;
        }
      }
    },

    sinkDown: function(n) {
      var length = this.content.length,
      element = this.content[n],
      elemScore = this.scoreFunction(element);

      while(true) {
        var child2N = (n + 1) * 2, child1N = child2N - 1;
        var swap = null;
        if (child1N < length) {
          var child1 = this.content[child1N],
          child1Score = this.scoreFunction(child1);
          if (child1Score < elemScore)
            swap = child1N;
        }
        if (child2N < length) {
          var child2 = this.content[child2N],
          child2Score = this.scoreFunction(child2);
          if (child2Score < (swap == null ? elemScore : child1Score))
            swap = child2N;
        }

        if (swap != null) {
          this.content[n] = this.content[swap];
          this.content[swap] = element;
          n = swap;
        }
        else {
          break;
        }
      }
    }
  };
}

/**
 * @brief The dbScan algorithm (density-based spatial clustering of applications with noise)
 * clusters points based on density.
 * @see http://en.wikipedia.org/wiki/DBSCAN
 *
 * Usage: 
 *    var clusters = dbScan(lData, DebugVoxelWidth * 0.25, 5, true)
 *    for (var i = 0; i < clusters.length; ++i) {
 *          console.log(clusters[i].centerx) // centerx, volume, slope, intercept, aspectratio, volume, radius
 *    }
 * 
 * @author John Hardy
 * @param data A list of data points [{x:3, y:4}, {x:1, y:1}, {x:5, y:0}]
 * @param eps The scanning distance threshold for each cluster.
 * @param minpts The minimum number of points required to be part of a cluster (the yellow ones on the wikipedia page top diagram)
 * @param editdata Controls if we copy the data or not.  If true, the data will be copied.  False will edit in place (and add meta-data) e.g. data[i].noise, data[i].cluster, data[i].visited etc
 * @return A list of clusters. Clusters are defined by the Cluster type within.
 */
function dbScan(data, eps, minpts, editdata) {
	
	// Create variables that store the number of 
	var lClusters = [];
	var bWriteToData = editdata || false;
	
	// Define a cluster.
	function Cluster() {
		
		// List of points.
		this.points = [];
		
		// AABB.
		this.minx = 999999999;
		this.miny = 999999999;
		this.maxx = -999999999;
		this.maxy = -999999999;
		
		// Running average (we don't know how many points there will be).
		this.avgx = new KnuthVariance();
		this.avgy = new KnuthVariance();
		
		// Variables for the line of best fit.
		this.sigmaX = 0;
		this.sigmaY = 0;
		this.sigmaX2 = 0;
		this.sigmaXY = 0;
		
		this.addPoint = function (p) {
			// Add it to the list of points.
			this.points.push(p);
			
			// Store the AABB.
			this.minx = Math.min(p.x, this.minx);
			this.miny = Math.min(p.y, this.miny);
			this.maxx = Math.max(p.x, this.maxx);
			this.maxy = Math.max(p.y, this.maxy);
			
			// Compute some averages for it etc.
			this.avgx.push(p.x);
			this.avgy.push(p.y);
			
			// Line of best fit.
			this.sigmaX += p.x;
			this.sigmaY += p.y;
			this.sigmaX2 += p.x*p.x;
			this.sigmaXY += p.x*p.y;
		}
		
		/** @brief Calculate the slope of all the points in this cluster. */
		this.slope = function() {
			return ((this.points.length * this.sigmaXY) - (this.sigmaX * this.sigmaY)) / ((this.points.length * this.sigmaX2) - (this.sigmaX * this.sigmaX));
		}
		
		/** @brief Calculate the intercept of all the points in this cluster. */
		this.intercept = function() {
			return (this.sigmaY - (this.slope() * this.sigmaX)) / this.points.length;
		}
		
		/** @brief Calculate the aspect ratio of all the points in this cluster. */
		this.aspectratio = function () {
			if (this.points.length == 1)
				return 1.0;
			return (this.maxx - this.minx) / (this.maxy - this.miny);
		}
		
		/** @brief Calculate the volume taken up by all the points in this cluster. */
		this.volume = function() {
			var w = (this.maxx - this.minx);
			var h = (this.maxy - this.miny);
			return w * h;
		}
		
		/** @brief Calculate the average x-value of all the points in this cluster. */
		this.centerx = function() {
			return this.avgx.mean();
		}
		
		/** @brief Calculate the average y-value of all the points in this cluster. */
		this.centery = function() {
			return this.avgy.mean();
		}
		
		/** @brief Calculate the radius of all the points in this cluster. */
		this.radius = function() {
			// Get the center point.
			x = this.centerx();
			y = this.centerx();
			
			// Compute square distance to max.
			var fMin = Math.abs(Math.pow(this.minx - x, 2) +  Math.pow(this.miny - y, 2));
			
			// Compute square distance to min.
			var fMax = Math.abs(Math.pow(this.maxx - x, 2) +  Math.pow(this.maxy - y, 2));
			
			// Return the length of the largest square distance.
			return Math.sqrt(Math.max(fMax, fMin));
		}
	}
	
	// Define a euclidan distance function for comparing epsilons.
	var treeDistance = function(a, b){
		return Math.pow(a.x - b.x, 2) +  Math.pow(a.y - b.y, 2);
	}
	
	// Work out the n nearest points to our points (n = 100, but you can pick whatever).
	function regionQuery(dPoint) {
		// Query the kd-tree for the nearest n points. n = 100 by default
		var lNearest = tree.nearest(dPoint, 100, eps * eps); // TODO: treedistance uses squared distance.
		
		// Discard the distance information returned by the tree.
		var lPoints = [];
		for (var i = 0; i < lNearest.length; ++i)
			lPoints.push(lNearest[i][0]);
		
		// Return the points.
		return lPoints;
	}
	
	// Expand a cluster by checking all the neigbours of neigbours.  Minesweeper stack based for speed.
	function expandCluster(dPoint, lNeighbours, pCluster) {
		// Add the point to the cluster.
		pCluster.points.push(dPoint);
		dPoint.cluster = pCluster;
		
		// For each neighbour.
		while((dNPoint = lNeighbours.pop()) != undefined)
		{
			// If we have not visited this neigbour yet.
			if (!!! dNPoint.visited)
			{
				// Mark it as visited.
				dNPoint.visited = true;
				
				// Add its neigbours into the cluster, if they have enough neigbours.
				var n = regionQuery(dNPoint);
				if (n.length >= minpts)
				{
					// Put them onto the stack, if they are not already visited.
					lNeighbours = lNeighbours.concat(n.filter(function(pt) { return !!! pt.visited; }));
				}
			}
			
			// If we are not a member of a cluster yet, make us a member.
			if (!!! dNPoint.cluster)
			{
				pCluster.addPoint(dNPoint);
				dNPoint.cluster = pCluster;
			}
		}
	}
	
	// If we don't want to modify the current collection.
	var data2 = data;
	if (!bWriteToData) {
		data2 = [];
		for (var i = 0; i < data.length; i++)
			data2.push({x: data[i].x, y: data[i].y, cluster: null});
	}
	
	
	// Build a kd-tree of points.
	
	var tree = new kdTree(data2, treeDistance, ["x", "y"]);
	
	// For each unvisited point P in dataset D.
	for (var i = 0; i < data2.length; ++i) {
		// Bail if the point is visited.
		if (!! data2[i].visited)
			continue;
		
		// Mark it as visited.
		data2[i].visited = true;
		
		// Return all points within P's neigbourhood.
		var n = regionQuery(data2[i]);
		
		// If we have less than minpts neighbours, then flag as noise.
		if (n.length < minpts)
		{
			data2[i].noise = true;
		}
		
		// Otherwise, generate us a new cluster.
		else
		{
			var cluster = new Cluster();
			lClusters.push(cluster);
			expandCluster(data2[i], n, cluster);
		}
	}
	
	// Return all the clusters we generated.
	return lClusters;
}

/**
 * @brief A TouchPoint manages a tracked touch point.  This accounts for
 * smoothing, position, strong locks (i.e. how sure are we that it exists)
 * and debugging data.
 * @author John Hardy
 */
function TouchPoint(processor, cluster) {
	
	this.processor = processor;                   // The TouchTracker that created us.
	this.colour = this.processor._nextColour();   // The debug colour for this point.
	
	this.avgx = cluster.centerx();                // The raw cluster x.
	this.smoothx = new KalmanDouble(cluster.centerx());  // The smoothed cluster x.
	this.smoothx.noise = 0.05;
	
	this.avgy = cluster.centery();                // The raw cluster y.
	this.smoothy = new KalmanDouble(cluster.centery());  // The smoothed cluster y.
	this.smoothy.noise = 0.05;
	
	this.lastcluster = cluster;                   // The reference to the last cluster of points.
	this.stronglock = false;                      // Is this touchpoint definately not just noise.
	
	this.id = this.processor.TrackerCount++;      // The id for this touch point.
	this.iFrameCount = 0;                         // The number of touch frames it has processed.
	
	this.lastupdate = new Date().getTime();       // The last time it recieved an update.
	
	this.iDeathTracker = 0;
	
	
	/** @brief The raw center of this touch point on the x axis. */
	this.centerx = function() {
		return this.avgx;//.get();
	}
	
	/** @brief The raw center of this touch point on the y axis. */
	this.centery = function() {
		return this.avgy;//.get();
	}
	
	/** @brief The raw center of this touch point on the x axis. */
	this.x = function() {
		return this.smoothx.get() / 100; // put us back into 0-1 space!
	}
	
	/** @brief The raw center of this touch point on the y axis. */
	this.y = function() {
		return this.smoothy.get() / 100;// put us back into 0-1 space!
	}
	
	// Add a visual and label to the debug trackers layer.
	this._visual = null;
	this._label = null;
	
	/**
	 * @brief Tell this touch point to consume a cluster.
	 */
	this.consume = function(cluster) {
		
		// Update the number of frames this point has processed.
		this.iFrameCount++;
		//this.lastupdate = new Date().getTime();
		
		// Update the tracker by this cluster.
		this.avgx = cluster.centerx();
		this.smoothx.push(cluster.centerx());
		
		this.avgy = cluster.centery();
		this.smoothy.push(cluster.centery());
		
		// Store the last cluster.
		this.lastcluster = cluster;
		
		this.iDeathTracker = 0;
		
		// If we are a strong lock.
		if (this.iFrameCount == this.processor.FramePersistance)
		{
			// If we have had x updates, set us to a strong tracker.
			this.stronglock = true;
			
			// Dispatch the start signal.
			if (this.processor.DispatchStart != null) {
				this.processor.DispatchStart(this);
			}
			
			// Add some visuals to the static debug layer.
			if (this.processor._DebugLayerStatic)
			{
				// Add us a new visual which we can move around.
				this._visual = document.createElement("div");
				this.processor._DebugLayerStatic.appendChild(this._visual);
				
				//this._visual.style.zIndex = "999";
				this._visual.style.pointerEvents= "none";
				this._visual.style.position= "absolute";
				
				this._visual.style.left = this.centerx() + "%";
				this._visual.style.top  = this.centery() + "%";
				this._visual.style.width = this.processor.DebugVoxelWidth * 2 + "px";
				this._visual.style.height = this.processor.DebugVoxelHeight * 2 + "px";
				this._visual.style.marginTop = -this.processor.DebugVoxelHeight + "px";
				this._visual.style.marginLeft = -this.processor.DebugVoxelWidth + "px";
				
				this._visual.style.borderRadius = this.processor.DebugVoxelWidth * 2 + "px";
				this._visual.style.backgroundColor = this.colour;
				this._visual.style.opacity = 0.8;
				this._visual.style.border="1px solid white";
				
				// And the label.
				this._label = null; /*document.createElement("div");
				this._label.innerHTML = this.id;
				this.processor._DebugLayerStatic.appendChild(this._label);
				
				this._label.style.pointerEvents= "none";
				this._label.style.position= "absolute";
				this._label.style.left = this.centerx() + "%";
				this._label.style.top  = this.centery() + "%";
				this._label.style.marginTop = "-40px";
				this._label.style.color = "yellow";
				
				/*
				this._label = $("<div>").css({
						position: "absolute",
						left : this.centerx() + "%",
						top  : this.centery() + "%",
						"margin-top" : "-40px",
						color: "yellow",
				}).appendTo(this.processor._DebugLayerStatic).text(this.id);
				*/
			}
		}
		
		// Don't dispatch move updates unless we have a strong lock.
		if (this.stronglock) {
			if (this.processor.DispatchMove != null) {
				this.processor.DispatchMove(this);
			}
		}
		
		// If debugging is enabled.
		if (this.processor._DebugLayerStatic) {
			// Update the visuals.
			if (this._visual) {
				this._visual.style.left = this.smoothx.get() + "%";
				this._visual.style.top = this.smoothy.get() + "%";
				/*this._visual.css({
					left : this.smoothx.get() + "%",
					top  : this.smoothy.get() + "%",
				});*/
			}
			if (this._label) {
				this._label.style.left = this.smoothx.get() + "%";
				this._label.style.top = this.smoothy.get() + "%";
				/*
				this._label.css({
					left : this.smoothx.get() + "%",
					top  : this.smoothy.get() + "%",
				});*/
			}
		}
	}
	
	/**
	 * @brief Check to see if this touch point should be deleted.
	 * @return True to flag for deletion, false to keep alive.
	 */
	this.consumeNothing = function() {
		// If we have four consecutive frames with NO input, consider us dead.
		if (this.iDeathTracker >= 4)
			return true;
		this.iDeathTracker++;
		return false;
		// Work out the time since the last input.
		//var elapsed = new Date().getTime() - this.lastupdate;
		//if (elapsed > this.processor.TouchRemoveTime)
		//	return true;
		//return false;
		
		// Suggest the tracker should die.
		//return false; // keep it alive.
		//return true; // kill it off
	}
	
	/**
	 * @brief Remove this touch point, fire events and remove visual debug.
	 */
	this.remove = function() {
	
		// Don't dispatch if we are not a strong lock.
		if (this.stronglock) {
			if (this.processor.DispatchStop != null) {
				this.processor.DispatchStop(this);
			}
		}
		
		// Remove debug visuals.
		if (this._visual)
			this.processor._DebugLayerStatic.removeChild(this._visual);
			//this._visual.remove();
		
		if (this._label)
			this.processor._DebugLayerStatic.removeChild(this._label);
			//this._label.remove();
	}
	
	/*
	this.drawCluster = function() {
		// For each point in this cluster, draw it.
		var points = this.lastcluster.points;
		for (var iPt = 0; iPt < points.length; ++iPt)
		{
			$("<div>").addClass("debugSpot").css({
				left : points[iPt].x + "%",
				top  : points[iPt].y + "%",
				width: TouchPoint_DebugVoxelWidth + "px",
				height: TouchPoint_DebugVoxelHeight + "px",
				"background-color": (this.stronglock) ? this.colour : "#CCC",
				opacity : "0.3",
			}).appendTo(pLayer);
		}
	}
	*/
}

/**
 * @brief TouchTracker turns point cloud data into touch events.
 * You will only need one of these.  You can also merge point cloud data
 * although you will need to transform it yourself if you do so.
 * Usage:
 *    var t = new TouchTracker()
 * @author John Hardy
 */
var TouchTracker = Class.extend({
	
	/**
	 * @brief The number of frames to wait before we consider a tracker as active and working.
	 * Default = 2.  Setting this to 1 makes it instantly start the tracker. 
	 * The lower this number, the faster it will react to a finger.  However, it also makes it more succeptibe to noise.
	 */
	FramePersistance : 2,
	
	/** @brief The number of trackers we have ever had active.  Gives each 'touch' an id. */
	TrackerCount : 0,
	
	/** @brief The number of milliseconds to wait before removing the touch point if no data. */
	TouchRemoveTime : 100,
	
	/** @brief The kinect point voxel width in pixels. This is set when updateSurface() is called. */
	DebugVoxelWidth : 2,
	
	/** @brief The kinect point voxel height in pixels. This is set when updateSurface() is called. */
	DebugVoxelHeight : 2,
	
	/** @brief The size of a kinect voxel. Default = 5mm. */
	KinectVoxel : 0.005,
	
	/** @brief The distance at which we can be sure the inputs are not the same (from frame to frame).  Given in meters. */
	DistanceCutOff : 0.07,
	
	/** @brief The size of the largest finger (in meters).  Anything larger will not be detected as a finger. */
	LargestFinger  : 0.04,
	
	/** The debug colours for the touch points. */
	DebugColours : ["red", "green", "blue", "yellow", "orange", "lime"],
	
	/** @brief Is debug mode enabled or not. */
	_Debug : false,
	
	/** @brief A layer that sits atop everything which we can draw debug data onto. */
	_DebugLayer : null,
	
	/** @brief A list of tracked touch points. */
	_Tracked : [],
	
	/** @brief A function that is called with all the newly created touch points. */
	DispatchStart : null,
	
	/** @brief A function that is called with all the old touch points. */
	DispatchStop  : null,
	
	/** @brief A function that is called with all the updated touch points. */
	DispatchMove  : null,
	
	
	/**
	 * @brief Create a new touch tracker.
	 */
	init: function(dParams) {
		// Parse out arguments.
		this.debug(dParams.debug || false);
		this.trails(dParams.trails || false);
		
		// Parse out handlers.
		this.DispatchStart = (dParams.start || null);
		this.DispatchStop = (dParams.stop || null);
		this.DispatchMove = (dParams.update || null);
		
		// Setup the surface.
		this.updateSurface();
	},
	
	/**
	 * @brief Process a set of points and dispatch touch points.
	 * @param lPoints Points in the format: [{x:3, y:1, z:3}, {x:1, y:2, z:3}]
	 */
	process : function(lPoints) {
		
		// Update the voxel sizes.
		this.updateSurface();
		
		// Hide everything on the debug layer.
		if (this._DebugLayer)
		{
			// this._DebugLayer.empty();
			while (this._DebugLayer.hasChildNodes()) {
				this._DebugLayer.removeChild(this._DebugLayer.lastChild);
			}
		}
		
		// Perform the DB scan.
		// The idea is to get the smallest distance (~2.5) with the largest minpoints (~5). ~3 works well.
		// 3 * 100 = 300 BECAUSE we are not in 0-1 space but 0-100 space.
		var fBest = (300 * this.DebugVoxelWidth) / window.innerWidth;  // 300 here is the scale factor on the voxel size.
		
		var lClusters = dbScan(lPoints, fBest, 5, true); // eps = 2.5 // this.DebugVoxelWidth * 0.25
		var lTracked = this._Tracked;
		
		// Classify the fingers from clusters.
		var lFingers = [];
		for (var i = 0; i < lClusters.length; ++i) {
			
			// Phase 1: Volume (disregard very large clusters)
			var fVolume = lClusters[i].volume();
			
			// Make it correct size for voxel.
			var fEstX = 100 * Convert_Meters2Pixels(this.LargestFinger, "w") / window.innerWidth;
			var fEstY = 100 * Convert_Meters2Pixels(this.LargestFinger, "h") / window.innerWidth; // TODO check innerHeight?
			var fEst = fEstX * fEstY;
			// 10 * 100 = 1000 BECAUSE we are not in 0-1 space but 0-100 space.
			//var fEst = ((1000 * this.DebugVoxelWidth * this.DebugVoxelWidth) / $(document).width());
			//$("#surf_name").text(Math.round(fVolume, 5) + ", " + Math.round(fEst, 5));
			if (fVolume > fEst) // fVolume > 60
				continue;
			
			// Disregard those with a given s.
			// TODO: Use residuals to detect "tightness" of the line.
			// http://en.wikipedia.org/wiki/File:Correlation_examples2.svg
			//var vSlope = Math.abs(lClusters[i].slope());
			//console.log(vSlope);
			//if (vSlope > 1.0)
			//	continue;
			
			// Now, this cluster passed the classifer so accept it as a finger!
			lFingers.push(lClusters[i]);
		}
		
		// Generate a list of ranked matches. FIX: Distance cut off.
		var lRanks = [];
		for (var i = 0; i < lTracked.length; ++i) {
			var pTracker = lTracked[i];
			for (var j = 0; j < lFingers.length; ++j) {
				var pCluster = lFingers[j];
				
				// Rank the two based on distance.
				var fRank = Math.pow(pCluster.centerx() - pTracker.centerx(), 2) +  Math.pow(pCluster.centery() - pTracker.centery(), 2);
				
				// TODO: Store the pair if the distance is not crazy. * 10 * 100 = 1000 BECAUSE we are not in 0-1 space but 0-100 space.
				//var fDist = 1000 * (this.DebugVoxelWidth / $(document).width()) ;//30;
				var fDist = 100 * Convert_Meters2Pixels(this.DistanceCutOff, "w") / window.innerWidth;
				if (fRank < (fDist * fDist)) // Square distance.
					lRanks.push({
						cluster : pCluster,
						tracker : pTracker,
						rank : fRank,
					});
			}
		}
		
		// Sort the rank table.
		lRanks = lRanks.sort(function(a, b) {
			if (a.rank < b.rank) return -1;
			if (a.rank > b.rank) return 1;
			return 0;
		});
		
		// Flag all the trackers and clusters as unassigned.
		for (var i = 0; i < lTracked.length; ++i) lTracked[i].assigned = false;
		for (var i = 0; i < lFingers.length; ++i) lFingers[i].assigned = false;
		
		// Match up the best ones.
		for (var i = 0; i < lRanks.length; ++i) {
			// Skip if the tracker or the cluster has been assigned.
			if (lRanks[i].cluster.assigned == true || lRanks[i].tracker.assigned == true)
				continue;
			
			// Assign the best match! woo!
			lRanks[i].tracker.consume(lRanks[i].cluster);
			lRanks[i].cluster.assigned = true;
			lRanks[i].tracker.assigned = true;
		}
		
		// If we have clusters left over, make them into touch points.
		for (var i = 0; i < lFingers.length; ++i) {
			if (lFingers[i].assigned == false)
			{
				// Make a new tracker.
				var pTracker = new TouchPoint(this, lFingers[i]);
				lTracked.push(pTracker);
				
			}
		}
		
		// If we have touch points left, remove them.
		var lRemove = [];
		for (var i = 0; i < lTracked.length; ++i) {
			if (lTracked[i].assigned == false)
			{
				// Suggest this tracker should die as it didn't recieve an update this frame.
				if (lTracked[i].consumeNothing())
					lRemove.push(i);
			}
		}
		
		// Sort so we remove the largest indexes first.
		//  this lets us remove them from the array without screwing up our indices :)
		lRemove.sort();
		lRemove.reverse();
		
		// Remove them from the array.
		for (var i = 0; i < lRemove.length; ++i) {
			if (lTracked[lRemove[i]])
				lTracked[lRemove[i]].remove();
			lTracked.splice(lRemove[i], 1);
		}
		
		// Draw debug data for all the trackers.
		if (this._DebugLayer)
		{
			/**/
			for (var i = 0; i < lTracked.length; ++i)
			{
				var pTouch = lTracked[i];
				
				var points = pTouch.lastcluster.points;
				for (var iPt = 0; iPt < points.length; ++iPt)
				{
					var el = document.createElement("div");
					el.style.position = "absolute";
					el.style.width = this.DebugVoxelWidth + "px";
					el.style.height = this.DebugVoxelHeight + "px";
					el.style.backgroundColor = (pTouch.stronglock) ? pTouch.colour : "#CCC";
					el.style.opacity = "0.3";
					
					el.style.left = points[iPt].x + "%";
					el.style.top = points[iPt].y + "%";
					
					this._DebugLayer.appendChild(el);
				}
			}
			
		}
	},
	
	/**
	 * @brief Enable or disable debug mode.
	 * @param bEnabled Do we want to enable or disable debug mode.
	 */
	debug : function(bEnabled) {
		// Set the variable.
		this._Debug = bEnabled;
		
		// Create a debug layer if needed.
		if (this._Debug)
		{
			this._DebugLayerStatic = document.createElement("div");
			document.body.appendChild(this._DebugLayerStatic);
			
			this._DebugLayerStatic.style.width= "100%";
			this._DebugLayerStatic.style.height= "100%";
			this._DebugLayerStatic.style.top= "0";
			this._DebugLayerStatic.style.left= "0";
			this._DebugLayerStatic.style.zIndex = "999";
			this._DebugLayerStatic.style.pointerEvents = "none";
			this._DebugLayerStatic.style.pointerEvents = "none";
			this._DebugLayerStatic.style.position= "absolute";
		}
		// Otherwise destroy it.
		else
		{
			if (this._DebugLayerStatic)
				document.body.removeChild(this._DebugLayerStatic);
			this._DebugLayerStatic = null;
		}
	},
	
	/**
	 * @brief Enable or disable debug mode.
	 * @param bEnabled Do we want to enable or disable debug mode.
	 */
	trails : function(bEnabled) {
		// Create a debug layer if needed.
		if (bEnabled)
		{
			this._DebugLayer = document.createElement("div");
			document.body.appendChild(this._DebugLayer);
			
			this._DebugLayer.style.width= "100%";
			this._DebugLayer.style.height= "100%";
			this._DebugLayer.style.top= "0";
			this._DebugLayer.style.left= "0";
			this._DebugLayer.style.zIndex = "999";
			this._DebugLayer.style.pointerEvents= "none";
			this._DebugLayer.style.position= "absolute";
		}
		// Otherwise destroy it.
		else
		{
			if (this._DebugLayer)
				document.body.removeChild(this._DebugLayer);
			this._DebugLayer = null;
		}
	},
	
	/**
	 * @brief Update this touch tracker to respect the new Surface.XX settings and page resolution.
	 */
	updateSurface : function() {
		this.DebugVoxelWidth  = this.KinectVoxel * (window.innerWidth  / Surface.Width);
		this.DebugVoxelHeight = this.KinectVoxel * (window.innerHeight / Surface.Height);
	},
	
	/**
	 * @brief Return the colour at the start of the debug colour list, and cycle the list.
	 */
	_nextColour : function() {
		var colour = this.DebugColours.splice(0, 1)[0];
		this.DebugColours.push(colour);
		return colour;
	},
	
	
});


/**
 * A simple class which we can use to inject multi-touch events into a web page.
 * Spec: http://dvcs.w3.org/hg/webevents/raw-file/tip/touchevents.html
 * TODO: Still requires support for [enter, leave and cancel].
 * Based on the work done here: http://smus.com/multi-touch-browser-patch
 *
 * Sample usage:
 *
 *   $('#mything').bind('touchstart', function(e) {
 *          $(this).css({"background-color": "red"});
 *   }); 
 *
 * John Hardy 2012
 *
 */
var MultiTouch = {
	
	START : 3,   // Signals that we have a new touch event.
	MOVE : 4,    // Signals that we have an update for a touch event.
	END : 5,     // Signals that a touch event has finished.
	
	cursors: [],
	
	// Data structure for associating cursors with objects.
	_data: {},
	
	_touchstart: function(touch) { this._create_event('touchstart', touch, {}); },
	_touchmove:  function(touch) { this._create_event('touchmove',  touch, {}); },
	_touchend:   function(touch) { this._create_event('touchend',   touch, {}); },
	
	_create_event: function(name, touch, attrs)
	{
		// Create an event property table.
		// 
		/**/
		var e = document.createEvent('UIEvent');
		e.initUIEvent(name, true, true);
		e.touches = this.cursors;
		e.targetTouches = this._get_target_touches(touch.target);
		e.changedTouches = [touch];
		
		
		/*
		var e = jQuery.Event(name, {
			bubbles : true,
			cancelable : true,
			
			touches : this.cursors,
			targetTouches : this._get_target_touches(touch.target),
			changedTouches : [touch],
			//originalEvent : touch,
		});
		*/
		
		// If we have a target element.
		if (touch.target)
		{
			//$(touch.target).trigger(e);
			touch.target.dispatchEvent(e);
		}
		// If we do not, simply dispatch on the document.
		else
		{
			//$(document).trigger(e);
			document.dispatchEvent(e);
		}
	},
	
	_get_target_touches: function(element)
	{
		var targetTouches = [];
		for (var i = 0; i < this.cursors.length; i++)
		{
			var touch = this.cursors[i];
			if (touch.target == element)
			{
				targetTouches.push(touch);
			}
		}
		return targetTouches;
	},
	
	
	inject: function(type, sid, x, y, devicetouch)
	{
		var dTouch;
		
		// If we are not creating a new touch event, use our existing data.
		if (type !== MultiTouch.START)
		{
			dTouch = this._data[sid];
		}
		
		// Otherwise, generate a new one and use that.
		else
		{
			dTouch =  { sid: sid };
			this._data[sid] = dTouch;
		}
		
		// Update the properties as per the spec: 
		// See http://dvcs.w3.org/hg/webevents/raw-file/tip/touchevents.html
		dTouch.identifier = sid;
		dTouch.pageX = window.innerWidth * x;
		dTouch.pageY = window.innerHeight * y;
		dTouch.target = document.elementFromPoint(dTouch.pageX, dTouch.pageY);
		dTouch.devicetouch = devicetouch;
		
		// Handle different kinds of event.
		switch (type)
		{
			case MultiTouch.START:
				this.cursors.push(dTouch);
				this._touchstart(dTouch);
				break;
			
			case MultiTouch.MOVE:
				this._touchmove(dTouch);
				break;
			
			case MultiTouch.END:
				this.cursors.splice(this.cursors.indexOf(dTouch), 1);
				this._touchend(dTouch);
				break;
		}
		
		if (type === MultiTouch.END)
			delete this._data[sid];
	}
};

/** @brief A singleton variable. */
var _KinectTouchInstance = null;

/**
 * @brief A class which adds multi-touch to a page.
 * @author John Hardy
 * @date 18th August 2012
 * Usage:
 *   var mt = new KinectTouch({
 *       point_limit : 200,         // The number of points we are allowed to process.
 *       surface_zoffset : 0.01,    // The offset from the surface (in meters) at which to start capturing data.
 *       height : 0.01,             // The distance from the surface offset (in meters) at which to stop capturing data.
 *       relativeto : Surface.Name, // The surface you want to take multi-touch input from.
 *   });
 *   mt.updateSurface() // <-- all this if you have new surface dimensions or render resolution
 */
var KinectTouch  = Class.extend({
	
	/**
	 * @brief Create a new Kinect touch tracker.
	 */
	init: function(dArgs)
	{
		// If we are already created, die.
		if (_KinectTouchInstance != null)
			throw "Error";
		_KinectTouchInstance = this;
		
		// Process arguments.
		var dArguments = {
			point_limit : 200,
			height:0.010,
			relativeto : Surface.Name,
			surface_zoffset : 0.001,
			callback : "KinectMultiTouch_HandlePointCloud"
		};
		
		// Override anything with our arguments.
		for (var key in dArgs)
			dArguments[key] = dArgs[key];
		
		// Variable.
		this.kLastLoop = new Date();
		this.fFps = 0;
		this.fFrameTime = 0;
		
		// Generate a new touch tracker to process the data.
		this.processor = new TouchTracker({
			debug   : dArgs.debug  || false,
			trails  : dArgs.trails || false,
			start   : function(touch){ MultiTouch.inject(MultiTouch.START, touch.id, touch.x(), touch.y(), touch); },
			stop    : function(touch){ MultiTouch.inject(MultiTouch.END,   touch.id, touch.x(), touch.y(), touch); },
			update  : function(touch){ MultiTouch.inject(MultiTouch.MOVE,  touch.id, touch.x(), touch.y(), touch); },
		});
		
		// Subscript to point cloud data.
		Authority.request("KinectLowestPointCube", dArguments);
	},
	
	/**
	 * @brief Enable or disable debug mode.
	 */
	enableDebug : function(bEnabled) { this.processor.debug(bEnabled); },
	
	/**
	 * @brief Update the touch tracker if we have new surface dimensions.
	 */
	updateSurface : function()
	{
		this.processor.updateSurface();
	},
	
	process : function(points) {
		
		// Work out the time since the last process.
		var kLoop = new Date;
		this.fFps = 1000 / (kLoop - this.kLastLoop);
		this.kLastLoop = kLoop;
		
		// Process the points.
		var start = new Date().getMilliseconds();
		this.processor.process(points);
		var stop = new Date().getMilliseconds();
		this.fFrameTime = stop - start;
	},
	
	/**
	 * @brief Get the number of touch frames processed per second.
	 */
	fps : function() {
		return this.fFps;
	},
	
	/**
	 * @brief Get how many ms it took to process the last frame of touch points.
	 */
	processTime : function() {
		return this.fFrameTime;
	},
	
});

/** @brief Handle the data passed in from the Kinect. */
function KinectMultiTouch_HandlePointCloud(lPoints) {
	
	// Transform data into the format for the dbscan.
	var lData = [];
	for (var i = 0; i < lPoints.length; ++i)
	{
		lData.push({
			x : Math.abs(lPoints[i][0]) * 100,
			y : Math.abs(lPoints[i][1]) * 100,
			z : Math.abs(lPoints[i][2]) * 100,
		});
	}
	
	// Process it.
	_KinectTouchInstance.process(lData);
}


