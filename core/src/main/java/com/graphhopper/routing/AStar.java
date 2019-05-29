/*
 *  Licensed to GraphHopper GmbH under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for
 *  additional information regarding copyright ownership.
 *
 *  GraphHopper GmbH licenses this file to you under the Apache License,
 *  Version 2.0 (the "License"); you may not use this file except in
 *  compliance with the License. You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package com.graphhopper.routing;

import com.graphhopper.coll.GHIntObjectHashMap;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.BeelineWeightApproximator;
import com.graphhopper.routing.weighting.WeightApproximator;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.SPTEntry;
import com.graphhopper.util.*;

import java.util.PriorityQueue;

/**
 * This class implements the A* algorithm according to
 * http://en.wikipedia.org/wiki/A*_search_algorithm
 * <p>
 * Different distance calculations can be used via setApproximation.
 * <p>
 *
 * @author Peter Karich
 */
public class AStar extends AbstractRoutingAlgorithm {
    private WeightApproximator weightApprox;  // 近似距离计算
    private int visitedCount;
    private GHIntObjectHashMap<AStarEntry> fromMap; //CLOSE set
    private PriorityQueue<AStarEntry> prioQueueOpenSet;  //OPEN set
    private AStarEntry currEdge; //当前节点
    private int to1 = -1;

    public AStar(Graph graph, Weighting weighting, TraversalMode tMode) {
        super(graph, weighting, tMode);
        int size = Math.min(Math.max(200, graph.getNodes() / 10), 2000);
        initCollections(size);
        BeelineWeightApproximator defaultApprox = new BeelineWeightApproximator(nodeAccess, weighting).setDistanceCalc(Helper.DIST_PLANE);
        defaultApprox.setDistanceCalc(Helper.DIST_PLANE);
        setApproximation(defaultApprox);
    }

    /**
     * @param approx defines how distance to goal Node is approximated
     */
    public AStar setApproximation(WeightApproximator approx) {
        weightApprox = approx;
        return this;
    }

    protected void initCollections(int size) {
        fromMap = new GHIntObjectHashMap<>();
        prioQueueOpenSet = new PriorityQueue<>(size);
    }

    @Override
    public Path calcPath(int from, int to) {
        checkAlreadyRun();
        to1 = to;

        weightApprox.setTo(to);
        double weightToGoal = weightApprox.approximate(from);
        currEdge = new AStarEntry(EdgeIterator.NO_EDGE, from, 0 + weightToGoal, 0);
        if (!traversalMode.isEdgeBased()) {
            fromMap.put(from, currEdge);
        }
        return runAlgo();
    }

    private Path runAlgo() {
        double currWeightToGoal, estimationFullWeight;
        EdgeExplorer explorer = outEdgeExplorer;
        while (true) {
            int currVertex = currEdge.adjNode;  //从起始节点开始遍历
            visitedCount++;
             if (isMaxVisitedNodesExceeded())
                return createEmptyPath();
            if (finished())
                break;

            EdgeIterator iter = explorer.setBaseNode(currVertex); //该节点所有邻边的集合
            while (iter.next()) {//遍历邻边
                if (!accept(iter, currEdge.edge))
                    continue;

                double alreadyVisitedWeight = weighting.calcWeight(iter, false, currEdge.edge)
                        + currEdge.weightOfVisitedPath;
                if (Double.isInfinite(alreadyVisitedWeight))
                    continue;

                int traversalId = traversalMode.createTraversalId(iter, false);
                AStarEntry ase = fromMap.get(traversalId);
                //如果该节点在不在CLOSE中,或者 在CLOSE中且遍历下一个节点的路径代价更短
                 if (ase == null || ase.weightOfVisitedPath > alreadyVisitedWeight) {
                    int neighborNode = iter.getAdjNode(); //邻点
                    currWeightToGoal = weightApprox.approximate(neighborNode);  // 计算优先级
                    estimationFullWeight = alreadyVisitedWeight + currWeightToGoal;  //计算起点到下一个点的预估代价
                    if (ase == null) { //如果该节点在不在CLOSE中
                        ase = new AStarEntry(iter.getEdge(), neighborNode, estimationFullWeight, alreadyVisitedWeight);
                        fromMap.put(traversalId, ase);
                    } else { //遍历下一个节点的路径代价更短
//                        assert (ase.weight > 0.9999999 * estimationFullWeight) : "Inconsistent distance estimate. It is expected weight >= estimationFullWeight but was "
//                                + ase.weight + " < " + estimationFullWeight + " (" + ase.weight / estimationFullWeight + "), and weightOfVisitedPath:"
//                                + ase.weightOfVisitedPath + " vs. alreadyVisitedWeight:" + alreadyVisitedWeight + " (" + ase.weightOfVisitedPath / alreadyVisitedWeight + ")";

                        prioQueueOpenSet.remove(ase);  //从OPEN中删除
                        ase.edge = iter.getEdge();
                        ase.weight = estimationFullWeight; //计算优先级
                        ase.weightOfVisitedPath = alreadyVisitedWeight;
                    }

                    ase.parent = currEdge; //设置m的父节点为n
                    prioQueueOpenSet.add(ase);  //将节点m加入openSet中(优先队列)

                    updateBestPath(iter, ase, traversalId);
                } //直接跳过
            }

            if (prioQueueOpenSet.isEmpty())
                return createEmptyPath();  //OPEN 为空说明地图搜索之后没有找到路径

            currEdge = prioQueueOpenSet.poll();
            if (currEdge == null)
                throw new AssertionError("Empty edge cannot happen");
        }

        return extractPath();
    }

    @Override
    protected Path extractPath() {
        return new Path(graph, weighting).
                setWeight(currEdge.weight).setSPTEntry(currEdge).extract();
    }

    @Override
    protected boolean finished() {
        return currEdge.adjNode == to1;
    }

    @Override
    public int getVisitedNodes() {
        return visitedCount;
    }

    protected void updateBestPath(EdgeIteratorState edgeState, SPTEntry bestSPTEntry, int traversalId) {
    }

    public static class AStarEntry extends SPTEntry {
        double weightOfVisitedPath;

        public AStarEntry(int edgeId, int adjNode, double weightForHeap, double weightOfVisitedPath) {
            super(edgeId, adjNode, weightForHeap);
            this.weightOfVisitedPath = weightOfVisitedPath;
        }

        @Override
        public final double getWeightOfVisitedPath() {
            return weightOfVisitedPath;
        }

        @Override
        public AStarEntry getParent() {
            return (AStarEntry) parent;
        }
    }

    @Override
    public String getName() {
        return Parameters.Algorithms.ASTAR + "|" + weightApprox;
    }
}
