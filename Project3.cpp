

/*
 * Project2.cpp
 * CSCI561 Project 2
 */

/*
 * For documentation regarding camera control, please refer to Camera.h in the System/Utility folder.
 */

#include <iostream>
#include <PxPhysicsAPI.h>
#include <vector>
#include "WheelbotModule.h"
#include "WheelbotSim.h"
#include "Environment.h"
#include "WheelbotConstants.h"
#include "Project3-Control.h"
#include "Experiment.h"
#include <cv.h>
rm3d::ai::WheelbotSim *rsim;
WheelbotSimBase * simulator;


bool onSegment(PxVec2 p, PxVec2 q, PxVec2 r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;
    
    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(PxVec2 p, PxVec2 q, PxVec2 r)
{
    // See 10th slides from following link for derivation of the formula
    // http://www.dcs.gla.ac.uk/~pat/52233/slides/Geometry1x1.pdf
    int val = (q.y - p.y) * (r.x - q.x) -
    (q.x - p.x) * (r.y - q.y);
    
    if (val == 0) return 0;  // colinear
    
    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool doIntersect(PxVec2 p1, PxVec2 q1, PxVec2 p2, PxVec2 q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    
    // General case
    if (o1 != o2 && o3 != o4)
        return true;
    
    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    
    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
    
    return false; // Doesn't fall in any of the above cases
}


PxReal getAngle(PxVec2 A, PxVec2 B) {
    
    A.normalize();
    B.normalize();
    
    // angle with +ve x-axis, in the range (−π, π]
    float thetaA = atan2(A.x, A.y);
    float thetaB = atan2(B.x, B.y);
    
    float thetaAB = thetaB - thetaA;
    
    // get in range (−π, π]
    while (thetaAB <= - PxPi)
        thetaAB += 2 * PxPi;
    
    while (thetaAB > PxPi)
        thetaAB -= 2 * PxPi;
    return thetaAB;
}

Graph createRandomGraph(int numNodes, float lowerX, float lowerZ, float upperX, float upperZ, float minDistance, int edgesToAdd) {
    Graph graph;
    //Choose nodes
    bool graphGood = false;
    while (!graphGood) {
        graph = Graph();
        for (int i=0; i<numNodes; i++) {
            bool mustChoose = true;
            while (mustChoose) {
                int sign1 = (rand() % 2 == 0) ? -1 : 1;
                int sign2 = (rand() % 2 == 0) ? -1 : 1;
                PxVec3 randPoint = PxVec3(sign1*rm3d::SimulationUtility::randomFloatInRange(lowerX, upperX),0,
                                          sign2*rm3d::SimulationUtility::randomFloatInRange(lowerZ, upperZ));
                if (i == 0) {
                    graph.graphPoints.push_back(randPoint);
                    break;
                }
                bool over = false;
                for (int j=0; j<i; j++) {
                    if ((randPoint - graph.graphPoints[j]).magnitude() < minDistance) {
                        over = true;
                    }
                }
                if (!over){
                    mustChoose = false;
                    graph.graphPoints.push_back(randPoint);
                }
                
            }
        }
        
        vector<float> keys(numNodes);
        vector<int> parent(numNodes);
        vector<bool> mstSet(numNodes);
        for (int i=0; i<numNodes; i++) {
            mstSet[i] = false;
        }
        vector<int> mst;
        for (int i=0; i<numNodes; i++) {
            keys[i] = 100000.0;
        }
        
        keys[0] = 0;
        parent[0] = -1;
        for (int i=0; i<numNodes - 1; i++) {
            float min = 1000000.0;
            int minNode = -1;
            
            for (int j=0; j<numNodes; j++) {
                if (mstSet[j] == false && keys[j] < min) {
                    min = keys[j];
                    minNode = j;
                }
            }
            mstSet[minNode] = true;
            for (int j=0; j<numNodes; j++) {
                if ((graph.graphPoints[j] - graph.graphPoints[minNode]).magnitude() < keys[j] && mstSet[j] == false) {
                    parent[j] = minNode;
                    keys[j] = (graph.graphPoints[j] - graph.graphPoints[minNode]).magnitude();
                }
                
            }
        }
        
        for (int i = 1; i<numNodes; i++) {
            graph.graphEdges.push_back(pair<int, int>(parent[i], i));
            graph.graphEdges.push_back(pair<int, int>(i, parent[i]));
        }
        
        int addedEdges = 0;
        while (addedEdges < edgesToAdd) {
            int node1 = rand()%numNodes;
            int node2 = rand()%numNodes;
            for (int i=0; i<graph.graphEdges.size(); i++) {
                if ((graph.graphEdges[i].first == node1 && graph.graphEdges[i].second == node2) ||
                    (graph.graphEdges[i].first == node2 && graph.graphEdges[i].second == node1) ||
                    node1 == node2) {
                    continue;
                } else {
                    bool intersect = false;
                    for (int j = 0; j<graph.graphEdges.size(); j++) {
                        if (graph.graphEdges[j].first == node1 ||
                            graph.graphEdges[j].second == node1 ||
                            graph.graphEdges[j].first == node2 ||
                            graph.graphEdges[j].second == node2) {
                            continue;
                        } else {
                            if (doIntersect(PxVec2(graph.graphPoints[graph.graphEdges[j].first].x, graph.graphPoints[graph.graphEdges[j].first].z),
                                            PxVec2(graph.graphPoints[graph.graphEdges[j].second].x, graph.graphPoints[graph.graphEdges[j].second].z),
                                            PxVec2(graph.graphPoints[node1].x, graph.graphPoints[node1].z),
                                            PxVec2(graph.graphPoints[node2].x, graph.graphPoints[node2].z))) {
                                intersect = true;
                            }
                        }
                    }
                    if (!intersect) {
                        bool unique = true;
                        for (int j = 0; j<graph.graphEdges.size(); j++) {
                            if ((graph.graphEdges[j].first == node1 && graph.graphEdges[j].second == node2) ||
                                (graph.graphEdges[j].second == node1 && graph.graphEdges[j].first == node2)) {
                                unique = false;
                            }
                        }
                        
                        if (unique) {
                            graph.graphEdges.push_back(pair<int, int>(node1, node2));
                            graph.graphEdges.push_back(pair<int, int>(node2, node1));
                            addedEdges++;
                        }
                    }
                }
            }
        }
        graphGood = true;
        for (int i=0; i<graph.graphPoints.size(); i++) {
            vector<int> edgesFromNode;
            for (int j=0; j<graph.graphEdges.size(); j++) {
                if(graph.graphEdges[j].first == i) {
                    edgesFromNode.push_back(graph.graphEdges[j].second);
                }
            }
            for (int j=0; j<edgesFromNode.size(); j++) {
                for (int k=0; k<edgesFromNode.size(); k++) {
                    if (j != k) {
                        PxVec2 vec1(graph.graphPoints[i].x - graph.graphPoints[edgesFromNode[j]].x,
                                    graph.graphPoints[i].z - graph.graphPoints[edgesFromNode[j]].z);
                        PxVec2 vec2(graph.graphPoints[i].x - graph.graphPoints[edgesFromNode[k]].x,
                                    graph.graphPoints[i].z - graph.graphPoints[edgesFromNode[k]].z);
                        if (abs(getAngle(vec1, vec2)) < 0.7 ) {
                            graphGood = false;
                            continue;
                        }
                    }
                }
            }
        }
        if (graphGood) break;
    }
    return graph;
}


class Project3:public rm3d::ai::Experiment {
public:
    static void init() {
        //Adjust graph and project level parameters here.
        int numNodes = 20;
        int squareWidth = 30;
        float minDistance = 5.0;
        int numPits =0;
        int extraEdges = 6;
        int numWumpuses = 0;
        bool shootingNoise = false;
        Graph graph = createRandomGraph(numNodes, 0, 0, squareWidth, squareWidth, minDistance, extraEdges);
        vector<float> graphWinds(graph.graphEdges.size());
        vector<float> graphSmell(graph.graphEdges.size());
        for (int i=0; i<graphWinds.size(); i++) {
            graphWinds[i] = 0.0;
        }
        for (int i=0; i<graphSmell.size(); i++) {
            graphSmell[i] = 0.0;
        }
        int initialNode = rand() % numNodes;
        int goalNode = initialNode;
        while (goalNode == initialNode) {
            goalNode = rand() % numNodes;
        }
        int addedPits = 0;
        int addedWumpuses = 0;
        vector<int> pits;
        vector<int> wumpuses;
        
        while (addedPits < numPits) {
            int pitIndex = rand() % numNodes;
            bool pitUnique = true;
            for (int i=0; i<pits.size(); i++) {
                if (pitIndex == pits[i]) {
                    pitUnique = false;
                }
            }
            if (pitUnique && pitIndex != initialNode && pitIndex != goalNode) {
                addedPits++;
                pits.push_back(pitIndex);
            }
        }
        
        while (addedWumpuses < numWumpuses) {
            int wumpusIndex = rand() % numNodes;
            bool wumpusUnique = true;
            for (int i=0; i<wumpuses.size(); i++) {
                if (wumpusIndex == wumpuses[i]) {
                    wumpusUnique = false;
                }
            }
            for (int i=0; i<pits.size(); i++) {
                if (wumpusIndex == pits[i]) {
                    wumpusUnique = false;
                }
            }
            if (wumpusUnique && wumpusIndex != initialNode && wumpusIndex != goalNode) {
                addedWumpuses++;
                wumpuses.push_back(wumpusIndex);
            }
        }
        for (int j =0; j<pits.size(); j++) {
            for (int i=0; i<graph.graphEdges.size(); i++) {

                if (graph.graphEdges[i].first == pits[j]) {
                    graphWinds[i] = (graph.graphPoints[graph.graphEdges[i].first] - graph.graphPoints[graph.graphEdges[i].second]).magnitude();
                } else if (graph.graphEdges[i].second == pits[j]) {
                    graphWinds[i] = (graph.graphPoints[graph.graphEdges[i].second] - graph.graphPoints[graph.graphEdges[i].first]).magnitude();
                }
            }
        }
        for (int j =0; j<wumpuses.size(); j++) {
            for (int i=0; i<graph.graphEdges.size(); i++) {
                
                if (graph.graphEdges[i].first == wumpuses[j]) {
                    graphSmell[i] = (graph.graphPoints[graph.graphEdges[i].first] - graph.graphPoints[graph.graphEdges[i].second]).magnitude();
                } else if (graph.graphEdges[i].second == wumpuses[j]) {
                    graphSmell[i] = (graph.graphPoints[graph.graphEdges[i].second] - graph.graphPoints[graph.graphEdges[i].first]).magnitude();
                }
            }
        }
        
        PxMaterial *mMaterial = simulator->getSimulationPhysics()->createMaterial(0.5, 0.5, 0.5);
        vector<Wumpus> wumpusDetails;
        for (int i=0; i<wumpuses.size(); i++) {
            PxTransform obInitial = PxTransform(graph.graphPoints[wumpuses[i]])*PxTransform(PxVec3(0,1,0));
            string name = "wumpus";
            stringstream ss;
            ss<<i;
            name += ss.str();
            char * nameChars = new char[name.length()];
            for (int i=0; i<name.length(); i++) {
                nameChars[i] = name[i];
            }
            simulator->createObstacle(obInitial, PxSphereGeometry(0.5), mMaterial, 10, 10, (const char *)nameChars, false, false, rm3d::Color(.5,.5,.5));
            Wumpus w;
            w.name = nameChars;
            w.index = wumpuses[i];
            wumpusDetails.push_back(w);
        }
        
        for (int i=0; i<simulator->getObstacles().size(); i++) {
            simulator->getObstacles()[i].obstacleBody->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
        }
        //Set position and orientation (around the y-axis) of Wheelbot in 3D world
        PxTransform in = PxTransform(PxVec3(graph.graphPoints[initialNode].x,rm3d::ai::WheelbotConstants::wheelbotWheelRadius,
                                            graph.graphPoints[initialNode].z));
        //Create wheelbot and load program in it
        rsim->createWheelbotModule("Wheelbot", in, (WheelbotModuleProgram *) new Project3_Control(simulator, graph, initialNode, goalNode, pits, graphWinds, wumpusDetails, graphSmell, shootingNoise, minDistance));
        PxMaterial *obMaterial = simulator->getSimulationPhysics()->createMaterial(.5, .5, .5);
        //simulator->followModuleWithCamera();
    }
    //Not used for this example
    static void shutdown() {
    }
    static void render() {
        //rm3d::Renderer::DrawFrame(rsim->getModuleWithName("Wheelbot")->getBody()->getGlobalPose());
    }
    //Environment properties
    static rm3d::ai::Environment env;
};
rm3d::ai::Environment Project3::env;

int main(int argc, char **argv) {
    Project3::env.zNearPlane = 0.1;
    Project3::env.zFarPlane = 1000.0;
    Project3::env.shouldSaveFrames = false;
    Project3::env.simulationTimeStep = 1/30.0f;
    Project3::env.disableLighting = false;
    Project3::env.showFPS = true;
    Project3::env.localTransformToCamera = PxTransform(PxVec3(-10, wheelbotWheelRadius + 10,0), PxQuat(-PxHalfPi/3.0, PxVec3(0,0,1)));
    //Execution Begins Here - Entry point to RM3D Simulation
    rsim = new rm3d::ai::WheelbotSim();
    //Disable docking - not needed for Wheelbot. This saves overhead
    rsim->setDockingEnabled(false);
    //Give simulator function pointers for custom code. First, cast rsim to base class
    simulator = (WheelbotSimBase *) rsim;
    //Set function pointers.
    simulator->setInitFunction(Project3::init);
    simulator->setShutdownFunction(Project3::shutdown);
    simulator->setCustomRenderFunction(Project3::render);
    //Program begins not paused
    simulator->setProgramPaused(false);
    //Start the simulation with a specified window size (in pixels) and environment options
    simulator->initializeSimulation(argc, argv, 1024, 768, &Project3::env,0, 50.0, 0, 0, 90);
	return 0;
}