/*
 * Implementation of Enhanced Conflict-based Search (ECBST)
 *
 * - ref
 * Barer, M., Sharon, G., Stern, R., & Felner, A. (2014).
 * Suboptimal Variants of the Conflict-Based Search Algorithm for the
 * Multi-Agent Pathfinding Problem. In Seventh Annual Symposium on Combinatorial
 * Search.
 */

#pragma once
#include <memory>
#include <tuple>

#include "lib_cbs.hpp"
#include "solver.hpp"
#include <sparsehash/sparse_hash_map>

class ECBST : public Solver
{
public:
    static const std::string SOLVER_NAME;

protected:
    // high-level node, see CBS for details
    struct HighLevelNode
    {
        Paths paths;
        int h_id;
        int depth = 0;
        int replan = -1;
        std::vector<LibCBS::Constraints> constraints;
        LibCBS::Conflicts conflicts;
        int makespan;
        int soc;
        int f;
        int LB;                  // lower bound
        std::vector<int> f_mins; // f_mins value in the low-level search
        bool valid;
        std::shared_ptr<HighLevelNode> parent = nullptr;
        int max_constraint_time = 0;

        HighLevelNode() {}
        HighLevelNode(Paths _paths, std::vector<LibCBS::Constraints> &_c, int _m, int _soc,
                      int _f, int _LB, std::vector<int> _f_mins, bool _valid)
            : paths(_paths),
              constraints(_c),
              makespan(_m),
              soc(_soc),
              f(_f),
              LB(_LB),
              f_mins(_f_mins),
              valid(_valid)
        {
        }
    };
    using HighLevelNode_p = std::shared_ptr<HighLevelNode>;
    using CompareHighLevelNode =
        std::function<bool(HighLevelNode_p, HighLevelNode_p)>;

    // used in the low-level search
    struct FocalNode
    {
        Node *v;      // location
        int g;        // in getTimedPath, g represents t
        int f1;       // used in open list
        int f2;       // used in focal list
        FocalNode *p; // parent
    };
    using CompareFocalNode = std::function<bool(FocalNode *, FocalNode *)>;
    using CheckFocalFin = std::function<bool(FocalNode *)>;
    using CheckInvalidFocalNode = std::function<bool(FocalNode *)>;
    using FocalHeuristics = std::function<int(FocalNode *)>;

    // sub optimality
    float sub_optimality;
    static const float DEFAULT_SUB_OPTIMALITY;


    
    void setInitialHighLevelNode(HighLevelNode_p n);
    void setInitialHighLevelNodeFocal(HighLevelNode_p n);
    Path getInitialPath(int id, const Paths &paths);
    Path getInitialPathFocal(HighLevelNode_p n, int id);

    // objective for open list
    CompareHighLevelNode getMainObjective();
    // objective for focal list
    CompareHighLevelNode getFocalObjective();

    void invoke(HighLevelNode_p h_node, int id);
    void invokeDP(HighLevelNode_p h_node, int id);

    // return path and f-min value
    std::tuple<Path, int> getFocalPath(HighLevelNode_p h_node, int id);
    std::tuple<Path, int> getTimedPathByFocalSearch(
        Node *const s, Node *const g, float w, // sub-optimality
        FocalHeuristics &f1Value, FocalHeuristics &f2Value,
        CompareFocalNode &compareOPEN, CompareFocalNode &compareFOCAL,
        CheckFocalFin &checkFocalFin,
        CheckInvalidFocalNode &checkInvalidFocalNode);

    // make path from focal node
    Path getPathFromFocalNode(FocalNode *_n);

    HighLevelNode_p old_node = nullptr;
    void generateChild(HighLevelNode_p &children, const HighLevelNode_p parent, const LibCBS::Constraint_p new_constraint);

    void countConflictsDP(HighLevelNode_p &child,int id);
    int numExpansions;
    // main
    void run();

public:
    ECBST(Problem *_P);
    ECBST(Graph *g, Config starts, Config goals);
    ~ECBST(){};

    void setParams(int argc, char *argv[]);
    static void printHelp();
    int getNumExpansions()
    {
        return numExpansions;
    }
};
