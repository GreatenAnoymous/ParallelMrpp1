#pragma once
#include <memory>
#include <tuple>
#include "lib_cbs.hpp"
#include "solver.hpp"
#include <map>

class ECBSW : public Solver
{
public:
    static const std::string SOLVER_NAME;

protected:
    // high-level node, see CBS for details
    struct HighLevelNode
    {
        Paths paths;
        int h_id;
        std::vector<LibCBS::Constraints> constraints;
        LibCBS::Conflicts conflicts;
        int max_constraint_time = 0;
        std::vector<int> replans;
        int makespan;
        int soc;
        int replan;
        int f;
        int LB;                  // lower bound
        std::vector<int> f_mins; // f_mins value in the low-level search
        bool valid;
        std::shared_ptr<HighLevelNode> parent = nullptr;
        HighLevelNode() {}
        std::vector<int> replan_agents;
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
    HighLevelNode_p old_node = nullptr;

    void setInitialHighLevelNode(HighLevelNode_p n);
    void setInitialHighLevelNodeFocal(HighLevelNode_p n);
    void setInitialHighLevelNodeFocalParallel(HighLevelNode_p n);
    Path getInitialPath(int id, const Paths &paths);
    Path getInitialPathFocal(HighLevelNode_p n, int id);
    Path getInitialPathFocal(HighLevelNode_p h_node, int id, std::vector<std::vector<int>> &PATH_TABLE2);

    // objective for open list
    CompareHighLevelNode getMainObjective();
    // objective for focal list
    CompareHighLevelNode getFocalObjective();

    void chooseDisjointConflicts(HighLevelNode_p n, std::vector<LibCBS::Conflict_p> &conflicts, int k = 4);

    void threadCountConflicts(std::vector<HighLevelNode_p> &, std::vector<int> &replanned_agents, int thread_id);

    std::tuple<Path, int> getFocalPath3(Node*s, Node *g, LibCBS::Constraints &cs, int makespan, int id);

    std::tuple<Path, int> getFocalPath2(HighLevelNode_p h_node, int id);

    HighLevelNode_p  AddressConflictsParallel(HighLevelNode_p n,std::vector<LibCBS::Conflict_p> &conflicts, int LB_min);

    HighLevelNode_p  AddressConflicts(HighLevelNode_p n,std::vector<LibCBS::Conflict_p> &conflicts, int LB_min);

    void generateChild(HighLevelNode_p &children, const HighLevelNode_p parent, const LibCBS::Constraint_p new_constraint);

    void invokeDP(HighLevelNode_p h_node, int id, std::vector<int> &replanned_agents);
    void invokeDP(HighLevelNode_p h_node, int id);

    void updatePathTableExcept(const Paths &paths, std::vector<int> &replanned_agents);
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
    std::vector<std::vector<std::vector<int>>> PATH_TABLES;

    int numExpansions;
    // main
    void run();

public:
    ECBSW(Problem *_P);
    ECBSW(Graph *g, Config starts, Config goals);
    ~ECBSW(){};

    void setParams(int argc, char *argv[]);
    static void printHelp();
    int getNumExpansions()
    {
        return numExpansions;
    }
};