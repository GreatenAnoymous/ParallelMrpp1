/*
 * Implementation of Enhanced Conflict-based Search (ECBSDE)
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
#include <atomic>
#include "buffer.hpp"
#include <experimental/memory>

class ECBSDE : public Solver
{
public:
    static const std::string SOLVER_NAME;

protected:
    // high-level node, see CBS for details
    struct HighLevelNode
    {
        Paths paths;
        int h_id;
        LibCBS::Constraints constraints;
        LibCBS::Conflicts conflicts;
        int makespan;
        int soc;
        int f;
        int LB;                  // lower bound
        std::vector<int> f_mins; // f_mins value in the low-level search
        bool valid;

        HighLevelNode() {}
        HighLevelNode(Paths _paths, LibCBS::Constraints _c, int _m, int _soc,
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

    using FocalList = std::priority_queue<HighLevelNode_p, std::vector<HighLevelNode_p>,
                                          CompareHighLevelNode>;
    CompareHighLevelNode compareOPEN = getMainObjective();
    CompareHighLevelNode compareFOCAL = getFocalObjective();

    // sub optimality
    float sub_optimality;
    static const float DEFAULT_SUB_OPTIMALITY;

    void setInitialHighLevelNodeFocal(HighLevelNode_p n);

    Path getInitialPathFocal(HighLevelNode_p n, int id);

    void clearPathTableForThread(const Paths &paths, const int thread_index);

    // objective for open list
    CompareHighLevelNode getMainObjective();
    // objective for focal list
    CompareHighLevelNode getFocalObjective();

    void invoke(HighLevelNode_p h_node, int id, int thread_id, int ci=0);

    // return path and f-min value
    std::tuple<Path, int> getFocalPath(HighLevelNode_p h_node, int id, int thread_id);
    std::tuple<Path, int> getTimedPathByFocalSearch(
        Node *const s, Node *const g, float w, // sub-optimality
        FocalHeuristics &f1Value, FocalHeuristics &f2Value,
        CompareFocalNode &compareOPEN, CompareFocalNode &compareFOCAL,
        CheckFocalFin &checkFocalFin,
        CheckInvalidFocalNode &checkInvalidFocalNode);

    // make path from focal node
    Path getPathFromFocalNode(FocalNode *_n);

    void initialize_thread_path_table();

    std::vector<std::shared_ptr<buffer<HighLevelNode_p>>> income_buffer;

    std::vector<std::unique_ptr<std::atomic<int>>> LBs;

    

    void updatePathTableForThread(const Paths &paths, const int id, const int thread_index);

    std::atomic<int> numExpansions{0};

    std::vector<std::vector<std::vector<int>>> PATH_TABLES;

    int getCurrentLowBound();

    // main
    void run();

    void thread_search(size_t thread_id);
    size_t getRandomHashCode(int thread_id);

    HighLevelNode_p incumbent = nullptr;

public:
    ECBSDE(Problem *_P);
    ECBSDE(Graph *g, Config starts, Config goals);
    ~ECBSDE(){};

    std::atomic<int> socLB{0};

    static void printHelp();

    int getNumExpansions()
    {
        return numExpansions.load();
    }

    int getSocLB()
    {
        return socLB.load();
    }
};
