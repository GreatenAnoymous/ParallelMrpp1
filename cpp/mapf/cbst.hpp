#pragma once
#include "cbs.hpp"

/**
 * @brief  In this variant we explore multiple high-level nodes simultaneously using 
 * multi-threading
 * 
 */
class CBST:public CBS{

public:
    CBST(Problem * _P);
    CBST(Graph *g, Config starts, Config goals);

private:
    void run();
    static bool compare(HighLevelNode_p a, HighLevelNode_p b)
    {
        if (a->soc != b->soc)
            return a->soc > b->soc;
        if (a->f != b->f)
            return a->f > b->f; // tie-breaker
        return false;
    };

    using OPENLIST=std::priority_queue<HighLevelNode_p, HighLevelNodes, decltype(&compare)>;
    void expand(HighLevelNode_p parentNode, std::vector<HighLevelNode_p> &children, int index);

};