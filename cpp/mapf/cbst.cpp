#include "cbst.hpp"
#include <thread>
#include <omp.h>
// #include <mutex>
CBST::CBST(Problem *_P) : CBS(_P)
{
}

CBST::CBST(Graph *g, Config starts, Config goals) : CBS(g, starts, goals)
{
}

void CBST::expand(HighLevelNode_p n, std::vector<HighLevelNode_p> &children, int index)
{

    LibCBS::Constraints constraints = LibCBS::getFirstConstraints(n->paths);
    if (constraints.empty())
    {
        // solved = true;
        return;
    }

    // create new nodes
    size_t count = 2*index;
    for (auto c : constraints)
    {
        LibCBS::Constraints new_constraints = n->constraints;
        new_constraints.push_back(c);
        // std::cout<<"num agents="<<n->paths.size()<<"  constraint size=  "<<new_constraints.size()<<std::endl;

        HighLevelNode_p m = std::make_shared<HighLevelNode>(
            666,             // id
            n->paths,        // (old) paths, updated by invoke
            new_constraints, // new constraints
            n->makespan,     // (old) makespan
            n->soc,          // (old) sum of costs
            n->f,            // (old) #conflicts
            true);
        invoke(m, c->id);
        if (!m->valid)
            continue;
        // HighLevelTree.push(m);
        children[count] = m;
        count++;
        // ++h_node_num;
    }
}

void CBST::run()
{
    const int num_threads = 16;
    // const int num_threads = std::thread::hardware_concurrency();
    // set objective function

    // OPEN
    OPENLIST HighLevelTree(&compare);

    HighLevelNode_p n = std::make_shared<HighLevelNode>();
    setInitialHighLevelNode(n);
    if (!n->valid)
        return; // failed to plan initial paths
    HighLevelTree.push(n);

    // start high-level search
    int h_node_num = 1;
    int iteration = 0;
    while (!HighLevelTree.empty())
    {
        ++iteration;
        // check limitation
        if (overCompTime())
        {
            info(" ", "timeout");
            break;
        }

        std::vector<HighLevelNode_p> mp_nodes;
        int soc_lb=-1;
        std::vector<HighLevelNode_p> zero_conflict_nodes;
        while (mp_nodes.size() < num_threads and HighLevelTree.size() != 0)
        {
            n = HighLevelTree.top();
            if (soc_lb == -1)
                soc_lb = n->soc;
            HighLevelTree.pop();
            h_node_num++;
            //when the number of conflicts is zero
            if (n->f == 0)
            {
                //only if the soc is equal to the lowerbound, terminate the loop
                if (n->soc == soc_lb)
                {
                    solved = true;
                    break;
                }
                //otherwise, the node is not guaranteed to be optimal
                else
                {
                    zero_conflict_nodes.push_back(n);
                    continue;
                }
            }
            mp_nodes.push_back(n);
        }
        if (solved)
            break;

        std::vector<HighLevelNode_p> children(2 * mp_nodes.size(), nullptr);

        // #pragma omp parallel num_threads(num_threads)
        //         {
        // #pragma omp parallel for
        std::vector<std::thread> threads;

        for (size_t i = 0; i < mp_nodes.size(); i++) {
            threads.emplace_back(&CBST::expand, this, mp_nodes[i], std::ref(children), i);
        }

        for (std::thread& thread : threads) {
            thread.join();
        }

        /*****************single threading*************/
        // for (size_t i = 0; i < mp_nodes.size(); i++)
        // {
        //     expand(mp_nodes[i], children, i);
        // }

        /************************single threading********/

        // }
        for (size_t i = 0; i < children.size(); i++)
        {
            if (children[i] != nullptr)
                HighLevelTree.push(children[i]);
        }
        for (auto &zn : zero_conflict_nodes)
            HighLevelTree.push(zn);
        // pickup one node with minimal f-value

        // info(" ", "elapsed:", getSolverElapsedTime(),
        //      ", explored_node_num:", iteration, ", nodes_num:", h_node_num,
        //      ", conflicts:", n->f, ", constraints:", n->constraints.size(),
        //      ", soc:", n->soc);

        // check conflict
    }

    if (solved)
        solution = pathsToPlan(n->paths);
}
