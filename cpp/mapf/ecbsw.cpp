#include "ecbsw.hpp"
#include <chrono>
#include <omp.h>
#include <mutex>
#include <thread>

const std::string ECBSW::SOLVER_NAME = "ECBSW";
const float ECBSW::DEFAULT_SUB_OPTIMALITY = 2.0;
const int numThreads = 8;

ECBSW::ECBSW(Problem *_P) : Solver(_P)
{
    sub_optimality = DEFAULT_SUB_OPTIMALITY;
    solver_name = ECBSW::SOLVER_NAME + "-" + std::to_string(sub_optimality);
}

ECBSW::ECBSW(Graph *graph, Config starts, Config goals) : Solver(graph, starts, goals)
{
    sub_optimality = DEFAULT_SUB_OPTIMALITY;
    solver_name = ECBSW::SOLVER_NAME + "-" + std::to_string(sub_optimality);
}

void ECBSW::setInitialHighLevelNodeFocalParallel(HighLevelNode_p n)
{
    n->paths = Paths(P->getNum());
    n->constraints.resize(P->getNum(), {});
    std::vector<int> f_mins = std::vector<int>(P->getNum(), -1);
    std::mutex pathTableMutex;
    auto worker = [&](int thread_id)
    {
        int start = thread_id * (P->getNum() / numThreads);
        int end = start + (P->getNum() / numThreads);
        if (thread_id == numThreads - 1)
        {
            end = P->getNum();
        }
        for (int i = start; i < end; ++i)
        {

            Path path;
            if (P->getGoal(i)->id == P->getStart(i)->id)
                path = {P->getStart(i)};
            else
            {

                std::vector<std::vector<int>> path_table = PATH_TABLE;
                {
                    std::lock_guard<std::mutex> lock(pathTableMutex); // Lock before creating a local copy
                    path_table = PATH_TABLE;
                }
                path = getInitialPathFocal(n, i, path_table);

            }
            std::cout << "path size for agent " << i << "  =  " << path.size() << "  " << P->getStart(i)->id << "  " << P->getGoal(i)->id << std::endl;
            n->paths.paths[i] = path;
            {
                std::lock_guard<std::mutex> lock(pathTableMutex); // Lock before creating a local copy
                updatePathTableForAgent(path, i);
            }

            f_mins[i] = path.size() - 1;
        }
    };

    std::vector<std::thread> threads;
    for (int i = 0; i < numThreads; i++)
    {
        threads.emplace_back(worker, i);
    }

    for (auto &thread : threads)
        thread.join();
    n->paths.format();

    n->makespan = n->paths.getMakespan();
    n->soc = n->paths.getSOC();
    LibCBS::getAllConflictsDP(n->paths, n->conflicts);
    // LibCBS::getAllConflictsParallel(n->paths, n->conflicts);
    n->f = n->conflicts.size();
    // n->f = n->paths.countConflict();
    n->valid = true;
    n->f_mins = f_mins;
    n->h_id = 0;
    n->LB = n->soc; // initial lower bound
    std::cout << "makespan =" << n->makespan << "  soc=" << n->soc << "  num of conflicts=" << n->f << std::endl;
}

std::tuple<Path, int> ECBSW::getFocalPath(HighLevelNode_p h_node, int id)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);
    // if (id == 831)
    // {
    //     std::cout << "starts and goals" << std::endl;
    //     s->println();
    //     g->println();
    //     std::cout << "starts and goals" << std::endl;
    // }

    // pre processing
    // LibCBS::Constraints constraints;
    int max_constraint_time = 0;
    for (auto c : h_node->constraints[id])
    {
        max_constraint_time = std::max(c->t, max_constraint_time);
    }

    // f-value for online list
    FocalHeuristics f1Value;
    if (pathDist(id) > max_constraint_time)
    {
        f1Value = [&](FocalNode *n)
        { return n->g + pathDist(id, n->v); };
    }
    else
    {
        f1Value = [&](FocalNode *n)
        {
            return std::max(max_constraint_time + 1, n->g + pathDist(id, n->v));
        };
    }
    // f1Value = [&](FocalNode* n) { return n->g + pathDist(id, n->v); };
    const auto paths = h_node->paths;
    const int makespan = paths.getMakespan();

    // update PATH_TABLE
    // updatePathTable(paths, id);

    // std::cout << "Updated" << std::endl;
    // auto end = std::chrono::system_clock::now();
    // std::chrono::duration<double> elapsed_seconds = end - start;
    // double dt = elapsed_seconds.count();

    // std::cout << "PathTable updated in " << dt << std::endl;
    FocalHeuristics f2Value = [&](FocalNode *n)
    {
        if (n->g == 0)
            return 0;
        // last node
        if (n->g > makespan)
        {
            if (PATH_TABLE[makespan][n->v->id] != Solver::NIL)
                return n->p->f2 + 1;
        }
        else
        {
            // vertex conflict
            if (PATH_TABLE[n->g][n->v->id] != Solver::NIL)
            {
                return n->p->f2 + 1;

                // swap conflict
            }
            else if (PATH_TABLE[n->g][n->p->v->id] != Solver::NIL &&
                     PATH_TABLE[n->g - 1][n->v->id] ==
                         PATH_TABLE[n->g][n->p->v->id])
            {
                return n->p->f2 + 1;
            }
        }
        return n->p->f2;
    };

    CompareFocalNode compareOPEN = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f1 != b->f1)
            return a->f1 > b->f1;

        return false;
    };

    CompareFocalNode compareFOCAL = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f2 != b->f2)
            return a->f2 > b->f2;
        if (a->f1 != b->f1)
            return a->f1 > b->f1;
        if (a->g != b->g)
            return a->g < b->g;
        return false;
    };

    CheckFocalFin checkFocalFin = [&](FocalNode *n)
    {
        return n->v == g && n->g > max_constraint_time;
    };

    CheckInvalidFocalNode checkInvalidFocalNode = [&](FocalNode *m)
    {
        for (auto c : h_node->constraints[id])
        {
            if (m->g == c->t && m->v == c->v)
            {
                // vertex or swap conflict
                if (c->u == nullptr || c->u == m->p->v)
                    return true;
            }
        }
        return false;
    };

    auto p = getTimedPathByFocalSearch(s, g, sub_optimality, f1Value, f2Value,
                                       compareOPEN, compareFOCAL, checkFocalFin,
                                       checkInvalidFocalNode);

    // std::cout << "*************************" << std::endl;
    // clear used path table
    // clearPathTable(paths);

    return p;
}

void ECBSW::run()
{
    // high-level search
    CompareHighLevelNode compareOPEN = getMainObjective();
    CompareHighLevelNode compareFOCAL = getFocalObjective();

    // OPEN, FOCAL
    std::priority_queue<HighLevelNode_p, std::vector<HighLevelNode_p>,
                        CompareHighLevelNode>
        OPEN(compareOPEN);
    using FocalList =
        std::priority_queue<HighLevelNode_p, std::vector<HighLevelNode_p>,
                            CompareHighLevelNode>;
    FocalList FOCAL(compareFOCAL);
    // std::vector<HighLevelNode_p> OPEN;

    // initial node

    HighLevelNode_p n = std::make_shared<HighLevelNode>();
    // setInitialHighLevelNode(n);
    setInitialHighLevelNodeFocal(n);
    // setInitialHighLevelNodeFocalParallel(n);
    old_node = n;

    OPEN.push(n);
    FOCAL.push(n);

    int LB_min = n->LB;

    // main loop
    int h_node_num = 1;
    numExpansions = 0;
    auto startCounter = std::chrono::high_resolution_clock::now();
    auto endCounter = std::chrono::high_resolution_clock::now();
    while (!FOCAL.empty())
    {
        ++numExpansions;
        endCounter = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedCounter = endCounter - startCounter;
        printf("one iteration cost=%f\n", elapsedCounter.count());
        startCounter = std::chrono::high_resolution_clock::now();
        // check limitation
        if (overCompTime())
        {
            std::cout << "time out" << std::endl;
            break;
        }

        // /*
        //  *  update focal list
        //  */
        // // pop out invalid nodes
        while (!OPEN.empty() && !OPEN.top()->valid)
            OPEN.pop();
        // failure
        if (OPEN.empty())
            break;
        // when lower bound is updated
        if (LB_min != OPEN.top()->LB)
        {
            // new lower bound
            LB_min = OPEN.top()->LB;
            // admissible f-value in open list
            float LB_bound = LB_min * sub_optimality;
            // for escape
            std::vector<HighLevelNode_p> tmp;
            // clear focal list
            FocalList EMPTY(compareFOCAL);
            FOCAL = EMPTY;
            // insert nodes to focal list
            while (!OPEN.empty())
            {
                HighLevelNode_p top = OPEN.top();
                OPEN.pop();
                // already searched
                if (!top->valid)
                    continue;
                // escape
                tmp.push_back(top);
                // higher than LB_bound
                if ((float)top->LB > LB_bound)
                    break;
                // lower than LB_bound
                FOCAL.push(top);
            }
            // back
            for (auto ele : tmp)
                OPEN.push(ele);
        }

        // pickup one node

        n = FOCAL.top();
        FOCAL.pop();

        n->valid = false; // closed

        info(" ", "elapsed:", getSolverElapsedTime(),
             ", explored_node_num:", numExpansions, ", nodes_num:", h_node_num,
             ", conflicts:", n->f, ", constraints:", n->constraints.size(),
             ", soc:", n->soc);
        std::cout << "Number of Node expansions=" << numExpansions << " number of conflicts=" << n->f << std::endl;
        // check conflict
        if (n->conflicts.empty())
        {
            // auto c = LibCBS::getFirstConstraints(n->paths);
            // for(int i=0;i<n->paths.size();i++){
            //     std::cout<<n->paths.get(i).size()<<"   "<<n->paths.get(i)[0]->id<<"   "<<n->paths.get(i).back()->id<<std::endl;
            // }
            // std::cout << n->paths.size() << std::endl;
            // std::cout << "makespan= " << n->paths.getMakespan() << std::endl;
            // std::cout << "final constraint size= " << n->paths.countConflict() << std::endl;
            std::cout << "CBS tree depth=" << n->constraints.size() << " num expansions=" << numExpansions << "  conflcits size=" << n->conflicts.size() << std::endl;
            solved = true;
            break;
        }

        auto start = std::chrono::high_resolution_clock::now();
        int last_conflicts=n->conflicts.size()/2;
        while (n->conflicts.size() >= 32 and n->conflicts.size()>last_conflicts)
        {
            last_conflicts=n->conflicts.size();
            LibCBS::Conflicts cfs;
            LibCBS::Constraints disjoints;
            chooseDisjointConflicts(n, cfs, 3);
            // n = AddressConflicts(n, cfs, LB_min);
            n = AddressConflictsParallel(n, cfs, LB_min);
            clearPathTable(old_node->paths);
            old_node = n;
            old_node->parent = nullptr;
            std::vector<int> agents;
            // updatePathTable(n->paths);
            updatePathTableExcept(n->paths, agents);
        }

        LibCBS::Constraints constraints = LibCBS::constraintsFromConflict(LibCBS::chooseConflict(n->conflicts));
        if (n->parent != old_node)
        {
            clearPathTable(old_node->paths);
            updatePathTable(n->paths, constraints[0]->id);
        }
        else
        {
            if (old_node != nullptr)
            {

                clearPathTableForAgent(old_node->paths.get(n->replan), n->replan);
                updatePathTableForAgent(n->paths.get(n->replan), n->replan);
            }
        }

        for (auto c : constraints)
        {
            // std::vector<LibCBS::Constraints> new_constraints = n->constraints;
            // new_constraints[c->id].push_back(c);
            HighLevelNode_p m;

            generateChild(m, n, c);

            if (!m->valid)
                continue;

            // OPEN.push_back(m);
            // start = std::chrono::high_resolution_clock::now();
            OPEN.push(m);
            // end = std::chrono::high_resolution_clock::now();
            // elapsed = end - start;
            // printf("push cost=%f\n", elapsed.count());
            if (m->LB <= LB_min * sub_optimality)
                FOCAL.push(m);
            // OPEN.push_back(m);
            ++h_node_num;
            m->h_id = h_node_num;
        }
        old_node = n;
        old_node->parent = nullptr;
    }

    // success
    if (solved)
        solution = pathsToPlan(n->paths);
}

std::tuple<Path, int> ECBSW::getFocalPath3(Node *s, Node *g, LibCBS::Constraints &cs, int makespan, int id)
{
    int max_constraint_time = 0;
    for (auto c : cs)
    {
        max_constraint_time = std::max(c->t, max_constraint_time);
    }

    // f-value for online list
    FocalHeuristics f1Value;
    if (pathDist(id) > max_constraint_time)
    {
        f1Value = [&](FocalNode *n)
        { return n->g + pathDist(id, n->v); };
    }
    else
    {
        f1Value = [&](FocalNode *n)
        {
            return std::max(max_constraint_time + 1, n->g + pathDist(id, n->v));
        };
    }
    // f1Value = [&](FocalNode* n) { return n->g + pathDist(id, n->v); };
    ;

    while (PATH_TABLE.size() <= makespan)
    {
        PATH_TABLE.push_back(PATH_TABLE.back());
    }

    // std::cout << "PathTable updated in " << dt << std::endl;
    FocalHeuristics f2Value = [&](FocalNode *n)
    {
        if (n->g == 0)
            return 0;
        // last node
        if (n->g > makespan)
        {
            if (PATH_TABLE[makespan][n->v->id] != Solver::NIL and PATH_TABLE[makespan][n->v->id] != id)
                return n->p->f2 + 1;
        }
        else
        {
            // vertex conflict
            if (PATH_TABLE[n->g][n->v->id] != Solver::NIL and PATH_TABLE[n->g][n->v->id] != id)
            {
                return n->p->f2 + 1;

                // swap conflict
            }
            else if (PATH_TABLE[n->g][n->p->v->id] != Solver::NIL &&
                     PATH_TABLE[n->g - 1][n->v->id] ==
                         PATH_TABLE[n->g][n->p->v->id] &&
                     PATH_TABLE[n->g][n->p->v->id] != id)
            {
                return n->p->f2 + 1;
            }
        }
        return n->p->f2;
    };

    CompareFocalNode compareOPEN = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f1 != b->f1)
            return a->f1 > b->f1;

        return false;
    };

    CompareFocalNode compareFOCAL = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f2 != b->f2)
            return a->f2 > b->f2;
        if (a->f1 != b->f1)
            return a->f1 > b->f1;
        if (a->g != b->g)
            return a->g < b->g;
        return false;
    };

    CheckFocalFin checkFocalFin = [&](FocalNode *n)
    {
        return n->v == g && n->g > max_constraint_time;
    };

    CheckInvalidFocalNode checkInvalidFocalNode = [&](FocalNode *m)
    {
        for (auto c : cs)
        {
            if (m->g == c->t && m->v == c->v)
            {
                // vertex or swap conflict
                if (c->u == nullptr || c->u == m->p->v)
                    return true;
            }
        }
        return false;
    };

    auto p = getTimedPathByFocalSearch(s, g, sub_optimality, f1Value, f2Value,
                                       compareOPEN, compareFOCAL, checkFocalFin,
                                       checkInvalidFocalNode);
    return p;
}

std::tuple<Path, int> ECBSW::getFocalPath2(HighLevelNode_p h_node, int id)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);
    // pre processing
    // LibCBS::Constraints constraints;
    int max_constraint_time = 0;
    for (auto c : h_node->constraints[id])
    {
        max_constraint_time = std::max(c->t, max_constraint_time);
    }

    // f-value for online list
    FocalHeuristics f1Value;
    if (pathDist(id) > max_constraint_time)
    {
        f1Value = [&](FocalNode *n)
        { return n->g + pathDist(id, n->v); };
    }
    else
    {
        f1Value = [&](FocalNode *n)
        {
            return std::max(max_constraint_time + 1, n->g + pathDist(id, n->v));
        };
    }
    // f1Value = [&](FocalNode* n) { return n->g + pathDist(id, n->v); };
    const auto paths = h_node->paths;
    const int makespan = paths.getMakespan();

    while (PATH_TABLE.size() <= makespan)
    {
        PATH_TABLE.push_back(PATH_TABLE.back());
    }

    // std::cout << "PathTable updated in " << dt << std::endl;
    FocalHeuristics f2Value = [&](FocalNode *n)
    {
        if (n->g == 0)
            return 0;
        // last node
        if (n->g > makespan)
        {
            if (PATH_TABLE[makespan][n->v->id] != Solver::NIL and PATH_TABLE[makespan][n->v->id] != id)
                return n->p->f2 + 1;
        }
        else
        {
            // vertex conflict
            if (PATH_TABLE[n->g][n->v->id] != Solver::NIL and PATH_TABLE[n->g][n->v->id] != id)
            {
                return n->p->f2 + 1;

                // swap conflict
            }
            else if (PATH_TABLE[n->g][n->p->v->id] != Solver::NIL &&
                     PATH_TABLE[n->g - 1][n->v->id] ==
                         PATH_TABLE[n->g][n->p->v->id] &&
                     PATH_TABLE[n->g][n->p->v->id] != id)
            {
                return n->p->f2 + 1;
            }
        }
        return n->p->f2;
    };

    CompareFocalNode compareOPEN = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f1 != b->f1)
            return a->f1 > b->f1;

        return false;
    };

    CompareFocalNode compareFOCAL = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f2 != b->f2)
            return a->f2 > b->f2;
        if (a->f1 != b->f1)
            return a->f1 > b->f1;
        if (a->g != b->g)
            return a->g < b->g;
        return false;
    };

    CheckFocalFin checkFocalFin = [&](FocalNode *n)
    {
        return n->v == g && n->g > max_constraint_time;
    };

    CheckInvalidFocalNode checkInvalidFocalNode = [&](FocalNode *m)
    {
        for (auto c : h_node->constraints[id])
        {
            if (m->g == c->t && m->v == c->v)
            {
                // vertex or swap conflict
                if (c->u == nullptr || c->u == m->p->v)
                    return true;
            }
        }
        return false;
    };

    auto p = getTimedPathByFocalSearch(s, g, sub_optimality, f1Value, f2Value,
                                       compareOPEN, compareFOCAL, checkFocalFin,
                                       checkInvalidFocalNode);
    return p;
}

void ECBSW::invokeDP(HighLevelNode_p h_node, int id)
{

    // clearPathTableForAgent(h_node->paths.get(id), id);

    auto res = getFocalPath2(h_node, id);
    Path path = std::get<0>(res);
    int f_min = std::get<1>(res); // lower bound

    // failed to find path
    if (path.empty())
    {
        std::cout << "failed to find a path" << std::endl;
        h_node->valid = false;
        // updatePathTableForAgent(h_node->paths.get(id), id);
        return;
    }

    // Paths paths = h_node->paths;

    h_node->paths.insert(id, path);

    // LibCBS::Conflicts cfs;
    // auto start = std::chrono::high_resolution_clock::now();
    // LibCBS::getConflictDP(h_node->parent->paths, h_node->paths.get(id), id, cfs);
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed = end - start;
    // printf("get COnlifct DP @ invoke=%f\n", elapsed.count());
    // for (auto &cf : h_node->parent->conflicts)
    // {
    //     if (cf->a1 == id or cf->a2 == id)
    //         continue;
    //     cfs.push_back(cf);
    // }

    // h_node->f = cfs.size();
    // h_node->conflicts = cfs;
    // it is efficient to reuse past data
    // h_node->f = h_node->f -
    //             h_node->paths.countConflict(id, h_node->paths.get(id)) +
    //             h_node->paths.countConflict(id, paths.get(id));
    // h_node->paths = paths;
    h_node->makespan = h_node->paths.getMakespan();
    h_node->soc = h_node->paths.getSOC();
    // update lower bound and f_min
    h_node->LB = h_node->LB - h_node->f_mins[id] + f_min;
    h_node->f_mins[id] = f_min;

    // updatePathTableForAgent(h_node->parent->paths.get(id), id);
}

ECBSW::CompareHighLevelNode ECBSW::getMainObjective()
{
    CompareHighLevelNode compare = [&](HighLevelNode_p a, HighLevelNode_p b)
    {
        if (a->LB != b->LB)
            return a->LB > b->LB;
        return false;
    };
    return compare;
}

ECBSW::CompareHighLevelNode ECBSW::getFocalObjective()
{
    CompareHighLevelNode compare = [&](HighLevelNode_p a, HighLevelNode_p b)
    {
        if (a->f != b->f)
            return a->f > b->f;

        if (a->soc != b->soc)
            return a->soc > b->soc;

        return false;
    };
    return compare;
}

void ECBSW::setInitialHighLevelNodeFocal(HighLevelNode_p n)
{
    n->paths = Paths();
    n->constraints.resize(P->getNum(), {});
    std::vector<int> f_mins;
    for (int i = 0; i < P->getNum(); ++i)
    {
        Path path;
        if (P->getGoal(i)->id == P->getStart(i)->id)
            path = {P->getStart(i)};
        else
            path = getInitialPathFocal(n, i);
        std::cout << "path size for agent " << i << "  =  " << path.size() << "  " << P->getStart(i)->id << "  " << P->getGoal(i)->id << std::endl;
        // updatePathTableWithoutClear(i, path, n->paths);
        // #if PARALLEL
        // #pragma omp critical
        updatePathTableForAgent(path, i);
        // n->paths.insert(i, path);
        n->paths.append(path);

        f_mins.push_back(path.size() - 1);
        // #endif
    }

    n->makespan = n->paths.getMakespan();
    n->soc = n->paths.getSOC();
    LibCBS::getAllConflictsDP(n->paths, n->conflicts);
    n->f = n->conflicts.size();
    // n->f = n->paths.countConflict();
    n->valid = true;
    n->f_mins = f_mins;
    n->h_id = 0;
    n->LB = n->soc; // initial lower bound
    std::cout << "makespan =" << n->makespan << "  soc=" << n->soc << "  num of conflicts=" << n->f << std::endl;
}

Path ECBSW::getInitialPath(int id, const Paths &paths)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);
    Nodes config_g = P->getConfigGoal();

    Path path = {s};
    Node *p = s;
    int t = 1;
    const int makespan = paths.getMakespan();
    const int num_agents = P->getNum();
    while (p != g)
    {
        p = *std::min_element(p->neighbor.begin(), p->neighbor.end(),
                              [&](Node *a, Node *b)
                              {
                                  if (pathDist(id, a) != pathDist(id, b))
                                      return pathDist(id, a) < pathDist(id, b);
                                  if (t <= makespan)
                                  {
                                      Node *v;
                                      for (int i = 0; i < num_agents; ++i)
                                      {
                                          if (paths.empty(i))
                                              continue;
                                          v = paths.get(i, t);
                                          if (v == a)
                                              return false;
                                          if (v == b)
                                              return true;
                                      }
                                  }
                                  if (a != g && inArray(a, config_g))
                                      return false;
                                  if (b != g && inArray(b, config_g))
                                      return true;
                                  return false;
                              });
        path.push_back(p);
        ++t;
    }

    return path;
}

void ECBSW::chooseDisjointConflicts(HighLevelNode_p n, std::vector<LibCBS::Conflict_p> &conflicts, int k)
{
    std::sort(n->conflicts.begin(), n->conflicts.end(),
              [](const LibCBS::Conflict_p a, const LibCBS::Conflict_p b)
              {
                  return a->t < b->t;
              });
    std::set<int> used_agents;
    size_t count = 0;
    for (auto c : n->conflicts)
    {
        if (used_agents.find(c->a1) == used_agents.end() and used_agents.find(c->a2) == used_agents.end())
        {
            conflicts.push_back(c);
            count++;
            used_agents.insert(c->a1);
            used_agents.insert(c->a2);
        }
        // if (count == k)
        //     break;
    }
    // std::cout << conflicts.size() << std::endl;
    // exit(0);
}

void ECBSW::updatePathTableExcept(const Paths &paths, std::vector<int> &replanned_agents)
{
    const int makespan = paths.getMakespan();
    const int num_agents = paths.size();
    const int nodes_size = G->getNodesSize();
    while ((int)PATH_TABLE.size() < makespan + 1)
        PATH_TABLE.push_back(std::vector<int>(nodes_size, NIL));
#if PARALLEL
#pragma omp parallel for
#endif
    for (int i = 0; i < num_agents; i++)
    {
        if (std::find(replanned_agents.begin(), replanned_agents.end(), i) != replanned_agents.end() or paths.empty(i))
        {
            continue;
        }
        auto p = paths.get(i);
        for (int t = 0; t <= makespan; ++t)
            PATH_TABLE[t][p[t]->id] = i;
    }
}

Path ECBSW::getInitialPathFocal(HighLevelNode_p h_node, int id, std::vector<std::vector<int>> &PATH_TABLE2)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);

    // pre processing
    int max_constraint_time = 0;

    // f-value for online list
    FocalHeuristics f1Value;
    if (pathDist(id) > max_constraint_time)
    {
        f1Value = [&](FocalNode *n)
        { return n->g + pathDist(id, n->v); };
    }
    else
    {
        f1Value = [&](FocalNode *n)
        {
            return std::max(max_constraint_time + 1, n->g + pathDist(id, n->v));
        };
    }
    // f1Value = [&](FocalNode* n) { return n->g + pathDist(id, n->v); };
    const auto paths = h_node->paths;
    const int makespan = paths.getMakespan();
    if (PATH_TABLE2.size() == 0)
        PATH_TABLE2.push_back(std::vector<int>(G->getNodesSize(), NIL));
    while (PATH_TABLE2.size() < makespan + 1)
    {
        PATH_TABLE2.push_back(PATH_TABLE2.back());
    }

    // update PATH_TABLE

    FocalHeuristics f2Value = [&](FocalNode *n)
    {
        if (n->g == 0)
            return 0;
        if (makespan == 0)
            return 0;
        // last node
        if (n->g > makespan)
        {
            if (PATH_TABLE2[makespan][n->v->id] != Solver::NIL)
                return n->p->f2 + 1;
        }
        else
        {
            // vertex conflict
            // assert(n->g < PATH_TABLE.size());
            if (PATH_TABLE2[n->g][n->v->id] != Solver::NIL)
            {
                return n->p->f2 + 1;

                // swap conflict
            }
            else if (PATH_TABLE2[n->g][n->p->v->id] != Solver::NIL &&
                     PATH_TABLE2[n->g - 1][n->v->id] ==
                         PATH_TABLE2[n->g][n->p->v->id])
            {
                return n->p->f2 + 1;
            }
        }
        return n->p->f2;
    };

    CompareFocalNode compareOPEN = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f1 != b->f1)
            return a->f1 > b->f1;

        return false;
    };

    CompareFocalNode compareFOCAL = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f2 != b->f2)
            return a->f2 > b->f2;
        if (a->f1 != b->f1)
            return a->f1 > b->f1;
        if (a->g != b->g)
            return a->g < b->g;
        return false;
    };

    CheckFocalFin checkFocalFin = [&](FocalNode *n)
    {
        return n->v == g && n->g > max_constraint_time;
    };

    CheckInvalidFocalNode checkInvalidFocalNode = [&](FocalNode *m)
    {
        for (auto c : h_node->constraints[id])
        {
            if (m->g == c->t && m->v == c->v)
            {
                // vertex or swap conflict
                if (c->u == nullptr || c->u == m->p->v)
                    return true;
            }
        }
        return false;
    };

    auto p = getTimedPathByFocalSearch(s, g, sub_optimality, f1Value, f2Value,
                                       compareOPEN, compareFOCAL, checkFocalFin,
                                       checkInvalidFocalNode);
    // clear used path table

    return std::get<0>(p);
}

Path ECBSW::getInitialPathFocal(HighLevelNode_p h_node, int id)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);

    // pre processing
    int max_constraint_time = 0;
    // LibCBS::Constraints constraints;
    // int max_constraint_time = 0;
    // for (auto c : h_node->constraints)
    // {
    //     if (c->id == id)
    //     {
    //         constraints.push_back(c);
    //         if (c->v == g && c->u == nullptr)
    //         {
    //             max_constraint_time = std::max(max_constraint_time, c->t);
    //         }
    //     }
    // }

    // f-value for online list
    FocalHeuristics f1Value;
    if (pathDist(id) > max_constraint_time)
    {
        f1Value = [&](FocalNode *n)
        { return n->g + pathDist(id, n->v); };
    }
    else
    {
        f1Value = [&](FocalNode *n)
        {
            return std::max(max_constraint_time + 1, n->g + pathDist(id, n->v));
        };
    }
    // f1Value = [&](FocalNode* n) { return n->g + pathDist(id, n->v); };
    const auto paths = h_node->paths;
    const int makespan = paths.getMakespan();
    if (PATH_TABLE.size() == 0)
        PATH_TABLE.push_back(std::vector<int>(G->getNodesSize(), NIL));
    while (PATH_TABLE.size() < makespan + 1)
    {
        PATH_TABLE.push_back(PATH_TABLE.back());
    }

    // update PATH_TABLE

    FocalHeuristics f2Value = [&](FocalNode *n)
    {
        if (n->g == 0)
            return 0;
        if (makespan == 0)
            return 0;
        // last node
        if (n->g > makespan)
        {
            if (PATH_TABLE[makespan][n->v->id] != Solver::NIL)
                return n->p->f2 + 1;
        }
        else
        {
            // vertex conflict
            // assert(n->g < PATH_TABLE.size());
            if (PATH_TABLE[n->g][n->v->id] != Solver::NIL)
            {
                return n->p->f2 + 1;

                // swap conflict
            }
            else if (PATH_TABLE[n->g][n->p->v->id] != Solver::NIL &&
                     PATH_TABLE[n->g - 1][n->v->id] ==
                         PATH_TABLE[n->g][n->p->v->id])
            {
                return n->p->f2 + 1;
            }
        }
        return n->p->f2;
    };

    CompareFocalNode compareOPEN = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f1 != b->f1)
            return a->f1 > b->f1;

        return false;
    };

    CompareFocalNode compareFOCAL = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f2 != b->f2)
            return a->f2 > b->f2;
        if (a->f1 != b->f1)
            return a->f1 > b->f1;
        if (a->g != b->g)
            return a->g < b->g;
        return false;
    };

    CheckFocalFin checkFocalFin = [&](FocalNode *n)
    {
        return n->v == g && n->g > max_constraint_time;
    };

    CheckInvalidFocalNode checkInvalidFocalNode = [&](FocalNode *m)
    {
        for (auto c : h_node->constraints[id])
        {
            if (m->g == c->t && m->v == c->v)
            {
                // vertex or swap conflict
                if (c->u == nullptr || c->u == m->p->v)
                    return true;
            }
        }
        return false;
    };

    auto p = getTimedPathByFocalSearch(s, g, sub_optimality, f1Value, f2Value,
                                       compareOPEN, compareFOCAL, checkFocalFin,
                                       checkInvalidFocalNode);
    // clear used path table

    return std::get<0>(p);
}

void ECBSW::generateChild(HighLevelNode_p &m, const HighLevelNode_p n, const LibCBS::Constraint_p c)
{
    m = std::make_shared<HighLevelNode>(
        n->paths, n->constraints, n->makespan, n->soc, n->f, n->LB,
        n->f_mins, true);
    m->parent = n;
    m->constraints[c->id].push_back(c);
    m->max_constraint_time = std::max(n->max_constraint_time, c->t);
    m->replan = c->id;
    // m->depth = n->depth + 1;
    invokeDP(m, c->id);
}

ECBSW::HighLevelNode_p ECBSW::AddressConflictsParallel(HighLevelNode_p n, std::vector<LibCBS::Conflict_p> &conflicts, int LB_min)
{
    std::mutex mtx;
    HighLevelNode_p m = std::make_shared<HighLevelNode>(
        n->paths, n->constraints, n->makespan, n->soc, n->f, n->LB,
        n->f_mins, true);
    int makespan = n->paths.getMakespan();
    auto addressHelper = [&](std::vector<LibCBS::Conflict_p> mcfs)
    {
        // std::cout << "conflicts size=" << mcfs.size() << std::endl;
        for (auto &conflict : mcfs)
        {
            LibCBS::Constraints constraints = LibCBS::constraintsFromConflict(conflict);
            bool foundByPass = false;
            std::tuple<Path, int> res;
            int id;
            for (auto &c : constraints)
            {
                auto new_constraints = n->constraints[c->id];
                new_constraints.push_back(c);
                id = c->id;
                res = getFocalPath3(P->getStart(id), P->getGoal(id), new_constraints, makespan, id);
                if (std::get<0>(res).size() == 0)
                    continue;
                // generateChild(m, parent, c);

                if (std::get<1>(res) <= n->f_mins[id] * sub_optimality)
                {

                    foundByPass = true;
                    break;
                }
                else
                {
                }
            }

            if (foundByPass == true)
            {
                // assert(std::get<0>(res).size() != 0);
                // m->paths.insert(id, std::get<0>(res));
                m->paths.paths[id] = std::get<0>(res);
            }
            else
            {
                std::cout << "found error when resolving conflicts (" << conflict->a1 << "  " << conflict->a2 << "  " << conflict->t << std::endl;
            }
        }
    };

    std::vector<std::thread> threads;
    size_t conflictsPerThread = conflicts.size() / numThreads;

    // Launch threads
    for (size_t i = 0; i < numThreads; ++i)
    {
        size_t startIdx = i * conflictsPerThread;
        size_t endIdx = (i == numThreads - 1) ? conflicts.size() : (i + 1) * conflictsPerThread;

        auto m_conflicts = std::vector<LibCBS::Conflict_p>(conflicts.begin() + startIdx, conflicts.begin() + endIdx);
        // std::cout << startIdx << "   " << endIdx << "   " << conflicts.size() <<"   "<<m_c << std::endl;
        threads.emplace_back(addressHelper, m_conflicts);
        // addressHelper(m_conflicts);
    }

    for (auto &thread : threads)
    {
        thread.join();
    }
    m->conflicts.clear();
    m->paths.format();
    LibCBS::getAllConflictsParallel(m->paths, m->conflicts, numThreads);
    m->f = m->conflicts.size();
    std::cout << "m conflicts count=" << m->conflicts.size() << std::endl;
    return m;
}

// return path and f_min
std::tuple<Path, int> ECBSW::getTimedPathByFocalSearch(
    Node *const s, Node *const g,
    float w, // sub-optimality
    FocalHeuristics &f1Value, FocalHeuristics &f2Value,
    CompareFocalNode &compareOPEN, CompareFocalNode &compareFOCAL,
    CheckFocalFin &checkFocalFin, CheckInvalidFocalNode &checkInvalidFocalNode)
{
    auto getNodeName = [&](FocalNode *n)
    {
        int height = G->getHeight();
        int width = G->getWidth();
        long long result = static_cast<long long>(n->v->id) + static_cast<long long>(height) * static_cast<long long>(width) * static_cast<long long>(n->g);
        return result;
    };

    std::vector<FocalNode *> GC; // garbage collection
    auto createNewNode = [&](Node *v, int g, int f1, int f2, FocalNode *p)
    {
        FocalNode *new_node = new FocalNode{v, g, f1, f2, p};
        GC.push_back(new_node);
        return new_node;
    };

    // OPEN, FOCAL, CLOSE
    std::priority_queue<FocalNode *, std::vector<FocalNode *>, CompareFocalNode>
        OPEN(compareOPEN);
    // std::unordered_map<std::string, bool> CLOSE;
    std::unordered_map<long long, bool> CLOSE;
    using FocalList = std::priority_queue<FocalNode *, std::vector<FocalNode *>,
                                          CompareFocalNode>;
    FocalList FOCAL(compareFOCAL);

    // initial node
    FocalNode *n;
    n = createNewNode(s, 0, 0, 0, nullptr);
    n->f1 = f1Value(n);
    n->f2 = f2Value(n);
    OPEN.push(n);
    FOCAL.push(n);
    int f1_min = n->f1;

    // main loop
    bool invalid = true;
    while (!OPEN.empty())
    {
        // check time limit
        if (overCompTime())
        {
            std::cout << "time out" << std::endl;
            break;
        }
        // std::cout<<n->g<<" :";
        // n->v->println();
        /*
         * update FOCAL list
         * see the high-level search
         */
        while (!OPEN.empty() && CLOSE.find(getNodeName(OPEN.top())) != CLOSE.end())
            OPEN.pop();
        if (OPEN.empty())
            break; // failed
        if (f1_min != OPEN.top()->f1)
        {
            f1_min = OPEN.top()->f1;
            float f1_bound = f1_min * w;
            std::vector<FocalNode *> tmp;
            FocalList EMPTY(compareFOCAL);
            FOCAL = EMPTY;
            while (!OPEN.empty())
            {
                FocalNode *top = OPEN.top();
                OPEN.pop();
                // already searched by focal
                if (CLOSE.find(getNodeName(top)) != CLOSE.end())
                    continue;
                tmp.push_back(top); // escape
                if ((float)top->f1 > f1_bound)
                    break;       // higher than f1_bound
                FOCAL.push(top); // lower than f1_bound
            }
            for (auto ele : tmp)
                OPEN.push(ele); // back
        }

        // focal minimum node
        n = FOCAL.top();
        FOCAL.pop();
        if (CLOSE.find(getNodeName(n)) != CLOSE.end())
            continue;
        CLOSE[getNodeName(n)] = true;

        // check goal condition
        if (checkFocalFin(n))
        {
            invalid = false;
            break;
        }

        // expand
        Nodes C = n->v->neighbor;
        C.push_back(n->v);
        for (auto u : C)
        {
            int g_cost = n->g + 1;
            FocalNode *m = createNewNode(u, g_cost, 0, 0, n);
            // set heuristics
            m->f1 = f1Value(m);
            m->f2 = f2Value(m);
            // already searched?
            if (CLOSE.find(getNodeName(m)) != CLOSE.end())
                continue;
            // check constraints
            if (checkInvalidFocalNode(m))
                continue;
            // update open list
            OPEN.push(m);
            if (m->f1 <= f1_min * w)
                FOCAL.push(m);
        }
    }

    Path path;
    // success
    if (!invalid)
        path = getPathFromFocalNode(n);
    std::tuple<Path, int> ret = std::make_tuple(path, f1_min);

    // free
    for (auto p : GC)
        delete p;

    return ret;
}

// reconstruct a path from focal node in the low-level node
Path ECBSW::getPathFromFocalNode(FocalNode *_n)
{
    Path path;
    FocalNode *n = _n;
    while (n != nullptr)
    {
        path.push_back(n->v);
        n = n->p;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void ECBSW::setParams(int argc, char *argv[])
{
    struct option longopts[] = {
        {"sub-optimality", required_argument, 0, 'w'},
        {0, 0, 0, 0},
    };
    optind = 1; // reset
    int opt, longindex;
    while ((opt = getopt_long(argc, argv, "w:", longopts, &longindex)) != -1)
    {
        switch (opt)
        {
        case 'w':
            sub_optimality = std::atof(optarg);
            if (sub_optimality < 1)
                halt("sub-optimality should be >= 1");
            solver_name = ECBSW::SOLVER_NAME + "-" + std::to_string(sub_optimality);
            break;
        default:
            break;
        }
    }
}

void ECBSW::printHelp()
{
    std::cout << ECBSW::SOLVER_NAME << "\n"
              << "  -w --sub-optimality [NUMBER]"
              << "  "
              << "sub-optimality >= 1" << std::endl;
}
