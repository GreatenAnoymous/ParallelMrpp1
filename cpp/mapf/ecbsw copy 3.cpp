#include "ecbsw.hpp"
#include <chrono>
#include <omp.h>
#include <thread>

const std::string ECBSW::SOLVER_NAME = "ECBSW";
const float ECBSW::DEFAULT_SUB_OPTIMALITY = 2.0;

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

void ECBSW::replan_path(HighLevelNode_p h_node, LibCBS::Constraints &new_constraints, std::vector<int> &replanned, std::vector<Path> &replanned_paths, std::vector<int> &fmins, std::vector<LibCBS::Conflicts> &new_conflicts, int thread_id)
{

    int id = replanned[thread_id];

    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);

    // pre processing
    int max_constraint_time = h_node->max_constraint_time;
    LibCBS::Constraints constraints = h_node->constraints[id];

    constraints.push_back(new_constraints[thread_id]);

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
    auto paths = h_node->paths;
    int makespan = paths.getMakespan();
    // std::vector<std::map<int, int>> tmp_table(makespan + 1, std::map<int, int>());
    // for (auto a : replanned)
    // {
    //     if (a == id)
    //         continue;
    //     for (int t = 0; t <= makespan; t++)
    //     {
    //         assert(t < tmp_table.size());
    //         tmp_table[t][h_node->paths.get(a, t)->id] = a;
    //     }
    // }

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
            if (PATH_TABLE[makespan][n->v->id] != Solver::NIL and PATH_TABLE[makespan][n->v->id] != id)
                return n->p->f2 + 1;
            // if (tmp_table[makespan].find(n->v->id) != tmp_table[makespan].end())
            //     return n->p->f2 + 1;
        }
        else
        {
            // vertex conflict
            if (PATH_TABLE[n->g][n->v->id] != Solver::NIL and PATH_TABLE[makespan][n->v->id] != id)
            {
                return n->p->f2 + 1;
                // swap conflict
            }

            if (PATH_TABLE[n->g][n->p->v->id] != Solver::NIL &&
                PATH_TABLE[n->g - 1][n->v->id] ==
                    PATH_TABLE[n->g][n->p->v->id] and
                PATH_TABLE[n->g][n->p->v->id] != id)
            {
                return n->p->f2 + 1;
            }
            // if (tmp_table[n->g].find(n->v->id) != tmp_table[n->g].end())
            //     return n->p->f2 + 1;

            // if (tmp_table[n->g].find(n->v->id) != tmp_table[n->g].end() and tmp_table[n->g - 1].find(n->v->id) != tmp_table[n->g - 1].end())
            // {
            //     if (
            //         tmp_table[n->g - 1][n->v->id] ==
            //         tmp_table[n->g][n->p->v->id])
            //     {
            //         return n->p->f2 + 1;
            //     }
            // }
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
        for (auto c : constraints)
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
    replanned_paths[thread_id] = std::get<0>(p);
    if (replanned_paths[thread_id].size() == 0)
        return;
    fmins[thread_id] = std::get<1>(p);

    // count new conflicts
    for (auto a : replanned)
    {
        paths.clear(a); // clear replanning agents
    }
    paths.format();
    makespan = paths.getMakespan();
    int num_agents = paths.size();
    const int path_size = replanned_paths[thread_id].size();

    // std::cout << "path size=" << path_size << "  makespan=" << makespan << std::endl;
    // std::cout << "current replanning for " << replanned[thread_id] << std::endl;
    for (int i = 0; i < num_agents; ++i)
    {
        if (paths.empty(i))
            continue;
        int tmax = std::max(path_size - 1, makespan);
        for (int t = 1; t <= tmax;)
        {
            // if (i == 100 and replanned[thread_id] == 913)
            //     std::cout << "checking " << i << "   " << replanned[thread_id] << "   " << t << "  " << replanned_paths[thread_id].back()->id << "  " << paths.get(i, 47)->id << std::endl;
            if (t > makespan)
            {
                if (replanned_paths[thread_id][t] == paths.get(i, makespan))
                {
                    // ++cnt;

                    {
                        LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(id, i, t, replanned_paths[thread_id][t]);
                        new_conflicts[thread_id].push_back(conflict);
                        // std::cout << "hay" << std::endl;
                    }

                    break;
                }
                int dt = paths.get(i, makespan)->manhattanDist(replanned_paths[thread_id][t]);
                t += std::max(dt - 1, 1);
                // t++;
                continue;
            }
            if (t > path_size - 1)
            {
                if (replanned_paths[thread_id].back() == paths.get(i, t))
                {
                    // ++cnt;

                    {
                        LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(id, i, t, replanned_paths[thread_id].back());
                        new_conflicts[thread_id].push_back(conflict);
                        // std::cout << "hay" << std::endl;
                    }

                    break;
                }
                int dt = paths.get(i, t)->manhattanDist(replanned_paths[thread_id].back());
                t += std::max(dt - 1, 1);
                // t++;
                continue;
            }

            // vertex conflict
            if (paths.get(i, t) == replanned_paths[thread_id][t])
            {

                {
                    LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(id, i, t, replanned_paths[thread_id][t]);
                    new_conflicts[thread_id].push_back(conflict);
                    // std::cout << "hay" << std::endl;
                }

                t++;

                continue;
            }
            // swap conflict
            if (paths.get(i, t) == replanned_paths[thread_id][t - 1] && paths.get(i, t - 1) == replanned_paths[thread_id][t])
            {

                {
                    LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(id, i, t, replanned_paths[thread_id][t], replanned_paths[thread_id][t - 1]);
                    new_conflicts[thread_id].push_back(conflict);
                    // std::cout << "hay" << std::endl;
                }
            }
            int dt = paths.get(i, t)->manhattanDist(replanned_paths[thread_id][t]) / 2;
            t += std::max(dt - 1, 1);
            // t++;
        }
    }

    // std::cout << "found new conflicts" << new_conflicts[thread_id].size() << std::endl;
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

    OPEN.push(n);
    FOCAL.push(n);

    int LB_min = n->LB;

    // main loop
    int h_node_num = 1;
    numExpansions = 0;
    auto startCounter = std::chrono::high_resolution_clock::now();
    auto endCounter = std::chrono::high_resolution_clock::now();
    while (!OPEN.empty())
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
            std::cout << "CBS tree depth=" << n->constraints.size() << " num expansions=" << numExpansions << std::endl;
            solved = true;
            break;
        }

        LibCBS::Conflicts cfs;
        LibCBS::Constraints disjoints;
        auto start = std::chrono::high_resolution_clock::now();
        chooseDisjointConflicts(n, cfs, 3);

        // for(auto cf: cfs){
        //     for(auto c: n->constraints){
        //         if(c->t==cf->t and (cf->a1==c->id or cf->a2==c->id) and (cf->v==c->v) and (cf->u==c->u)){
        //             assert(false);
        //         }
        //     }
        // }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        printf("choose conflicts cost=%f\n", elapsed.count());
        std::vector<int> replanned;
        start = std::chrono::high_resolution_clock::now();
        createDisjointConstraints(cfs, disjoints, replanned);
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        printf("create constraints cost=%f\n", elapsed.count());
        // for (auto i : replanned)
        // {
        //     std::cout << "replan for agent " << i << std::endl;
        // }
        // std::cout << "###############" << std::endl;
        std::vector<Path> replaned_paths(replanned.size(), Path());
        std::vector<int> fmins(replanned.size(), 0);
        std::vector<LibCBS::Conflicts> new_conflicts(replanned.size(), LibCBS::Conflicts());
        int pN = replanned.size();
        std::vector<std::thread> threads;

        // start = std::chrono::high_resolution_clock::now();
        // if (old_node == n->parent)
        // {
        //     if (old_node != nullptr)
        //     {
        //         for (auto a : n->replan_agents)
        //         {
        //             clearPathTableForAgent(old_node->paths.get(a), a);
        //             updatePathTableForAgent(n->paths.get(a), a);
        //         }
        //     }
        // }
        // else
        // {
        //     std::cout << "rebuilt table" << std::endl;
        //     clearPathTable(old_node->paths);
        //     updatePathTableExcept(n->paths, replanned);
        // }
        if (old_node != nullptr)
        {
            std::vector<int> agents;
            clearPathTable(old_node->paths);
            updatePathTableExcept(n->paths, agents);
        }
        else
        {
            // for (auto a : replanned)
            // {
            //     clearPathTableForAgent(n->paths.get(a), a);
            // }
        }
        // end = std::chrono::high_resolution_clock::now();
        // elapsed = end - start;
        // printf("update path table cost=%f\n", elapsed.count());
        start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < pN; ++i)
        {
            threads.emplace_back(&ECBSW::replan_path, this, n, std::ref(disjoints),
                                 std::ref(replanned), std::ref(replaned_paths),
                                 std::ref(fmins), std::ref(new_conflicts), i);
        }

        // Join threads (wait for all threads to finish)
        for (auto &thread : threads)
        {
            thread.join();
        }
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        printf("replan path cost=%f\n", elapsed.count());
        // // // std::cout << "?????" << std::endl;
        start = std::chrono::high_resolution_clock::now();
        std::vector<HighLevelNode_p> children;
        create_nodes(n, cfs, disjoints, replanned, replaned_paths, fmins, new_conflicts, children);
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        printf("create nodes=%f\n", elapsed.count());
        for (auto c : children)
        {
            if (!c->valid)
                continue;
            OPEN.push(c);
            if (c->LB <= LB_min * sub_optimality)
                FOCAL.push(c);
        }
        old_node->parent = nullptr;
        old_node = n;
        // LibCBS::Constraints constraints = LibCBS::constraintsFromConflict(LibCBS::chooseConflict(n->conflicts));
        // create new nodes
        // for (auto c : constraints)
        // {
        //     LibCBS::Constraints new_constraints = n->constraints;
        //     new_constraints.push_back(c);
        //     HighLevelNode_p m = std::make_shared<HighLevelNode>(
        //         n->paths, new_constraints, n->makespan, n->soc, n->f, n->LB,
        //         n->f_mins, true);
        //     m->conflicts = n->conflicts;

        //     // invokeDP(m, c->id);

        //     if (!m->valid)
        //         continue;
        //     OPEN.push(m);
        //     if (m->LB <= LB_min * sub_optimality)
        //         FOCAL.push(m);
        //     ++h_node_num;
        //     m->h_id = h_node_num;
        //     break;
        // }
    }

    // success
    if (solved)
        solution = pathsToPlan(n->paths);
}

void ECBSW::createDisjointConstraints(const LibCBS::Conflicts &conflicts, LibCBS::Constraints &constraints, std::vector<int> &replanned_agents)
{
    for (auto cf : conflicts)
    {
        std::cout << "conflict addressing " << cf->a1 << "  " << cf->a2 << "   " << cf->t << std::endl;
        LibCBS::Constraint_p c_i =
            std::make_shared<LibCBS::Constraint>(cf->a1, cf->t, cf->v, cf->u);
        LibCBS::Constraint_p c_j;
        if (cf->u == nullptr)
            c_j = std::make_shared<LibCBS::Constraint>(cf->a2, cf->t, cf->v, cf->u);
        else
            c_j = std::make_shared<LibCBS::Constraint>(cf->a2, cf->t, cf->u, cf->v);
        constraints.push_back(c_i);
        replanned_agents.push_back(c_i->id);
        constraints.push_back(c_j);
        replanned_agents.push_back(c_j->id);
    }
}

void ECBSW::createGreedyConstraints(const LibCBS::Conflicts &conflicts, LibCBS::Constraints &constraints)
{
    std::random_device rd;

    // Use the random device to seed the random number engine
    std::default_random_engine generator(rd());

    // Create a uniform distribution for integers in the range [0, 1]
    std::uniform_int_distribution<int> distribution(0, 1);

    // Generate a random integer using the distribution

    for (auto cf : conflicts)
    {
        int randomValue = distribution(generator);
        if (randomValue == 0)
        {
            LibCBS::Constraint_p c_i =
                std::make_shared<LibCBS::Constraint>(cf->a1, cf->t, cf->v, cf->u);
            constraints.push_back(c_i);
        }
        else
        {
            LibCBS::Constraint_p c_j;
            if (cf->u == nullptr)
                c_j = std::make_shared<LibCBS::Constraint>(cf->a2, cf->t, cf->v, cf->u);
            else
                c_j = std::make_shared<LibCBS::Constraint>(cf->a2, cf->t, cf->u, cf->v);
            constraints.push_back(c_j);
        }
    }
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
    std::vector<int> f_mins; // vector of costs for respective paths
                             // #if PARALLEL
                             // #pragma omp parallel for
                             // #endif
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
    // clearPathTable(n->paths);
    // std::cout<<"debug??? "<<paths.size()<<"  "<<paths.getMakespan()<<std::endl;

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
    std::cout << conflicts.size() << std::endl;
    exit(0);
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

void ECBSW::ByPass(HighLevelNode_p &n)
{
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
        return n->v->id + G->getHeight() * G->getWidth() * n->g;
        // return std::to_string(n->v->id) + "-" + std::to_string(n->g);
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
    std::unordered_map<int, bool> CLOSE;
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

void ECBSW::create_nodes(const HighLevelNode_p n, const LibCBS::Conflicts &cfs, const LibCBS::Constraints &constraints, const std::vector<int> &replanned, const std::vector<Path> &replanned_paths, const std::vector<int> &fmins, const std::vector<LibCBS::Conflicts> &new_conflicts, std::vector<HighLevelNode_p> &children)
{

    std::vector<std::vector<int>> result = {{}};
    // auto start = std::chrono::high_resolution_clock::now();
    for (const auto &cf : cfs)
    {
        std::vector<std::vector<int>> temp;
        std::vector<int> current_vec = {cf->a1, cf->a2};

        for (const auto &element : current_vec)
        {
            for (const auto &combination : result)
            {
                auto current_combination = combination;
                current_combination.push_back(element);
                temp.push_back(current_combination);
            }
        }

        result = temp;
    }
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed = end - start;
    // printf("@@@@ cartesian product cost=%f\n", elapsed.count());
    children.resize(result.size(), nullptr);
    // threading for node i
    auto threading = [&](int i)
    {
        auto ci = std::make_shared<HighLevelNode>(
            n->paths, n->constraints, n->makespan, n->soc, n->f, n->LB,
            n->f_mins, true);

        // printf("@@@@ old count cost=%f\n", elapsed.count());
        // std::cout << "current conflicts" << children[i]->conflicts.size() << std::endl;

        int num_agents = ci->paths.size();
        std::map<int, int> agent_id;

        for (auto a : result[i])
        {
            int id = 0;
            // std::cout << replanned.size() << "   " << constraints.size() << std::endl;
            // std::cout << "debug " << a << std::endl;
            for (id = 0; id < replanned.size(); id++)
            {
                if (replanned[id] == a)
                    break;
            }
            agent_id[a] = id;
            if (replanned_paths[id].empty())
            {
                ci->valid = false;
                children[i] = ci;
                return;
            }
            ci->paths.insert(a, replanned_paths[id]);
        }

        auto start = std::chrono::high_resolution_clock::now();

        for (auto cf : n->conflicts)
        {
            bool conflicted = false;
            for (auto a : result[i])
            {
                if (a == cf->a1 or a == cf->a2)
                {
                    conflicted = true;
                    break;
                }
            }
            if (!conflicted)
                ci->conflicts.push_back(cf);
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        int makespan = ci->paths.getMakespan();
        for (auto a : result[i])
        {
            int id = agent_id[a];
            for (auto c : new_conflicts[id])
            {
                ci->conflicts.push_back(c);
            }
        }
        std::vector<int> others;
        for (int id = 0; id < replanned.size(); id++)
        {
            if (std::find(result[i].begin(), result[i].end(), replanned[id]) == result[i].end())
                others.push_back(id);
        }

        for (int k = 0; k < result[i].size(); k++)
        {
            int ak = result[i][k];
            int idk = agent_id[ak];
            int lk = replanned_paths[idk].size();

            for (auto id : others)
            {
                int aj = replanned[id];
                // std::cout << "other agent " << aj << "  " << lk << "  " << makespan << std::endl;
                int tmax = std::max(lk - 1, makespan);
                for (int t = 1; t <= tmax;)
                {

                    if (t > makespan)
                    {
                        if (replanned_paths[idk][t] == ci->paths.get(aj, makespan))
                        {
                            // ++cnt;

                            {
                                LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(ak, aj, t, replanned_paths[idk][t]);
                                ci->conflicts.push_back(conflict);
                            }

                            break;
                        }
                        // std::cout<<"ci "<<ci->paths.get(aj, makespan)->id<<std::endl;
                        // std::cout<<t<<"   "<<replanned_paths[idk].size()<<std::endl;

                        int dt = ci->paths.get(aj, makespan)->manhattanDist(replanned_paths[idk][t]);
                        t += std::max(dt - 1, 1);
                        // t++;
                        continue;
                    }
                    if (t > lk - 1)
                    {
                        if (replanned_paths[idk].back() == ci->paths.get(aj, t))
                        {
                            // ++cnt;

                            {
                                LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(ak, aj, t, replanned_paths[idk].back());
                                ci->conflicts.push_back(conflict);
                            }

                            break;
                        }
                        int dt = ci->paths.get(aj, t)->manhattanDist(replanned_paths[idk].back());
                        t += std::max(dt - 1, 1);
                        // t++;
                        continue;
                    }

                    // vertex conflict
                    // std::cout<<aj<<"   "<<t<<"   "<<makespan <<"  "<<ci->paths.size()<<"   "<<ci->paths.get(aj).size()<<std::endl;
                    if (ci->paths.get(aj, t) == replanned_paths[idk][t])
                    {

                        {
                            LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(ak, aj, t, replanned_paths[idk][t]);
                            ci->conflicts.push_back(conflict);
                        }
                        int dt = ci->paths.get(aj, t)->manhattanDist(replanned_paths[idk][t]) / 2;
                        t += std::max(dt - 1, 1);
                        // t++;
                        continue;
                    }
                    // swap conflict
                    if (ci->paths.get(aj, t) == replanned_paths[idk][t - 1] && ci->paths.get(aj, t - 1) == replanned_paths[idk][t])
                    {

                        {
                            LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(ak, aj, t, replanned_paths[idk][t], replanned_paths[idk][t - 1]);
                            ci->conflicts.push_back(conflict);
                        }
                    }
                    // int dt = ci->paths.get(aj, t)->manhattanDist(replanned_paths[idk][t]) / 2;
                    // t += std::max(dt - 1, 1);
                    t++;
                }
            }

            for (int j = k + 1; j < result[i].size(); j++)
            {
                int aj = result[i][j];
                int idj = agent_id[aj];
                int lj = replanned_paths[idj].size();
                int tmax = std::max(lk, lj);
                for (int t = 1; t < tmax;)
                {

                    if (t >= lk)
                    {
                        if (replanned_paths[idj][t] == replanned_paths[idk].back())
                        {
                            LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(ak, aj, t, replanned_paths[idj][t]);
                            ci->conflicts.push_back(conflict);
                        }
                        // int dt = replanned_paths[idj][t]->manhattanDist(replanned_paths[idk].back());
                        // t += std::max(dt - 1, 1);
                        t++;
                        continue;
                    }
                    if (t >= lj)
                    {
                        // std::cout<<t<<"  "<<replanned_paths[idk].size()<<"  "<<replanned_paths[idj].size()<<"  "<<std::endl;
                        if (replanned_paths[idk][t] == replanned_paths[idj].back())
                        {
                            LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(ak, aj, t, replanned_paths[idj].back());
                            ci->conflicts.push_back(conflict);
                        }
                        // int dt = replanned_paths[idk][t]->manhattanDist(replanned_paths[idj].back());
                        // t += std::max(dt - 1, 1);
                        t++;
                        continue;
                    }
                    // if ((ak == 300 and aj == 418) or (ak == 418 and aj == 300))
                    //     std::cout << "checking " << ak << "   " << aj << "   " << t << "  " << replanned_paths[idk][126]->id << "  " << replanned_paths[idk][125]->id << "  " << ci->paths.get(aj, 126)->id << "  " << ci->paths.get(aj, 125)->id << std::endl;

                    if (replanned_paths[idj][t] == replanned_paths[idk][t])
                    {
                        LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(ak, aj, t, replanned_paths[idj][t]);
                        ci->conflicts.push_back(conflict);
                    }

                    if (replanned_paths[idj][t - 1] == replanned_paths[idk][t] and replanned_paths[idj][t] == replanned_paths[idk][t - 1])
                    {
                        LibCBS::Conflict_p conflict = std::make_shared<LibCBS::Conflict>(ak, aj, t, replanned_paths[idk][t], replanned_paths[idk][t - 1]);
                        ci->conflicts.push_back(conflict);
                    }
                    // int dt = replanned_paths[idj][t]->manhattanDist(replanned_paths[idk][t]) / 2;
                    // t += std::max(dt - 1, 1);
                    t++;

                    // edge
                }
            }
        }

        ci->parent = n;

        for (auto a : result[i])
        {
            int id = agent_id[a];
            // std::cout << replanned.size() << "   " << constraints.size() << std::endl;
            // std::cout<<ci->constraints.size()<<"  "<<n->constraints.size()<<"   "<<a<<std::endl;
            ci->constraints[a].push_back(constraints[id]);
            ci->f_mins[a] = fmins[id];
            ci->replan_agents.push_back(a);
        }
        ci->parent = n;
        ci->f = ci->conflicts.size();
        // assert(ci->f == ci->paths.countConflict());
        ci->makespan = makespan;
        ci->soc = ci->paths.getSOC();
        children[i] = ci;

        // auto cs = LibCBS::getFirstConstraints(ci->paths);
        // bool found = false;
        // for (auto cf : ci->conflicts)
        // {

        //     std::set<int> set1 = {cs[0]->id, cs[1]->id};
        //     std::set<int> set2 = {cf->a1, cf->a2};
        //     if (set1 == set2)
        //     {
        //         if (cs[0]->id == cf->a1 and cs[0]->v == cf->v and cs[0]->u == cf->u)
        //         {
        //             found = true;
        //             break;
        //         }
        //         if (cs[1]->id == cf->a1 and cs[1]->v == cf->v and cs[1]->u == cf->u)
        //         {
        //             found = true;
        //             break;
        //         }
        //     }
        // }
        // if (found == false)
        // {
        //     std::cout << result[i][0] << " conflicts missinng " << cs[0]->id << "   " << cs[1]->id << "   " << cs[0]->t << "   " << cs[0]->v->id << "   " << ((cs[0]->u == nullptr) ? -1 : cs[0]->u->id) << std::endl;
        //     for (auto cf : ci->conflicts)
        //     {
        //         std::cout << "conflict debug " << cf->a1 << "  " << cf->a2 << "  " << cf->t << std::endl;
        //     }

        //     assert(false);
        // }
        // end = std::chrono::high_resolution_clock::now();
        // elapsed = end - start;
        // printf("@@@@ one thread cost=%f\n", elapsed.count());
        // std::cout << "all children created" << std::endl;
    };

    std::vector<std::thread> threads;

    // start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < result.size(); ++i)
    {
        threads.emplace_back(threading, i);
    }
    // // end = std::chrono::high_resolution_clock::now();
    // // elapsed = end - start;
    // // printf("@@@@ create threads conflict=%f\n", elapsed.count());
    // // start = std::chrono::high_resolution_clock::now();
    // // Join the threads
    for (auto &thread : threads)
    {
        thread.join();
    }
    // end = std::chrono::high_resolution_clock::now();
    // elapsed = end - start;
    // printf("@@@@ total count conflict=%f\n", elapsed.count());

    // start=std::chrono::high_resolution_clock::now();
    // for (int i = 0; i < result.size(); i++)
    // {
    //     // std::cout << "threading " << i << std::endl;
    //     auto start = std::chrono::high_resolution_clock::now();
    //     threading(i);
    //     auto end = std::chrono::high_resolution_clock::now();
    //     std::chrono::duration<double> elapsed = end - start;
    //     printf("@@@@ count conflict=%f\n", elapsed.count());
    // }
    // end=std::chrono::high_resolution_clock::now();
    // elapsed = end - start;
    // printf("@@@@ total count conflict=%f\n", elapsed.count());
}
