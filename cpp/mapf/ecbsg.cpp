#include "ecbsg.hpp"
#include <chrono>
#include <omp.h>
#include <map>

const int num_threads = 8;

// Define a hash function for int keys

const std::string ECBSG::SOLVER_NAME = "ECBSG";
const float ECBSG::DEFAULT_SUB_OPTIMALITY = 2.0;

ECBSG::ECBSG(Problem *_P) : Solver(_P)
{
    sub_optimality = DEFAULT_SUB_OPTIMALITY;
    solver_name = ECBSG::SOLVER_NAME + "-" + std::to_string(sub_optimality);
}

ECBSG::ECBSG(Graph *graph, Config starts, Config goals) : Solver(graph, starts, goals)
{
    sub_optimality = DEFAULT_SUB_OPTIMALITY;
    solver_name = ECBSG::SOLVER_NAME + "-" + std::to_string(sub_optimality);
    agents_division = std::vector<std::set<size_t>>(num_threads, std::set<size_t>());
}

void ECBSG::run()
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
    // OPEN.push_back(n);
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

        /*
         *  update focal list
         */
        // pop out invalid nodes
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
        // n = OPEN.back();
        // OPEN.pop_back();
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
            // if (c.size() == 0)
            //     std::cout << "no conflicts validation" << std::endl;
            std::cout << "CBS tree depth=" << n->depth << " num expansions=" << numExpansions << std::endl;
            solved = true;
            break;
        }

        bool byPassing = true;

        while (byPassing)
        {
            if (n->conflicts.size() == 0)
            {
                std::cout << "CBS tree depth=" << n->depth << " num expansions=" << numExpansions << std::endl;
                solved = true;
                if (solved)
                    solution = pathsToPlan(n->paths);
                return;
            }
            LibCBS::Constraints constraints = LibCBS::constraintsFromConflict(LibCBS::chooseConflict(n->conflicts));
            bool foundByPass = false;
            HighLevelNode_p m;
            for (auto c : constraints)
            {

                generateChild(m, n, c);
                // std::cout<<m->conflicts.size()<<"   "<<n->conflicts.size()<<std::endl;
                if (m->valid == true and m->LB <= LB_min * sub_optimality and m->conflicts.size() < n->conflicts.size())
                {
                    foundByPass = true;
                    std::cout << "agent " << c->id << "  path replanned" << std::endl;
                    updatePathTableForAgent(m->paths.get(c->id), c->id);
                    break;
                }
            }
            if (foundByPass == false)
                break;
            else
            {
                old_node = n;
                n = m;
            }
            numExpansions++;
            std::cout << "Number of Node expansions BYpass=" << numExpansions << " number of conflicts=" << n->f << std::endl;
        }

        LibCBS::Constraints constraints = LibCBS::constraintsFromConflict(LibCBS::chooseConflict(n->conflicts));
        // std::cout<<n->conflicts[0]->a1<<" "<<n->conflicts[0]->a2<<"  "<<n->conflicts[0]->t
        // LibCBS::Constraints constraints = LibCBS::getConstraintsFast(n->paths);
        // LibCBS::Constraints constraints = LibCBS::getFirstConstraints(n->paths);
        // if (constraints.empty())
        // {
        //     solved = true;
        //     break;
        // }

        // check if still in the same branch
        // auto start = std::chrono::high_resolution_clock::now();
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
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> elapsed = end - start;
        // printf("update table time cost=%f\n", elapsed.count());
        // create new nodes

        for (auto c : constraints)
        {
            // std::vector<LibCBS::Constraints> new_constraints = n->constraints;
            // new_constraints[c->id].push_back(c);
            HighLevelNode_p m;
            auto start = std::chrono::high_resolution_clock::now();
            generateChild(m, n, c);

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            printf("invoke time cost=%f\n", elapsed.count());
            if (!m->valid)
                continue;

            // OPEN.push_back(m);
            start = std::chrono::high_resolution_clock::now();
            OPEN.push(m);
            end = std::chrono::high_resolution_clock::now();
            elapsed = end - start;
            printf("push cost=%f\n", elapsed.count());
            if (m->LB <= LB_min * sub_optimality)
                FOCAL.push(m);
            // OPEN.push_back(m);
            ++h_node_num;
            m->h_id = h_node_num;
        }
        old_node = n;
    }

    // success
    if (solved)
        solution = pathsToPlan(n->paths);
}

void ECBSG::generateChild(HighLevelNode_p &m, const HighLevelNode_p n, const LibCBS::Constraint_p c)
{
    m = std::make_shared<HighLevelNode>(
        n->paths, n->constraints, n->makespan, n->soc, n->f, n->LB,
        n->f_mins, true);
    m->parent = n;
    m->constraints[c->id].push_back(c);
    m->max_constraint_time = std::max(n->max_constraint_time, c->t);
    m->replan = c->id;
    m->depth = n->depth + 1;

    invokeDP(m, c->id);
}

ECBSG::CompareHighLevelNode ECBSG::getMainObjective()
{
    CompareHighLevelNode compare = [&](HighLevelNode_p a, HighLevelNode_p b)
    {
        if (a->LB != b->LB)
            return a->LB > b->LB;
        return false;
    };
    return compare;
}

ECBSG::CompareHighLevelNode ECBSG::getFocalObjective()
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

void ECBSG::setInitialHighLevelNode(HighLevelNode_p n)
{
    n->paths = Paths(P->getNum());

    std::vector<int> f_mins; // vector of costs for respective paths
    for (int i = 0; i < P->getNum(); ++i)
    {
        Path path = getInitialPath(i, n->paths);

        n->paths.insert(i, path);
        f_mins.push_back(path.size() - 1);
        std::cout << "path size for agent " << i << "  =  " << path.size() << "  " << P->getStart(i)->id << "  " << P->getGoal(i)->id << std::endl;
    }
    // std::cout<<"debug??? "<<paths.size()<<"  "<<paths.getMakespan()<<std::endl;
    // n->paths = paths;
    // n->constraints = {};
    // n->makespan = paths.getMakespan();
    // n->soc = paths.getSOC();
    // n->f = paths.countConflict();
    // n->valid = true;
    // n->f_mins = f_mins;
    // n->h_id = 0;
    // n->LB = n->soc; // initial lower bound
    n->constraints.resize(P->getNum(), {});
    n->makespan = n->paths.getMakespan();
    n->soc = n->paths.getSOC();
    LibCBS::getAllConflictsDP(n->paths, n->conflicts);
    n->f = n->conflicts.size();
    // n->f = n->paths.countConflict();
    n->valid = true;
    n->f_mins = f_mins;
    n->h_id = 0;
    n->LB = n->soc; // initial lower bound
    std::cout << "initial makespan =" << n->makespan << "  soc=" << n->soc << "  num of conflicts=" << n->f << std::endl;
}

void ECBSG::setInitialHighLevelNodeFocal(HighLevelNode_p n)
{
    n->paths = Paths();
    n->constraints.resize(P->getNum(), {});

    std::vector<int> f_mins; // vector of costs for respective paths
                             // #if PARALLEL
                             // #pragma omp parallel for
                             // #endif
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < P->getNum(); ++i)
    {
        Path path;
        if (P->getGoal(i)->id == P->getStart(i)->id)
            path = {P->getStart(i)};
        else
            path = getInitialPathFocal(n, i);
        std::cout << "path size for agent " << i << "  =  " << path.size() << "  " << P->getStart(i)->id << "  " << P->getGoal(i)->id << std::endl;
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

    // n->constraints = {};
    n->makespan = n->paths.getMakespan();
    n->soc = n->paths.getSOC();
    LibCBS::getAllConflictsDP(n->paths, n->conflicts);
    n->f = n->conflicts.size();
    // n->f = n->paths.countConflict();
    n->valid = true;
    n->f_mins = f_mins;
    n->h_id = 0;
    n->LB = n->soc;

    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed = end - start;
    // printf("set initial node=%f\n", elapsed.count());
    // exit(0)
    // initial lower bound
    // std::cout<<"makespan ="<<n->makespan<<"  soc="<<n->soc<<"  num of conflicts="<<n->f<<std::endl;
    // std::map<int, std::set<int>> graph;
    // for (auto c : n->conflicts)
    // {
    //     if (graph.find(c->a1) != graph.end())
    //     {
    //         graph[c->a1].insert(c->a2);
    //         graph[c->a2].insert(c->a1);
    //     }
    //     else
    //     {
    //         graph[c->a1] = {c->a2};
    //         graph[c->a2] = {c->a1};
    //     }
    // }
    // std::vector<std::vector<int>> components = find_components(graph);
    // std::cout << "components size=" << components.size() << std::endl;
    // for(auto& con:components){
    //     std::cout<<con.size()<<std::endl;
    // }
    // exit(0);
}

Path ECBSG::getInitialPath(int id, const Paths &paths)
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

void ECBSG::invoke(HighLevelNode_p h_node, int id)
{

    auto res = getFocalPath(h_node, id);
    Path path = std::get<0>(res);
    int f_min = std::get<1>(res); // lower bound

    // failed to find path
    if (path.empty())
    {
        h_node->valid = false;
        return;
    }

    Paths paths = h_node->paths;
    paths.insert(id, path);

    // it is efficient to reuse past data
    h_node->f = h_node->f -
                h_node->paths.countConflict(id, h_node->paths.get(id)) +
                h_node->paths.countConflict(id, paths.get(id));
    h_node->paths = paths;
    h_node->makespan = h_node->paths.getMakespan();
    h_node->soc = h_node->paths.getSOC();
    // update lower bound and f_min
    h_node->LB = h_node->LB - h_node->f_mins[id] + f_min;
    h_node->f_mins[id] = f_min;
}

void ECBSG::invokeDP(HighLevelNode_p h_node, int id)
{

    clearPathTableForAgent(h_node->paths.get(id), id);
    // updatePathTable(h_node->paths, id);
    auto res = getFocalPath(h_node, id);
    Path path = std::get<0>(res);
    int f_min = std::get<1>(res); // lower bound

    // failed to find path
    if (path.empty())
    {
        h_node->valid = false;
        updatePathTableForAgent(h_node->paths.get(id), id);
        return;
    }

    // Paths paths = h_node->paths;

    h_node->paths.insert(id, path);

    LibCBS::Conflicts cfs;
    auto start = std::chrono::high_resolution_clock::now();
    LibCBS::getConflictDP(h_node->parent->paths, h_node->paths.get(id), id, cfs);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    printf("get COnlifct DP @ invoke=%f\n", elapsed.count());
    for (auto &cf : h_node->parent->conflicts)
    {
        if (cf->a1 == id or cf->a2 == id)
            continue;
        cfs.push_back(cf);
    }

    h_node->f = cfs.size();
    h_node->conflicts = cfs;
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
    // clearPathTable(h_node->paths);
    // clearPathTableForAgent(path, id);
    updatePathTableForAgent(h_node->parent->paths.get(id), id);
}

void ECBSG::countConflictsDP(HighLevelNode_p &h_node, int id)
{
    LibCBS::Conflicts cfs;
    LibCBS::getConflictDP(h_node->parent->paths, h_node->paths.get(id), id, cfs);
    for (auto &cf : h_node->parent->conflicts)
    {
        if (cf->a1 == id or cf->a2 == id)
            continue;
        cfs.push_back(cf);
    }
    //

    h_node->f = cfs.size();
    h_node->conflicts = cfs;
    assert(h_node->conflicts.size() < h_node->parent->conflicts.size());
}

std::tuple<Path, int> ECBSG::getFocalPath(HighLevelNode_p h_node, int id)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);

    // pre processing
    // LibCBS::Constraints constraints;
    int max_constraint_time = h_node->max_constraint_time;

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
    // clear used path table
    // clearPathTable(paths);

    return p;
}

Path ECBSG::getInitialPathFocal(HighLevelNode_p h_node, int id)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);

    // pre processing
    LibCBS::Constraints constraints;
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

// return path and f_min
std::tuple<Path, int> ECBSG::getTimedPathByFocalSearch(
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
Path ECBSG::getPathFromFocalNode(FocalNode *_n)
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

void ECBSG::setParams(int argc, char *argv[])
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
            solver_name = ECBSG::SOLVER_NAME + "-" + std::to_string(sub_optimality);
            break;
        default:
            break;
        }
    }
}

void ECBSG::printHelp()
{
    std::cout << ECBSG::SOLVER_NAME << "\n"
              << "  -w --sub-optimality [NUMBER]"
              << "  "
              << "sub-optimality >= 1" << std::endl;
}
