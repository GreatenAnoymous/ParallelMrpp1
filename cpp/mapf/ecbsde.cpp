#include "ecbsde.hpp"
#include <limits>
#include <mutex>
#include <thread>
#include <random>
#define NUM_THREADS 2

const std::string ECBSDE::SOLVER_NAME = "multi-threading ECBS";
const float ECBSDE::DEFAULT_SUB_OPTIMALITY = 2.0;
const int income_threshold = 100;

ECBSDE::ECBSDE(Problem *_P) : Solver(_P)
{

    sub_optimality = DEFAULT_SUB_OPTIMALITY;
    solver_name = ECBSDE::SOLVER_NAME + "-" + std::to_string(sub_optimality);
    initialize_thread_path_table();
}

ECBSDE::ECBSDE(Graph *graph, Config starts, Config goals) : Solver(graph, starts, goals)
{
    sub_optimality = DEFAULT_SUB_OPTIMALITY;
    initialize_thread_path_table();
    solver_name = ECBSDE::SOLVER_NAME + "-" + std::to_string(sub_optimality);
}

void ECBSDE::initialize_thread_path_table()
{
    PATH_TABLES.resize(NUM_THREADS);
    income_buffer.resize(NUM_THREADS);
    for (size_t i = 0; i < NUM_THREADS; i++)
    {
        income_buffer[i] = std::make_shared<buffer<HighLevelNode_p>>();
    }
    LBs.resize(NUM_THREADS);
    for (auto &p : LBs)
    {
        p = std::make_unique<std::atomic<int>>(std::numeric_limits<int>::max()); // init atomic ints to 0
    }
}

int ECBSDE::getCurrentLowBound()
{
    int min_lb = std::numeric_limits<int>::max();

    for (const auto &atomicInt : LBs)
    {
        // Access the underlying int value using load()
        int current = atomicInt->load();

        // Update min if the current value is smaller
        if (current < min_lb)
        {
            min_lb = current;
        }
    }
    return min_lb;
}

void ECBSDE::updatePathTableForThread(const Paths &paths, const int id, const int thread_index)
{
    const int makespan = paths.getMakespan();
    const int num_agents = paths.size();
    const int nodes_size = G->getNodesSize();
    // extend PATH_TABLE
    while ((int)PATH_TABLES[thread_index].size() < makespan + 1)
        PATH_TABLES[thread_index].push_back(std::vector<int>(nodes_size, NIL));
    // update locations
    for (int i = 0; i < num_agents; ++i)
    {
        if (i == id || paths.empty(i))
            continue;
        auto p = paths.get(i);
        for (int t = 0; t <= makespan; ++t)
            PATH_TABLES[thread_index][t][p[t]->id] = i;
    }
}

void ECBSDE::clearPathTableForThread(const Paths &paths, const int thread_index)
{
    const int makespan = paths.getMakespan();
    const int num_agents = paths.size();
    for (int i = 0; i < num_agents; ++i)
    {
        if (paths.empty(i))
            continue;
        auto p = paths.get(i);
        for (int t = 0; t <= makespan; ++t)
        {
            if (t >= PATH_TABLES[thread_index].size())
                continue;

            PATH_TABLES[thread_index][t][p[t]->id] = NIL;
        }
    }
}

void ECBSDE::thread_search(size_t thread_id)
{
    FocalList OPEN(compareOPEN);
    FocalList FOCAL(compareFOCAL);
    std::vector<std::vector<HighLevelNode_p>> outgo_buffer(NUM_THREADS, std::vector<HighLevelNode_p>());
    std::vector<HighLevelNode_p> tmp;

    while (true)
    {

        if (incumbent != nullptr)
            break;
        // check time limit
        // if (overCompTime())
        // {
        //     std::cout << "time out" << std::endl;
        //     break;
        // }
        // /* code */

        if (!income_buffer[thread_id]->isempty())
        {
            // terminates[thread_id] = false;
            if (income_buffer[thread_id]->size() >= income_threshold)
            {
                income_buffer[thread_id]->lock();
                tmp = income_buffer[thread_id]->pull_all_with_lock();
                income_buffer[thread_id]->release_lock();
                uint size = tmp.size();
                for (int i = 0; i < size; ++i)
                {
                    OPEN.push(tmp[i]); // Not sure optimal or not. Vector to Heap.
                }
            
            }
            else if (income_buffer[thread_id]->try_lock())
            {
                tmp = income_buffer[thread_id]->pull_all_with_lock();

                income_buffer[thread_id]->release_lock();

                uint size = tmp.size();
                for (int i = 0; i < size; ++i)
                {
                    OPEN.push(tmp[i]); // Not sure optimal or not.
                }
                // tmp.clear();
            }
        }
        while (!OPEN.empty() && !OPEN.top()->valid)
            OPEN.pop();

        if (OPEN.empty())
        {
            continue;
        }

        LBs[thread_id]->store(OPEN.top()->LB);
        int lb = getCurrentLowBound();
        int lb_old = socLB.load();

        if (lb != lb_old)
        {
            socLB.store(lb);
            float LB_bound = lb * sub_optimality;
            std::vector<HighLevelNode_p> tmp;
            FocalList EMPTY(compareFOCAL);
            FOCAL = EMPTY;
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
        else
        {
            for (int i = 0; i < tmp.size(); ++i)
            {
                // std::cout << socLB.load() * sub_optimality << "  " << tmp[i]->soc << std::endl;
                if (tmp[i]->soc <= socLB.load() * sub_optimality)
                    FOCAL.push(tmp[i]); // Not sure optimal or not.
            }
        }
        tmp.clear();
        // std::cout<<"tmp size="<<tmp.size()<<"  "<<tmp[i]->soc<<"  " <<std::endl;
        if (FOCAL.empty())
            continue;
        // std::cout << "num expansions=" << numExpansions << std::endl;
        auto n = FOCAL.top();
        // std::cout << "Number of Node expansions=" << numExpansions << " number of conflicts=" << n->f << std::endl;

        if (n->conflicts.empty() and n->soc <= socLB.load() * sub_optimality)
        {
            std::mutex mtx;
            std::lock_guard<std::mutex> lock(mtx);
            incumbent = n;
            mtx.unlock();
            solved = true;
            std::cout << "CBS tree depth=" << n->constraints.size() << " num expansions=" << numExpansions.load() << std::endl;
            break;
        }

        LibCBS::Constraints constraints = LibCBS::constraintsFromConflict(LibCBS::chooseConflict(n->conflicts));
        // find the solution

        FOCAL.pop();
        n->valid = false; // closed
        numExpansions.fetch_add(1, std::memory_order_relaxed);
        int result = numExpansions.load(std::memory_order_relaxed);

        // LibCBS::Constraints constraints = LibCBS::getConstraintsFast(n->paths);
        // if (constraints.empty())
        // {
        //     solved = true;
        //     break;
        // }
        size_t ci = 0;
        for (auto c : constraints)
        {
            LibCBS::Constraints new_constraints = n->constraints;
            new_constraints.push_back(c);
            HighLevelNode_p m = std::make_shared<HighLevelNode>(
                n->paths, new_constraints, n->makespan, n->soc, n->f, n->LB,
                n->f_mins, true);
            m->conflicts = n->conflicts;
            invoke(m, c->id, thread_id, ci);
            ci++;

            // printf("invoke time cost=%f\n", elapsed.count());
            if (!m->valid)
                continue;

            income_buffer[m->h_id]->push(m);
            income_buffer[m->h_id]->release_lock();

            // OPEN.push(m);
            // if (m->LB <= socLB * sub_optimality)
            //     FOCAL.push(m);
            // m->h_id = h_node_num;
        }
    }
}

std::tuple<Path, int> ECBSDE::getFocalPath(HighLevelNode_p h_node, int id, int thread_id)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);

    // pre processing
    LibCBS::Constraints constraints;
    int max_constraint_time = 0;
    for (auto c : h_node->constraints)
    {
        if (c->id == id)
        {
            constraints.push_back(c);
            if (c->v == g && c->u == nullptr)
            {
                max_constraint_time = std::max(max_constraint_time, c->t);
            }
        }
    }

    // f-value for online list
    FocalHeuristics f1Value;
    // distance table is shared and  read-only
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
    auto start = std::chrono::system_clock::now();
    updatePathTableForThread(paths, id, thread_id);

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
            if (PATH_TABLES[thread_id][makespan][n->v->id] != Solver::NIL)
                return n->p->f2 + 1;
        }
        else
        {
            // vertex conflict
            if (PATH_TABLES[thread_id][n->g][n->v->id] != Solver::NIL)
            {
                return n->p->f2 + 1;

                // swap conflict
            }
            else if (PATH_TABLES[thread_id][n->g][n->p->v->id] != Solver::NIL &&
                     PATH_TABLES[thread_id][n->g - 1][n->v->id] ==
                         PATH_TABLES[thread_id][n->g][n->p->v->id])
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
    // clear used path table
    clearPathTableForThread(paths, thread_id);

    return p;
}

void ECBSDE::setInitialHighLevelNodeFocal(HighLevelNode_p n)
{
    n->paths = Paths();

    std::vector<int> f_mins; // vector of costs for respective paths
    for (int i = 0; i < P->getNum(); ++i)
    {
        Path path;
        if (P->getGoal(i)->id == P->getStart(i)->id)
            path = {P->getStart(i)};
        else
            path = getInitialPathFocal(n, i);
        // std::cout << "path size for agent " << i << "  =  " << path.size() << "  " << P->getStart(i)->id << "  " << P->getGoal(i)->id << std::endl;
        // updatePathTableWithoutClear(i, path, n->paths);
        updatePathTableForAgent(path, i);
        // n->paths.insert(i, path);
        n->paths.append(path);

        f_mins.push_back(path.size() - 1);
    }
    // clearPathTable(n->paths);
    PATH_TABLE.clear();
    // std::cout<<"debug??? "<<paths.size()<<"  "<<paths.getMakespan()<<std::endl;

    n->constraints = {};
    n->makespan = n->paths.getMakespan();
    n->soc = n->paths.getSOC();
    LibCBS::getAllConflicts(n->paths, n->conflicts);
    n->f = n->conflicts.size();
    std::cout << "initiali conflicts number=" << n->f << std::endl;
    n->valid = true;
    n->f_mins = f_mins;
    n->h_id = 0;
    n->LB = n->soc; // initial lower bound
}

Path ECBSDE::getInitialPathFocal(HighLevelNode_p h_node, int id)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);

    // pre processing
    LibCBS::Constraints constraints;
    int max_constraint_time = 0;
    for (auto c : h_node->constraints)
    {
        if (c->id == id)
        {
            constraints.push_back(c);
            if (c->v == g && c->u == nullptr)
            {
                max_constraint_time = std::max(max_constraint_time, c->t);
            }
        }
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
    // clear used path table

    return std::get<0>(p);
}

ECBSDE::CompareHighLevelNode ECBSDE::getMainObjective()
{
    CompareHighLevelNode compare = [&](HighLevelNode_p a, HighLevelNode_p b)
    {
        if (a->LB != b->LB)
            return a->LB > b->LB;
        return false;
    };
    return compare;
}

ECBSDE::CompareHighLevelNode ECBSDE::getFocalObjective()
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

void ECBSDE::run()
{
    HighLevelNode_p n = std::make_shared<HighLevelNode>();
    setInitialHighLevelNodeFocal(n);
    income_buffer[0]->push(n);
    socLB = n->LB;
    // pthread_barrier_init(&barrier, NULL, tnum);

    // Create an array to store thread IDs
    std::vector<std::thread> threads;

    for (size_t i = 0; i < NUM_THREADS; i++)
    {
        threads.emplace_back(&ECBSDE::thread_search, this, i);
    }

    for (size_t i = 0; i < NUM_THREADS; i++)
    {
        threads[i].join();
    }

    solution = pathsToPlan(incumbent->paths);
}

size_t ECBSDE::getRandomHashCode(int thread_id)
{
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_int_distribution<size_t> dist(0, NUM_THREADS - 1);
    // return dist(gen);
    if (thread_id + 1 >= NUM_THREADS)
        return 0;
    return thread_id + 1;
}

std::tuple<Path, int> ECBSDE::getTimedPathByFocalSearch(
    Node *const s, Node *const g,
    float w, // sub-optimality
    FocalHeuristics &f1Value, FocalHeuristics &f2Value,
    CompareFocalNode &compareOPEN, CompareFocalNode &compareFOCAL,
    CheckFocalFin &checkFocalFin, CheckInvalidFocalNode &checkInvalidFocalNode)
{
    auto getNodeName = [&](FocalNode *n)
    {
        return n->v->id + G->getNodesSize() * n->g;
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

void ECBSDE::invoke(HighLevelNode_p h_node, int id, int thread_id, int ci)
{
    auto res = getFocalPath(h_node, id, thread_id);
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
    LibCBS::Conflicts cfs;
    LibCBS::getConflict(h_node->paths, paths.get(id), id, cfs);

    for (auto &cf : h_node->conflicts)
    {
        // std::cout << cf->a1 << "  " << cf->a2 << "  " << id << std::endl;
        if (cf->a1 == id or cf->a2 == id)
            continue;
        cfs.push_back(cf);
    }
    // std::cout << "cfs conflicts size=" << cfs.size() << std::endl;
    h_node->f = cfs.size();
    h_node->conflicts = cfs;
    // it is efficient to reuse past data
    // h_node->f = h_node->f -
    //             h_node->paths.countConflict(id, h_node->paths.get(id)) +
    //             h_node->paths.countConflict(id, paths.get(id));
    h_node->paths = paths;
    h_node->makespan = h_node->paths.getMakespan();
    h_node->soc = h_node->paths.getSOC();
    // update lower bound and f_min
    h_node->LB = h_node->LB - h_node->f_mins[id] + f_min;
    h_node->f_mins[id] = f_min;
    if (ci == 0)
        h_node->h_id = thread_id;
    else
        h_node->h_id = getRandomHashCode(thread_id);
}

Path ECBSDE::getPathFromFocalNode(FocalNode *_n)
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