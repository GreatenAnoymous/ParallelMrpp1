#include "lib_cbs.hpp"
#include <set>
#include <sparsehash/sparse_hash_map>
#include <random>
#include <thread>
// cache
using google::sparse_hash_map;
std::unordered_map<std::string, LibCBS::MDD_p> LibCBS::MDD::PURE_MDD_TABLE;

void LibCBS::Constraint::println()
{
    if (u == nullptr)
    {
        std::cout << "vertex constraint: <id=" << id << ", t=" << t
                  << ", v=" << v->id << ">" << std::endl;
    }
    else
    {
        std::cout << "edge constraint: <id=" << id << ", t=" << t << ", v:" << u->id
                  << "->" << v->id << ">" << std::endl;
    }
}

LibCBS::Constraints LibCBS::getConstraintsFast(const Paths &paths)
{
    Constraints constraints = {};
    int num_agents = paths.size();
    int makespan = paths.getMakespan();

    // Note: shared variables are declared outside the parallel region
    Constraints result_constraints;
    bool found_constraints = false;

#if PARALLEL
#pragma omp parallel for shared(result_constraints, found_constraints)
#endif
    for (int t = 1; t <= makespan; ++t)
    {
        // sparse_hash_map<int, int, std::hash<int>> vagents;
        // sparse_hash_map<int, int, std::hash<int>> eagents;
        std::unordered_map<size_t, size_t> vagents;
        std::unordered_map<long, size_t> eagents;

#if PARALLEL
#pragma omp parallel for
#endif
        for (int i = 0; i < num_agents; ++i)
        {
            // vertex conflicts
            int vt = paths.get(i, t)->id;
            if (vagents.find(vt) != vagents.end())
            {
                size_t j = vagents[vt];
                Constraint_p c_i = std::make_shared<Constraint>(i, t, paths.get(i, t), nullptr);
                Constraint_p c_j = std::make_shared<Constraint>(j, t, paths.get(j, t), nullptr);
#if PARALLEL
#pragma omp critical
#endif
                {
                    result_constraints = {c_i, c_j};
                    found_constraints = true;
                }

#if PARALLEL
#pragma omp cancel for
#endif
            }
            else
            {
                // vagents[vt] = i;
                vagents.insert({vt, i});
            }

            // edge conflicts
            int vtt = paths.get(i, t - 1)->id;
            if (vtt == vt)
                continue;

            long edge_index = (vt > vtt) ? (static_cast<long>(vtt) * 100000 + vt) : (static_cast<long>(vt) * 100000 + vtt);
            if (eagents.find(edge_index) != eagents.end())
            {
                int j = eagents[edge_index];
                Constraint_p c_i = std::make_shared<Constraint>(i, t, paths.get(i, t), paths.get(i, t - 1));
                Constraint_p c_j = std::make_shared<Constraint>(j, t, paths.get(j, t), paths.get(j, t - 1));
#if PARALLEL
#pragma omp critical
#endif
                {

                    result_constraints = {c_i, c_j};
                    found_constraints = true;
                }
#if PARALLEL
#pragma omp cancel for
#endif
            }
            else
            {
                // eagents[edge_index] = i;
                eagents.insert({edge_index, i});
            }
        }
    }

    if (found_constraints)
    {
        return result_constraints;
    }
    else
    {
        return {};
    }
}

LibCBS::Constraints LibCBS::constraintsFromConflict(LibCBS::Conflict_p cf)
{
    Constraint_p c_i =
        std::make_shared<Constraint>(cf->a1, cf->t, cf->v, cf->u);
    Constraint_p c_j;
    if (cf->u == nullptr)
        c_j = std::make_shared<Constraint>(cf->a2, cf->t, cf->v, cf->u);
    else
        c_j = std::make_shared<Constraint>(cf->a2, cf->t, cf->u, cf->v);
    return {c_i, c_j};
}

// not found -> return {}
LibCBS::Constraints LibCBS::getFirstConstraints(const Paths &paths)
{
    Constraints constraints = {};
    int num_agents = paths.size();
    int makespan = paths.getMakespan();
    std::cout << num_agents << "    " << makespan << std::endl;
    for (int t = 1; t <= makespan; ++t)
    {
        for (int i = 0; i < num_agents; ++i)
        {
            for (int j = i + 1; j < num_agents; ++j)
            {
                // vertex conflict
                if (paths.get(i, t) == paths.get(j, t))
                {
                    Constraint_p c_i =
                        std::make_shared<Constraint>(i, t, paths.get(i, t), nullptr);
                    Constraint_p c_j =
                        std::make_shared<Constraint>(j, t, paths.get(j, t), nullptr);
                    Constraints constraints = {c_i, c_j};
                    return constraints;
                }
                // swap conflict
                if (paths.get(i, t) == paths.get(j, t - 1) &&
                    paths.get(j, t) == paths.get(i, t - 1))
                {
                    Constraint_p c_i = std::make_shared<Constraint>(i, t, paths.get(i, t),
                                                                    paths.get(i, t - 1));
                    Constraint_p c_j = std::make_shared<Constraint>(j, t, paths.get(j, t),
                                                                    paths.get(j, t - 1));
                    Constraints constraints = {c_i, c_j};
                    return constraints;
                }
            }
        }
    }
    return {};
}

// used for ICBS
// detect prioritized constraints for paths[i][t] and paths[j][t]
void LibCBS::getPrioritizedConflict(const int t, const int i, const int j,
                                    const Paths &paths, const MDDs &mdds,
                                    Constraints &cardinal_conflicts,
                                    Constraints &semi_cardinal_constraints,
                                    Constraints &non_cardinal_constraints)
{
    int c_i = mdds[i]->c;
    int c_j = mdds[j]->c;
    int w_i = (t <= c_i) ? mdds[i]->getWidth(t) : 0;
    int w_j = (t <= c_j) ? mdds[j]->getWidth(t) : 0;
    // vertex conflict
    if (paths.get(i, t) == paths.get(j, t))
    {
        Constraint_p constraint_i =
            std::make_shared<Constraint>(i, t, paths.get(i, t), nullptr);
        Constraint_p constraint_j =
            std::make_shared<Constraint>(j, t, paths.get(j, t), nullptr);
        // cardinal conflicts
        if ((t <= c_i && w_i == 1 && t <= c_j && w_j == 1) ||
            (t > c_i && w_j == 1) || (t > c_j && w_i == 1))
        {
            cardinal_conflicts.push_back(constraint_i);
            cardinal_conflicts.push_back(constraint_j);
            return;
        }
        // semi-cardinal conflicts
        if (semi_cardinal_constraints.empty() &&
            (t > c_i || t > c_j || w_i == 1 || w_j == 1))
        {
            semi_cardinal_constraints.push_back(constraint_i);
            semi_cardinal_constraints.push_back(constraint_j);
        }
        else if (non_cardinal_constraints.empty())
        {
            non_cardinal_constraints.push_back(constraint_i);
            non_cardinal_constraints.push_back(constraint_j);
        }
    }
    // swap conflict
    if (paths.get(i, t) == paths.get(j, t - 1) &&
        paths.get(j, t) == paths.get(i, t - 1))
    {
        Constraint_p constraint_i = std::make_shared<Constraint>(
            i, t, paths.get(i, t), paths.get(i, t - 1));
        Constraint_p constraint_j = std::make_shared<Constraint>(
            j, t, paths.get(j, t), paths.get(j, t - 1));
        // cardinal conflicts
        if ((t <= c_i && w_i == 1 && mdds[i]->body[t][0]->prev.size() == 1) &&
            (t <= c_j && w_j == 1 && mdds[j]->body[t][0]->prev.size() == 1))
        {
            cardinal_conflicts.push_back(constraint_i);
            cardinal_conflicts.push_back(constraint_j);
            return;
        }
        // semi-cardinal conflicts
        if (semi_cardinal_constraints.empty() &&
            (t > c_i || t > c_j ||
             (w_i == 1 && mdds[i]->body[t][0]->prev.size() == 1) ||
             (w_j == 1 && mdds[j]->body[t][0]->prev.size() == 1)))
        {
            semi_cardinal_constraints.push_back(constraint_i);
            semi_cardinal_constraints.push_back(constraint_j);
        }
        else if (non_cardinal_constraints.empty())
        {
            non_cardinal_constraints.push_back(constraint_i);
            non_cardinal_constraints.push_back(constraint_j);
        }
    }
}

// used for ICBS
// detect prioritized constraints
LibCBS::Constraints LibCBS::getPrioritizedConflict(const Paths &paths,
                                                   const MDDs &mdds)
{
    Constraints cardinal_constraints = {};
    Constraints semi_cardinal_constraints = {};
    Constraints non_cardinal_constraints = {};
    int num_agents = paths.size();
    for (int t = 1; t <= paths.getMakespan(); ++t)
    {
        for (int i = 0; i < num_agents; ++i)
        {
            for (int j = i + 1; j < num_agents; ++j)
            {
                getPrioritizedConflict(t, i, j, paths, mdds, cardinal_constraints,
                                       semi_cardinal_constraints,
                                       non_cardinal_constraints);
                if (!cardinal_constraints.empty())
                    return cardinal_constraints;
            }
        }
    }
    if (!semi_cardinal_constraints.empty())
    {
        return semi_cardinal_constraints;
    }
    else if (!non_cardinal_constraints.empty())
    {
        return non_cardinal_constraints;
    }
    return {};
}

// used for ICBS as a refine-solver
// detect prioritized conflicts only for a part of agents
LibCBS::Constraints LibCBS::getPrioritizedConflict(
    const Paths &paths, const MDDs &mdds, const std::vector<int> &sample)
{
    Constraints cardinal_constraints = {};
    Constraints semi_cardinal_constraints = {};
    Constraints non_cardinal_constraints = {};
    const int sample_size = sample.size();
    for (int t = 1; t <= paths.getMakespan(); ++t)
    {
        for (int i = 0; i < sample_size; ++i)
        {
            for (int j = i + 1; j < sample_size; ++j)
            {
                getPrioritizedConflict(t, sample[i], sample[j], paths, mdds,
                                       cardinal_constraints, semi_cardinal_constraints,
                                       non_cardinal_constraints);
                if (!cardinal_constraints.empty())
                    return cardinal_constraints;
            }
        }
    }
    if (!semi_cardinal_constraints.empty())
    {
        return semi_cardinal_constraints;
    }
    else if (!non_cardinal_constraints.empty())
    {
        return non_cardinal_constraints;
    }
    return {};
}

// used in ICBS as a refine-solver
LibCBS::Constraints LibCBS::getConstraintsByFixedPaths(
    const Plan &plan, const std::vector<int> &fixed_agents)
{
    Constraints constraints;
    int makespan = plan.getMakespan();
    for (auto i : fixed_agents)
    {
        for (int t = 1; t <= makespan; ++t)
        {
            Constraint_p c_vertex =
                std::make_shared<Constraint>(-1, t, plan.get(t, i), nullptr);
            constraints.push_back(c_vertex);
            // notice! be careful to set swap constraints
            Constraint_p c_swap = std::make_shared<Constraint>(
                -1, t, plan.get(t - 1, i), plan.get(t, i));
            constraints.push_back(c_swap);
        }
        // constraints at goal
        Node *g = plan.get(makespan, i);
        Constraint_p c_last = std::make_shared<Constraint>(-1, makespan, g, true);
        constraints.push_back(c_last);
    }
    return constraints;
}

bool LibCBS::MDDNode::operator==(const MDDNode &other) const
{
    return t == other.t && v == other.v;
}

LibCBS::MDD::MDD(int _c, int _i, Solver *_solver, Constraints constraints, int time_limit)
    : c(_c),
      i(_i),
      s(_solver->getP()->getStart(i)),
      g(_solver->getP()->getGoal(i)),
      solver(_solver)
{
    // for timeout
    auto t_s = Time::now();

    // check possibility
    valid = solver->pathDist(i) <= c;

    // build MDD without constraints
    build(time_limit);

    // check time limit
    if (time_limit > 0 && (int)getElapsedTime(t_s) > time_limit)
    {
        valid = false;
        return;
    }

    // update MDD with constraints
    update(constraints);
}

LibCBS::MDD::~MDD()
{
    for (MDDNode *node : GC)
        delete node;
}

LibCBS::MDD::MDD(const MDD &other)
    : c(other.c),
      i(other.i),
      s(other.s),
      g(other.g),
      valid(other.valid),
      solver(other.solver)
{
    copy(other);
}

LibCBS::MDDNode *LibCBS::MDD::createNewNode(int t, Node *v)
{
    MDDNode *new_node = new MDDNode(t, v);
    GC.push_back(new_node); // garbage collection
    return new_node;
}

void LibCBS::MDD::copy(const MDD &other)
{
    if (!valid)
        return;

    // initial node
    MDDNode *new_node = createNewNode(0, s);
    body.push_back({new_node});

    // generate body
    for (auto nodes : other.body)
    {
        MDDNodes new_nodes;
        if (!nodes.empty() && nodes[0]->t == 0)
            continue; // starts
        MDDNodes new_prev_nodes = body[body.size() - 1];
        for (auto node : nodes)
        {
            new_node = createNewNode(node->t, node->v);
            new_nodes.push_back(new_node);
            // create link
            for (auto prev_node : node->prev)
            {
                auto itr = std::find_if(new_prev_nodes.begin(), new_prev_nodes.end(),
                                        [prev_node](MDDNode *_node)
                                        {
                                            return _node->t == prev_node->t &&
                                                   _node->v == prev_node->v;
                                        });
                if (itr != new_prev_nodes.end())
                {
                    auto new_prev_node = *itr;
                    new_prev_node->next.push_back(new_node);
                    new_node->prev.push_back(new_prev_node);
                }
            }
        }
        body.push_back(new_nodes);
    }
}

void LibCBS::MDD::build(int time_limit)
{
    auto t_s = Time::now();
    // impossible
    if (!valid)
        return;
    // check registered
    auto itr = PURE_MDD_TABLE.find(getPureMDDName());
    if (itr != PURE_MDD_TABLE.end())
    {
        valid = itr->second->valid;
        copy(*(itr->second));
        return;
    }

    // add start node
    body.push_back({createNewNode(0, s)});
    // build
    for (int t = 0; t < c; ++t)
    {
        MDDNodes nodes_at_t = body[t];
        MDDNodes nodes_at_t_next;
        for (auto node : nodes_at_t)
        {
            // check time limit
            if (time_limit > 0 && getElapsedTime(t_s) > time_limit)
            {
                valid = false;
                return;
            }

            Nodes cands = node->v->neighbor;
            cands.push_back(node->v);
            for (auto v : cands)
            {
                // valid
                if (solver->pathDist(i, v) + t + 1 <= c)
                {
                    // already exists?
                    MDDNode *next_node = nullptr;
                    for (auto _node : nodes_at_t_next)
                    {
                        if (_node->v == v)
                        {
                            next_node = _node;
                            break;
                        }
                    }
                    if (next_node == nullptr)
                    {
                        // create a new MDD node
                        next_node = createNewNode(t + 1, v);
                        nodes_at_t_next.push_back(next_node);
                    }
                    // create link
                    node->next.push_back(next_node);
                    next_node->prev.push_back(node);
                }
            }
        }
        body.push_back(nodes_at_t_next);
    }

    // register a new MDD without conflicts
    PURE_MDD_TABLE[getPureMDDName()] = std::make_shared<MDD>(*this);
}

void LibCBS::MDD::update(const Constraints &_constraints)
{
    if (!valid || _constraints.empty())
        return;
    // format constraints
    Constraints constraints;
    for (auto constraint : _constraints)
    {
        if (constraint->id != i && constraint->id != -1)
            continue;
        // vertex conflict at the goal, must increase cost
        if (constraint->t >= c && constraint->u == nullptr)
        {
            if (constraint->v == g)
            {
                valid = false;
                return;
            }
            else
            {
                continue;
            }
        }
        // swap conflict, never occur
        if (constraint->t > c && constraint->u != nullptr)
        {
            continue;
        }
        constraints.push_back(constraint);
    }

    // delete nodes
    for (auto constraint : constraints)
    {
        if (constraint->stay)
        { // check goal
            for (int t = constraint->t; t <= c; ++t)
            {
                auto itr_v = std::find_if(
                    body[t].begin(), body[t].end(),
                    [constraint](MDDNode *node)
                    { return node->v == constraint->v; });
                if (itr_v == body[t].end())
                    continue;
                MDDNode *node_v = *itr_v;
                deleteForward(node_v);
                deleteBackword(node_v);
            }
            continue;
        }

        auto itr_v = std::find_if(
            body[constraint->t].begin(), body[constraint->t].end(),
            [constraint](MDDNode *node)
            { return node->v == constraint->v; });
        if (itr_v == body[constraint->t].end())
            continue;
        MDDNode *node_v = *itr_v;
        if (constraint->u == nullptr)
        { // vertex constraints, v
            deleteForward(node_v);
            deleteBackword(node_v);
        }
        else
        { // swap conflict, u->v
            auto itr_vu = std::find_if(
                node_v->prev.begin(), node_v->prev.end(),
                [constraint](MDDNode *node)
                { return node->v == constraint->u; });
            MDDNode *node_u = *itr_vu;
            if (itr_vu != node_v->prev.end())
            {
                auto itr_uv = std::find_if(
                    node_u->next.begin(), node_u->next.end(),
                    [node_v](MDDNode *node)
                    { return node->v == node_v->v; });
                node_v->prev.erase(itr_vu);
                node_u->next.erase(itr_uv);
                if (node_v->prev.empty())
                    deleteForward(node_v);
                if (node_u->next.empty())
                    deleteBackword(node_u);
            }
        }
    }

    // update validity
    if (body[0].empty() || body[c].empty())
        valid = false;
}

bool LibCBS::MDD::forceUpdate(const Constraints &_constraints)
{
    bool updated = false;
    if (!valid)
        return false;

    // format constraints
    Constraints constraints;
    for (auto constraint : _constraints)
    {
        if (constraint->id != i && constraint->id != -1)
            continue;
        // vertex conflict at the goal, must increase cost
        if (constraint->t >= c && constraint->u == nullptr && constraint->v == g)
        {
            valid = false;
        }
        // unused constraints
        if (constraint->t > c)
            continue;
        constraints.push_back(constraint);
    }

    // delete nodes
    for (auto constraint : constraints)
    {
        if (constraint->stay)
        { // check goal
            for (int t = constraint->t; t <= c; ++t)
            {
                auto itr_v = std::find_if(
                    body[t].begin(), body[t].end(),
                    [constraint](MDDNode *node)
                    { return node->v == constraint->v; });
                if (itr_v == body[t].end())
                    continue;
                MDDNode *node_v = *itr_v;
                deleteForward(node_v);
                deleteBackword(node_v);
                updated = true;
            }
            continue;
        }

        auto itr_v = std::find_if(
            body[constraint->t].begin(), body[constraint->t].end(),
            [constraint](MDDNode *node)
            { return node->v == constraint->v; });
        if (itr_v == body[constraint->t].end())
            continue;

        MDDNode *node_v = *itr_v;
        if (constraint->u == nullptr)
        { // vertex constraints, v
            deleteForward(node_v);
            deleteBackword(node_v);
            updated = true;
        }
        else
        { // swap conflict, u->v
            auto itr_vu = std::find_if(
                node_v->prev.begin(), node_v->prev.end(),
                [constraint](MDDNode *node)
                { return node->v == constraint->u; });
            MDDNode *node_u = *itr_vu;
            if (itr_vu != node_v->prev.end())
            {
                auto itr_uv = std::find_if(
                    node_u->next.begin(), node_u->next.end(),
                    [node_v](MDDNode *node)
                    { return node->v == node_v->v; });
                updated = true;
                node_v->prev.erase(itr_vu);
                node_u->next.erase(itr_uv);
                if (node_v->prev.empty())
                    deleteForward(node_v);
                if (node_u->next.empty())
                    deleteBackword(node_u);
            }
        }
    }

    // update validity
    if (body[0].empty() || body[c].empty())
        valid = false;

    return updated;
}

// delete unreachable nodes recursively
void LibCBS::MDD::deleteForward(MDDNode *node)
{
    for (auto next_node : node->next)
    {
        auto itr =
            std::find_if(next_node->prev.begin(), next_node->prev.end(),
                         [node](MDDNode *_node)
                         { return _node->v == node->v; });
        next_node->prev.erase(itr);
        if (next_node->prev.empty())
            deleteForward(next_node);
    }
    auto itr =
        std::find_if(body[node->t].begin(), body[node->t].end(),
                     [node](MDDNode *_node)
                     { return _node->v == node->v; });
    if (itr != body[node->t].end())
        body[node->t].erase(itr);
}

// delete unreachable nodes recursively
void LibCBS::MDD::deleteBackword(MDDNode *node)
{
    for (auto prev_node : node->prev)
    {
        auto itr =
            std::find_if(prev_node->next.begin(), prev_node->next.end(),
                         [node](MDDNode *_node)
                         { return _node->v == node->v; });
        prev_node->next.erase(itr);
        if (prev_node->next.empty())
            deleteBackword(prev_node);
    }
    auto itr =
        std::find_if(body[node->t].begin(), body[node->t].end(),
                     [node](MDDNode *_node)
                     { return _node->v == node->v; });
    if (itr != body[node->t].end())
        body[node->t].erase(itr);
}

std::string LibCBS::MDD::getPureMDDName()
{
    return std::to_string(s->id) + "-" + std::to_string(g->id) + "-" +
           std::to_string(c) + "-" + std::to_string(i) + "-";
}

// make path using MDD
// if MDD is valid, the search must success
// invalid MDD -> return {}
Path LibCBS::MDD::getPath(std::mt19937 *const MT) const
{
    if (!valid)
        return {};
    if (body[0].empty() || body[c].empty())
        halt("invalid MDD");

    // forward search
    MDDNode *node = body[0][0];
    MDDNode *goal_node = body[c][0];
    Path path;
    while (node != goal_node)
    {
        path.push_back(node->v);
        if (node->next.empty())
            halt("invalid MDD");
        if (MT != nullptr)
        {
            node = randomChoose(node->next, MT); // randomize
        }
        else
        {
            node = node->next[0];
        }
    }
    path.push_back(goal_node->v);
    return path;
}

Path LibCBS::MDD::getPath(Constraint_p const constraint, std::mt19937 *const MT) const
{
    if (!valid)
        return {};
    if (constraint == nullptr)
        return getPath();
    // create a new MDD by copying itself
    MDD mdd = *this;
    // incorporate new constraint
    mdd.update({constraint});
    return mdd.getPath();
}

Path LibCBS::MDD::getPath(const Constraints &_constraints, std::mt19937 *const MT) const
{
    if (!valid)
        return {};
    // create a new MDD by copying itself
    MDD mdd = *this;
    // incorporate new constraint
    mdd.update(_constraints);
    return mdd.getPath();
}

int LibCBS::MDD::getWidth(int t) const
{
    if (t < 0 || c < t)
        halt("invalid index");
    return body[t].size();
}

void LibCBS::MDD::println() const
{
    std::cout << "MDD_" << i << "^" << c << ", valid=" << valid << std::endl;
    if (!valid)
        return;
    for (int t = 0; t <= c; ++t)
    {
        std::cout << "t=" << t << std::endl;
        for (auto node : body[t])
        {
            std::cout << "- v=(" << node->v->pos.x << ", " << node->v->pos.y << "), "
                      << "t=" << node->t << ", next: ";
            for (auto next_node : node->next)
            {
                std::cout << "(" << next_node->v->pos.x << ", " << next_node->v->pos.y
                          << "), ";
            }
            std::cout << ", prev: ";
            for (auto prev_node : node->prev)
            {
                std::cout << "(" << prev_node->v->pos.x << ", " << prev_node->v->pos.y
                          << "), ";
            }
            std::cout << std::endl;
        }
    }
}

void LibCBS::MDD::halt(const std::string &msg) const
{
    std::cout << "error@MDD: " << msg << std::endl;
    this->~MDD();
    std::exit(1);
}

void LibCBS::getAllConflicts(const Paths &paths, Conflicts &conflicts)
{
    conflicts.clear();
    int num_agents = paths.size();
    int makespan = paths.getMakespan();
#if PARALLEL
#pragma omp parallel for collapse(2)
#endif
    for (int t = 1; t <= makespan; ++t)
    {
        for (int i = 0; i < num_agents; ++i)
        {
            for (int j = i + 1; j < num_agents; ++j)
            {
                // vertex conflict
                if (paths.get(i, t) == paths.get(j, t))
                {
#if PARALLEL
#pragma omp critical
#endif
                    {
                        Conflict_p cij = std::make_shared<Conflict>(i, j, t, paths.get(i, t));
                        conflicts.push_back(cij);
                    }
                }
                // swap conflict
                if (paths.get(i, t) == paths.get(j, t - 1) &&
                    paths.get(j, t) == paths.get(i, t - 1))
                {
#if PARALLEL
#pragma omp critical
#endif
                    {
                        Conflict_p cij = std::make_shared<Conflict>(i, j, t, paths.get(i, t), paths.get(j, t));
                        conflicts.push_back(cij);
                    }
                }
            }
        }
    }
}

void LibCBS::getAllConflictsParallel(const Paths &paths, Conflicts &conflicts, int numThreads)
{
    conflicts.clear();
    int num_agents = paths.size();
    int makespan = paths.getMakespan();

 
    std::vector<Conflicts> m_conflicts(numThreads, Conflicts());
    auto worker = [&](int t0, int t1, int thread_id)
    {
        for (int t = t0; t <= t1; ++t)
        {
            std::unordered_map<size_t, std::vector<int>> vagents;
            std::unordered_map<long, std::vector<int>> eagents;
            for (int i = 0; i < num_agents; ++i)
            {
                int vt = paths.get(i, t)->id;
                if (vagents.find(vt) != vagents.end())
                {
                    for (auto j : vagents[vt])
                    {
                        if (i <= j)
                        {
                            Conflict_p cij = std::make_shared<Conflict>(i, j, t, paths.get(i, t));
                            m_conflicts[thread_id].push_back(cij);
                        }
                        else
                        {
                            Conflict_p cij = std::make_shared<Conflict>(j, i, t, paths.get(i, t));
                            m_conflicts[thread_id].push_back(cij);
                        }
                    }
                    vagents[vt].push_back(i);
                }
                else
                {
                    vagents.insert({vt, {i}});
                }
                int vtt = paths.get(i, t - 1)->id;
                // if (vtt == vt)
                //     continue;
                long edge_index = (vt > vtt) ? (static_cast<long>(vtt) * 100000 + vt) : (static_cast<long>(vt) * 100000 + vtt);
                if (eagents.find(edge_index) != eagents.end())
                {
                    for (auto j : eagents[edge_index])
                    {
                        if (i <= j)
                        {
                            Conflict_p cij = std::make_shared<Conflict>(i, j, t, paths.get(i, t), paths.get(i, t - 1));
                            m_conflicts[thread_id].push_back(cij);
                        }
                        else
                        {
                            Conflict_p cij = std::make_shared<Conflict>(j, i, t, paths.get(j, t), paths.get(j, t - 1));
                            m_conflicts[thread_id].push_back(cij);
                        }
                    }
                    eagents[edge_index].push_back(i);
                }
                else
                {
                    eagents.insert({edge_index, {i}});
                }
            }
        }
    };

    std::vector<std::thread> threads;
    for (int i = 0; i < numThreads; i++)
    {
        int t0 = 1 + i * makespan / numThreads;
        int t1 = t0 + makespan / numThreads;
        if (i == numThreads - 1)
            t1 = makespan;
        threads.emplace_back(worker, t0, t1, i);
    }

    for (auto &thread : threads)
    {
        thread.join();
    }

    for (auto &mcfs : m_conflicts)
    {
        for (auto &cf : mcfs)
        {
            conflicts.emplace_back(cf);
        }
    }
}

void LibCBS::getAllConflictsDP(const Paths &paths, Conflicts &conflicts)
{
    conflicts.clear();
    int num_agents = paths.size();
    int makespan = paths.getMakespan();

    // #pragma omp for
    for (int t = 1; t <= makespan; ++t)
    {
        std::unordered_map<size_t, std::vector<int>> vagents;
        std::unordered_map<long, std::vector<int>> eagents;
        for (int i = 0; i < num_agents; ++i)
        {
            int vt = paths.get(i, t)->id;
            if (vagents.find(vt) != vagents.end())
            {
                for (auto j : vagents[vt])
                {
                    if (i <= j)
                    {
                        Conflict_p cij = std::make_shared<Conflict>(i, j, t, paths.get(i, t));
                        conflicts.push_back(cij);
                    }
                    else
                    {
                        Conflict_p cij = std::make_shared<Conflict>(j, i, t, paths.get(i, t));
                        conflicts.push_back(cij);
                    }
                }
                vagents[vt].push_back(i);
            }
            else
            {
                vagents.insert({vt, {i}});
            }
            int vtt = paths.get(i, t - 1)->id;
            // if (vtt == vt)
            //     continue;
            long edge_index = (vt > vtt) ? (static_cast<long>(vtt) * 100000 + vt) : (static_cast<long>(vt) * 100000 + vtt);
            if (eagents.find(edge_index) != eagents.end())
            {
                for (auto j : eagents[edge_index])
                {
                    if (i <= j)
                    {
                        Conflict_p cij = std::make_shared<Conflict>(i, j, t, paths.get(i, t), paths.get(i, t - 1));
                        conflicts.push_back(cij);
                    }
                    else
                    {
                        Conflict_p cij = std::make_shared<Conflict>(j, i, t, paths.get(j, t), paths.get(j, t - 1));
                        conflicts.push_back(cij);
                    }
                }
                eagents[edge_index].push_back(i);
            }
            else
            {
                eagents.insert({edge_index, {i}});
            }
        }
    }
}

void LibCBS::getConflict(const Paths &paths, const Path &path, int id, Conflicts &conflicts)
{
    conflicts.clear();
    int cnt = 0;
    int makespan = paths.getMakespan();
    int num_agents = paths.size();
    const int path_size = path.size();
#if PARALLEL
#pragma omp parallel for
#endif
    for (int i = 0; i < num_agents; ++i)
    {
        if (i == id)
            continue;
        for (int t = 1; t < path_size; ++t)
        {
            if (t > makespan)
            {
                if (path[t] == paths.get(i, makespan))
                {
                    // ++cnt;
#if PARALLEL
#pragma omp critical
#endif
                    {
                        Conflict_p conflict;
                        if (id <= i)
                            conflict = std::make_shared<Conflict>(id, i, t, path[t]);
                        else
                            conflict = std::make_shared<Conflict>(i, id, t, path[t]);
                        conflicts.push_back(conflict);
                    }

                    break;
                }
                continue;
            }
            // vertex conflict
            if (paths.get(i, t) == path[t])
            {
#if PARALLEL
#pragma omp critical
#endif
                {
                    Conflict_p conflict;
                    if (id <= i)
                        conflict = std::make_shared<Conflict>(id, i, t, path[t]);
                    else
                        conflict = std::make_shared<Conflict>(i, id, t, path[t]);
                    conflicts.push_back(conflict);
                }

                continue;
            }
            // swap conflict
            if (paths.get(i, t) == path[t - 1] && paths.get(i, t - 1) == path[t])
            {
#if PARALLEL
#pragma omp critical
#endif
                {
                    Conflict_p conflict;
                    if (id <= i)
                        conflict = std::make_shared<Conflict>(id, i, t, path[t], path[t - 1]);
                    else
                        conflict = std::make_shared<Conflict>(i, id, t, path[t - 1], path[t]);
                    conflicts.push_back(conflict);
                }
            }
        }
    }
}

void LibCBS::getConflictDP(const Paths &paths, const Path &path, int id, Conflicts &conflicts)
{
    conflicts.clear();
    int cnt = 0;
    int makespan = paths.getMakespan();
    int num_agents = paths.size();
    const int path_size = path.size();
#if PARALLEL
#pragma omp parallel for
#endif
    for (int i = 0; i < num_agents; ++i)
    {
        if (i == id)
            continue;
        for (int t = 1; t < path_size;)
        {
            if (t > makespan)
            {
                if (path[t] == paths.get(i, makespan))
                {
                    // ++cnt;
#if PARALLEL
#pragma omp critical
#endif
                    {
                        Conflict_p conflict = std::make_shared<Conflict>(id, i, t, path[t]);
                        conflicts.push_back(conflict);
                    }

                    break;
                }
                int dt = paths.get(i, makespan)->manhattanDist(path[t]) / 2;
                t += std::max(dt - 1, 1);
                // t++;
                continue;
            }

            // vertex conflict
            if (paths.get(i, t) == path[t])
            {
#if PARALLEL
#pragma omp critical
#endif
                {
                    Conflict_p conflict = std::make_shared<Conflict>(id, i, t, path[t]);
                    conflicts.push_back(conflict);
                }
                int dt = paths.get(i, t)->manhattanDist(path[t]) / 2;
                t += std::max(dt - 1, 1);
                // t++;
                continue;
            }
            // swap conflict
            if (paths.get(i, t) == path[t - 1] && paths.get(i, t - 1) == path[t])
            {
#if PARALLEL
#pragma omp critical
#endif
                {
                    Conflict_p conflict = std::make_shared<Conflict>(id, i, t, path[t], path[t - 1]);
                    conflicts.push_back(conflict);
                }
            }
            int dt = paths.get(i, t)->manhattanDist(path[t]) / 2;
            t += std::max(dt - 1, 1);
            // t++;
        }
    }
}

LibCBS::Conflict_p LibCBS::chooseConflict(LibCBS::Conflicts &cfs)
{
    std::default_random_engine generator(std::time(0)); // Seed with current time
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    LibCBS::Conflict_p min_cf = cfs[0];

    for (int i = 0; i < cfs.size(); ++i)
    {
        auto cf = cfs[i];
        {
            if (min_cf->t >= cf->t and distribution(generator) < 0.5)
            {

                min_cf = cf;
            }
        }
    }

    return min_cf;
}