#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include "hca.hpp"
#include "paths.hpp"
#include "problem.hpp"
#include "plan.hpp"
#include "solver.hpp"
#include "ecbs.hpp"
#include "pbs.hpp"
#include "maxflow.hpp"
#include "pibt.hpp"
#include "pibt_complete.hpp"

#include "push_and_swap.hpp"
#include "../lacam/lacam.hpp"
#include "../unlabeled/tswap.hpp"
namespace py = pybind11;


PYBIND11_MODULE(MAPF, m ){
    m.doc() = "MAPF wrapper";

    pybind11::class_<Paths>(m, "Paths")
        .def(pybind11::init())
        .def(pybind11::init<int>())
        .def(pybind11::init<std::vector<Path>>())
        .def("get",py::overload_cast<int>(&Paths::get,py::const_),"get Path",py::return_value_policy::reference)
        .def("get",py::overload_cast<int,int>(&Paths::get,py::const_),"get node",py::return_value_policy::reference)
        .def("empty",py::overload_cast<>(&Paths::empty,py::const_),"empty")
        .def("empty",py::overload_cast<int>(&Paths::empty,py::const_),"empty")
        .def("clear",&Paths::clear,"clear")
        .def("size",&Paths::size,"size")
        .def("getMaxLengthPaths",&Paths::getMaxLengthPaths," get maximum length")
        .def("getSOC",&Paths::getSOC,"get sum of costs")
        .def("getMakespan",&Paths::getMakespan,"get makespan")
        .def("format",&Paths::format,"format")
        .def("countConflict",py::overload_cast<>(&Paths::countConflict,py::const_));

    py::class_<Plan>(m,"Plan")
        .def(py::init())
        .def("get",py::overload_cast<const int >(&Plan::get,py::const_),"get configuration at t",py::return_value_policy::reference)
        .def("get",py::overload_cast<const int,const int>(&Plan::get,py::const_),"time step, agent location",py::return_value_policy::reference)
        .def("getPath",&Plan::getPath,"get Path",py::return_value_policy::reference)
        .def("getPathCost",&Plan::getPathCost,"get Path Cost")
        .def("last",py::overload_cast<>(&Plan::last,py::const_),"last configuration",py::return_value_policy::reference)
        .def("last",py::overload_cast<const int >(&Plan::last,py::const_),"last configuration",py::return_value_policy::reference)
        .def("clear",&Plan::clear,"clear")
        .def("add",&Plan::add,"add new configuration")
        .def("empty",&Plan::empty,"empty()")
        .def("size",&Plan::size,"size()")
        .def("reverse",&Plan::reverse,"reverse the plan")
        .def("lastVec",&Plan::lastVec,"last configuration in vector")
        .def("getMakespan",&Plan::getMakespan,"getMakespan")
        .def("printPath",&Plan::printPath,"print path of an agent")
        .def("getSOC",&Plan::getSOC,"getSOC");


    pybind11::class_<Problem>(m,"Problem")
        .def(py::init<const std::string &>())
        .def(py::init<Problem *,int>())
        .def(py::init<Grid*,Config,Config,int,int>())
        .def("getNum",&Problem::getNum,"get number of robots")
        .def("getConfigStart",&Problem::getConfigStart,"get start config",py::return_value_policy::reference)
        .def("getConfigGoal",&Problem::getConfigGoal,"get goal config",py::return_value_policy::reference)
        .def("setStarts",&Problem::setStarts,"set start config")
        .def("setGoals",&Problem::setGoals,"set goal config");




    pybind11::class_<MinimumSolver>(m,"MinimumSolver")
        .def(py::init<Problem*>())
        .def("solve",&MinimumSolver::solve,"solve the problem")
        .def(py::init<Graph*,Config,Config,int>())
        .def("getSolution",&MinimumSolver::getSolution,"get solution",py::return_value_policy::reference)
        .def("setStarts",&MinimumSolver::setStarts,"set starts")
        .def("setGoals",&MinimumSolver::setGoals,"set goals")
        .def("setMaxTimestep",&MinimumSolver::setMaxTimestep,"set max time step")
        .def("getCompTime",&MinimumSolver::getCompTime,"get comp time");

    

    pybind11::class_<Solver,MinimumSolver>(m,"Solver")
        .def(py::init<Problem *>())
        .def(py::init<Graph*,Config,Config,int>())
        .def("getPath",&Solver::getPath,"get path",py::return_value_policy::reference)
        .def("initDistTable",&Solver::initDistTable,"initDistTable")
        .def("getLowerBoundSOC",&Solver::getLowerBoundSOC,"get soc LB")
        .def("resetEnv",&Solver::resetEnv,"reset environment")
        .def("getLowerBoundMakespan",&Solver::getLowerBoundMakespan,"get makespan LB")
        .def("getHeuristicMap",&Solver::getHeuristicMap,"get heuristic map")
        .def("localShortestPathInFOV",&Solver::localShortestPathInFOV,"get the heuristic channel")
        .def("planToPaths",&Solver::planToPaths,"plan to paths",py::return_value_policy::reference)
        .def("pathsToPlans",&Solver::pathsToPlan,"paths to plan",py::return_value_policy::reference);

    pybind11::class_<PushAndSwap,Solver>(m,"PushAndSwap")
        .def(pybind11::init<Problem*>())
        .def(pybind11::init<Grid*,Config,Config>());
    

    pybind11::class_<ECBS,Solver>(m,"ECBS")
        .def(pybind11::init<Problem*>())
        .def("getNumExpansions",&ECBS::getNumExpansions)
        .def(py::init<Graph*,Config,Config>());


    pybind11::class_<PBS,Solver>(m,"PBS")
        .def(pybind11::init<Problem*>())
        .def(py::init<Grid *,Config,Config>());

    pybind11::class_<PIBT,Solver>(m,"PIBT")
        .def(pybind11::init<Problem*>())
        .def(py::init<Grid *,Config,Config>());

    pybind11::class_<FlowBasedUMRPP,Solver>(m,"FlowBasedUMRPP")
        .def(py::init<Problem*>())
        .def(py::init<Grid*,Config,Config>())
        .def("solveWeighted",&FlowBasedUMRPP::solveWeighted,"solve using min-cost maxflow");


    pybind11::class_<PIBT_COMPLETE,Solver>(m,"PIBT_COMPLETE")
        .def(py::init<Problem*>())
        .def(py::init<Grid*,Config,Config>());

    pybind11::class_<LacamSolver,Solver>(m,"LacamSolver")
        .def(py::init<Grid*,Config,Config>())
        .def("solveInstance",&LacamSolver::solveInstance);
    

    pybind11::class_<HCA,Solver>(m,"HCA")
        .def(py::init<Problem*>())
        .def(py::init<Graph*,Config,Config,int>())
        .def("getNumExpansions",&HCA::getNumExpansions)
        .def("sovleWithTotalPriority",&HCA::solveWithTotalPriority,"prioritized planning",py::return_value_policy::reference);


    pybind11::class_<TSWAP,Solver>(m,"TSWAP")
        .def(py::init<Problem*>())
        .def(py::init<Grid*,Config,Config>());



}