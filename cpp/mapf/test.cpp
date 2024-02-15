#include <iostream>
#include <fstream>
#include "json.hpp"
#include "cbs.hpp"
#include <chrono>
#include "ecbs.hpp"
#include "cbst.hpp"
#include "ecbst.hpp"
#include <gperftools/profiler.h>
#include "hca.hpp"
#include "ecbsde.hpp"
#include "pibt.hpp"
#include "push_and_swap.hpp"
#include "ecbsw.hpp"
#include <atomic>
#include <array>

void hca_exe(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " grid_size instance_name output_name" << std::endl;
        // return 1;
    }

    // Get the arguments
    std::string map_name = argv[1];
    std::string instance_name = argv[2];
    std::string output_name = argv[3];

    Grid *graph = new Grid(map_name);
    nlohmann::json json_data;
    std::ifstream file(instance_name);
    std::cout << instance_name << std::endl;
    file >> json_data;

    // Convert the JSON data to a std::vector<std::vector<int>>.
    std::vector<std::vector<int>> starts_vec = json_data["starts"];
    std::vector<std::vector<int>> goals_vec = json_data["goals"];

    std::cout << "number of agents=" << starts_vec.size() << std::endl;
    Config starts, goals;

    for (int k = 0; k < starts_vec.size(); k++)
    {
        starts.push_back(graph->getNode(starts_vec[k][0], starts_vec[k][1]));
        goals.push_back(graph->getNode(goals_vec[k][0], goals_vec[k][1]));
    }
    // Open the output file
    std::ofstream outfile(output_name);

    HCA *solver = new HCA(graph, starts, goals, 120000);
    auto start_time = std::chrono::high_resolution_clock::now();
    // solver->solveInstance();
    ProfilerStart("test.prof");
    solver->solve();
    ProfilerStop();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    double seconds = duration_seconds.count();
    auto solution = solver->getSolution();
    // Call the solver function
    nlohmann::json data;

    // Add the data to the JSON object.
    data["mkpn"] = solution.getMakespan() / (double)solver->getLowerBoundMakespan();
    data["soc"] = solution.getSOC() / (double)solver->getLowerBoundSOC();
    data["runtime"] = seconds;
    // solver(grid_size, instance_name, data);

    // Write the data to the output file
    outfile << data << std::endl;

    // Close the output file
    outfile.close();
}

void push_exe(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " grid_size instance_name output_name" << std::endl;
        // return 1;
    }

    // Get the arguments
    std::string map_name = argv[1];
    std::string instance_name = argv[2];
    std::string output_name = argv[3];

    Grid *graph = new Grid(map_name);
    nlohmann::json json_data;
    std::ifstream file(instance_name);
    std::cout << instance_name << std::endl;
    file >> json_data;

    // Convert the JSON data to a std::vector<std::vector<int>>.
    std::vector<std::vector<int>> starts_vec = json_data["starts"];
    std::vector<std::vector<int>> goals_vec = json_data["goals"];

    std::cout << "number of agents=" << starts_vec.size() << std::endl;
    Config starts, goals;

    for (int k = 0; k < starts_vec.size(); k++)
    {
        starts.push_back(graph->getNode(starts_vec[k][0], starts_vec[k][1]));
        goals.push_back(graph->getNode(goals_vec[k][0], goals_vec[k][1]));
    }
    // Open the output file
    std::ofstream outfile(output_name);
    Problem *P = new Problem(graph, starts, goals, 150000);
    PushAndSwap *solver = new PushAndSwap(P);
    auto start_time = std::chrono::high_resolution_clock::now();
    // solver->solveInstance();
    // ProfilerStart("test.prof");
    solver->solve();
    // ProfilerStop();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    double seconds = duration_seconds.count();
    auto solution = solver->getSolution();
    // Call the solver function
    nlohmann::json data;

    // Add the data to the JSON object.
    data["mkpn"] = solution.getMakespan() / (double)solver->getLowerBoundMakespan();
    data["soc"] = solution.getSOC() / (double)solver->getLowerBoundSOC();
    data["runtime"] = seconds;
    // solver(grid_size, instance_name, data);

    // Write the data to the output file
    outfile << data << std::endl;

    // Close the output file
    outfile.close();
}

void pibt_exe(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " grid_size instance_name output_name" << std::endl;
        // return 1;
    }

    // Get the arguments
    std::string map_name = argv[1];
    std::string instance_name = argv[2];
    std::string output_name = argv[3];

    Grid *graph = new Grid(map_name);
    nlohmann::json json_data;
    std::ifstream file(instance_name);
    std::cout << instance_name << std::endl;
    file >> json_data;

    // Convert the JSON data to a std::vector<std::vector<int>>.
    std::vector<std::vector<int>> starts_vec = json_data["starts"];
    std::vector<std::vector<int>> goals_vec = json_data["goals"];

    std::cout << "number of agents=" << starts_vec.size() << std::endl;
    Config starts, goals;

    for (int k = 0; k < starts_vec.size(); k++)
    {
        starts.push_back(graph->getNode(starts_vec[k][0], starts_vec[k][1]));
        goals.push_back(graph->getNode(goals_vec[k][0], goals_vec[k][1]));
    }
    // Open the output file
    std::ofstream outfile(output_name);

    PIBT *solver = new PIBT(graph, starts, goals);
    auto start_time = std::chrono::high_resolution_clock::now();
    // solver->solveInstance();
    ProfilerStart("test.prof");
    solver->solve();
    ProfilerStop();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    double seconds = duration_seconds.count();
    auto solution = solver->getSolution();
    // Call the solver function
    nlohmann::json data;

    // Add the data to the JSON object.
    data["mkpn"] = solution.getMakespan() / (double)solver->getLowerBoundMakespan();
    data["soc"] = solution.getSOC() / (double)solver->getLowerBoundSOC();
    data["runtime"] = seconds;
    // solver(grid_size, instance_name, data);

    // Write the data to the output file
    outfile << data << std::endl;

    // Close the output file
    outfile.close();
}

void ecbs_exe(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " grid_size instance_name output_name" << std::endl;
        // return 1;
    }

    // Get the arguments
    std::string map_name = argv[1];
    std::string instance_name = argv[2];
    std::string output_name = argv[3];

    Grid *graph = new Grid(map_name);
    nlohmann::json json_data;
    std::ifstream file(instance_name);
    std::cout << instance_name << std::endl;
    file >> json_data;

    // Convert the JSON data to a std::vector<std::vector<int>>.
    std::vector<std::vector<int>> starts_vec = json_data["starts"];
    std::vector<std::vector<int>> goals_vec = json_data["goals"];

    std::cout << "number of agents=" << starts_vec.size() << std::endl;
    Config starts, goals;

    for (int k = 0; k < starts_vec.size(); k++)
    {
        starts.push_back(graph->getNode(starts_vec[k][0], starts_vec[k][1]));
        goals.push_back(graph->getNode(goals_vec[k][0], goals_vec[k][1]));
    }
    // Open the output file
    std::ofstream outfile(output_name);

    ECBS *solver = new ECBS(graph, starts, goals);

    auto start_time = std::chrono::high_resolution_clock::now();
    // solver->solveInstance();
    // ProfilerStart("test.prof");
    solver->solve();
    // ProfilerStop();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    double seconds = duration_seconds.count();
    auto solution = solver->getSolution();
    // Call the solver function
    nlohmann::json data;

    // Add the data to the JSON object.
    data["mkpn"] = solution.getMakespan() / (double)solver->getLowerBoundMakespan();
    data["soc"] = solution.getSOC() / (double)solver->getSocLB();
    data["runtime"] = seconds;
    data["numExpansion"] = solver->getNumExpansions();
    // solver(grid_size, instance_name, data);

    // Write the data to the output file
    outfile << data << std::endl;

    // Close the output file
    outfile.close();
}

void ecbst_exe(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " grid_size instance_name output_name" << std::endl;
        // return 1;
    }

    // Get the arguments
    std::string map_name = argv[1];
    std::string instance_name = argv[2];
    std::string output_name = argv[3];

    Grid *graph = new Grid(map_name);
    nlohmann::json json_data;
    std::ifstream file(instance_name);
    std::cout << instance_name << std::endl;
    file >> json_data;

    // Convert the JSON data to a std::vector<std::vector<int>>.
    std::vector<std::vector<int>> starts_vec = json_data["starts"];
    std::vector<std::vector<int>> goals_vec = json_data["goals"];

    std::cout << "number of agents=" << starts_vec.size() << std::endl;
    Config starts, goals;

    for (int k = 0; k < starts_vec.size(); k++)
    {
        starts.push_back(graph->getNode(starts_vec[k][0], starts_vec[k][1]));
        goals.push_back(graph->getNode(goals_vec[k][0], goals_vec[k][1]));
    }
    // Open the output file
    std::ofstream outfile(output_name);

    ECBST *solver = new ECBST(graph, starts, goals);
    auto start_time = std::chrono::high_resolution_clock::now();
    // solver->solveInstance();
    ProfilerStart("test.prof");
    solver->solve();
    ProfilerStop();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    double seconds = duration_seconds.count();
    auto solution = solver->getSolution();
    // Call the solver function
    nlohmann::json data;

    // Add the data to the JSON object.
    data["mkpn"] = solution.getMakespan() / (double)solver->getLowerBoundMakespan();
    data["soc"] = solution.getSOC() / (double)solver->getLowerBoundSOC();
    data["runtime"] = seconds;
    data["numExpansion"] = solver->getNumExpansions();
    // solver(grid_size, instance_name, data);

    // Write the data to the output file
    outfile << data << std::endl;

    // Close the output file
    outfile.close();
}

void ecbsw_exe(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " grid_size instance_name output_name" << std::endl;
        // return 1;
    }

    // Get the arguments
    std::string map_name = argv[1];
    std::string instance_name = argv[2];
    std::string output_name = argv[3];

    Grid *graph = new Grid(map_name);
    nlohmann::json json_data;
    std::ifstream file(instance_name);
    std::cout << instance_name << std::endl;
    file >> json_data;

    // Convert the JSON data to a std::vector<std::vector<int>>.
    std::vector<std::vector<int>> starts_vec = json_data["starts"];
    std::vector<std::vector<int>> goals_vec = json_data["goals"];

    std::cout << "number of agents=" << starts_vec.size() << std::endl;
    Config starts, goals;

    for (int k = 0; k < starts_vec.size(); k++)
    {
        starts.push_back(graph->getNode(starts_vec[k][0], starts_vec[k][1]));
        goals.push_back(graph->getNode(goals_vec[k][0], goals_vec[k][1]));
    }
    // Open the output file
    std::ofstream outfile(output_name);

    ECBSW *solver = new ECBSW(graph, starts, goals);
    auto start_time = std::chrono::high_resolution_clock::now();
    // solver->solveInstance();
    ProfilerStart("test.prof");
    solver->solve();
    ProfilerStop();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    double seconds = duration_seconds.count();
    auto solution = solver->getSolution();
    // Call the solver function
    nlohmann::json data;

    // Add the data to the JSON object.
    data["mkpn"] = solution.getMakespan() / (double)solver->getLowerBoundMakespan();
    data["soc"] = solution.getSOC() / (double)solver->getLowerBoundSOC();
    data["runtime"] = seconds;
    data["numExpansion"] = solver->getNumExpansions();
    // solver(grid_size, instance_name, data);

    // Write the data to the output file
    outfile << data << std::endl;

    // Close the output file
    outfile.close();
}

void ecbsde_exe(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " grid_size instance_name output_name" << std::endl;
        // return 1;
    }

    // Get the arguments
    std::string map_name = argv[1];
    std::string instance_name = argv[2];
    std::string output_name = argv[3];

    Grid *graph = new Grid(map_name);
    nlohmann::json json_data;
    std::ifstream file(instance_name);
    std::cout << instance_name << std::endl;
    file >> json_data;

    // Convert the JSON data to a std::vector<std::vector<int>>.
    std::vector<std::vector<int>> starts_vec = json_data["starts"];
    std::vector<std::vector<int>> goals_vec = json_data["goals"];

    std::cout << "number of agents=" << starts_vec.size() << std::endl;
    Config starts, goals;

    for (int k = 0; k < starts_vec.size(); k++)
    {
        starts.push_back(graph->getNode(starts_vec[k][0], starts_vec[k][1]));
        goals.push_back(graph->getNode(goals_vec[k][0], goals_vec[k][1]));
    }
    // Open the output file
    std::ofstream outfile(output_name);

    ECBSDE *solver = new ECBSDE(graph, starts, goals);
    auto start_time = std::chrono::high_resolution_clock::now();
    // solver->solveInstance();
    // ProfilerStart("test.prof");
    solver->solve();
    // ProfilerStop();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    double seconds = duration_seconds.count();
    auto solution = solver->getSolution();
    // Call the solver function
    nlohmann::json data;

    // Add the data to the JSON object.
    data["mkpn"] = solution.getMakespan() / (double)solver->getLowerBoundMakespan();
    data["soc"] = solution.getSOC() / (double)solver->getSocLB();
    data["runtime"] = seconds;
    data["numExpansion"] = solver->getNumExpansions();
    // solver(grid_size, instance_name, data);

    // Write the data to the output file
    outfile << data << std::endl;

    // Close the output file
    outfile.close();
}

void test()
{
    int num_agents = 10000;
    int path_length = 500;
    std::vector<Path> tmp(path_length, Path(path_length, nullptr));
    Path path(path_length, nullptr);
    Paths paths(tmp);

    auto start = std::chrono::high_resolution_clock::now();

    auto cnt = paths.countConflict(5, path);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Sequential: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";
}

void testThreading()
{
    std::vector<double> sums(16, 0);
    auto threading = [&](long long k, int thread_id)
    {
        // double sum = 0;
        long long i = 0;
        while (i <= k)
        {
            sums[thread_id] += i / 1000.;
            i++;
        }
        // sums[thread_id] = sum;
        // std::cout << "sum=" << sum << std::endl;
        // return sum;
    };
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::thread> threads;
    for (int i = 0; i < 16; ++i)
    {
        threads.emplace_back(threading, 1e6, i);
    }
    for (auto &thread : threads)
    {
        thread.join();
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    printf("parallel=%f\n", elapsed.count());

    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 16; ++i)
    {
        threading(1e6, i);
    }
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    printf("serial=%f\n", elapsed.count());
}

void serial()
{
    auto start = std::chrono::high_resolution_clock::now();
    const int num_iterations = 1 << 27;

    // Number of threads to spawn
    const int num_threads = 4;

    // Atomic integers to increment
    std::atomic<int> sum{0};
    for (int i = 0; i < num_iterations; i++)
    {
        sum += 1;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "sum=" << sum.load() << std::endl;
    printf("serial=%f\n", elapsed.count());
}

void false_sharing()
{
    auto start = std::chrono::high_resolution_clock::now();
    const int num_iterations = 1 << 27;

    // Number of threads to spawn
    const int num_threads = 4;

    // Atomic integers to increment
    std::array<std::atomic<int>, 4> counters;
    for (int i = 0; i < 4; i++)
    {
        counters[i] = 0;
    }
    // auto counters = std::make_array<std::atomic<int>>(0, 0, 0, 0);
    // std::array<std::atomic<int>, 4> counters = {std::atomic<int>{0}, std::atomic<int>{0}, std::atomic<int>{0}, std::atomic<int>{0}};

    std::atomic<int> final_sum{0};

    // Number of elements to process per thread
    const int elements_per_thread = num_iterations / num_threads;

    // Lambda for our work
    auto work = [&](int thread_id)
    {
        // std::atomic<int>  sum {0};
        for (int i = 0; i < elements_per_thread; i++)
        {
            counters[thread_id]++;
            // sum++;
        }
        final_sum += counters[thread_id];
        // final_sum += sum;
    };

    // Spawn threads
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; i++)
    {
        threads.emplace_back(work, i);
    }
    for (auto &thread : threads)
    {
        thread.join();
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    printf("threading=%f\n", elapsed.count());
    std::cout << final_sum.load() << std::endl;
}

// Function to solve the problem

int main(int argc, char *argv[])
{
    // std::cout << "Maximum value of int: " << std::numeric_limits<int>::max() << std::endl;
    // cbs_exe(argc, argv);
    // cbst_exe(argc, argv);
    // ecbst_exe(argc, argv);
    ecbsw_exe(argc, argv);
    // push_exe(argc, argv);
    // testThreading();
    // serial();
    // false_sharing();

    // pibt_exe(argc, argv);
    // hca_exe(argc, argv);
    // ecbsde_exe(argc, argv);
    // ecbs_exe(argc, argv);
    // test();

    return 0;
}