#pragma once

#include "tbb/task_scheduler_init.h"

#include "problem.hpp"
#include "io.hpp"
#include "global/space.hpp"
// #include "dagSched/tests.h" // to add np_schedulability_analysis
#include "sources/Tools/profilier.h"
#include "sources/RTA/RTA_BASE.h"
#include "sources/TaskModel/DAG_Narsi19.h"

namespace rt_num_opt
{
    /**
     * @brief this class overrides all the methods of RTA_BASE except CheckSchedulabilityDirect
     *
     */
    class RTA_Narsi19 : public RTA_BASE<DAG_Narsi19>
    {
    private:
        DAG_Narsi19 dagNarsi_;

    public:
        RTA_Narsi19(DAG_Narsi19 &dagNarsi) : RTA_BASE<DAG_Narsi19>(dagNarsi)
        {
            dagNarsi_ = dagNarsi;
        }

        double RTA_Common_Warm(double beginTime, int index) override
        {
            CoutError("This function should not be used: RTA_Common_Warm, RTA_Narsi19!");
            return -1;
        }

        VectorDynamic ResponseTimeOfTaskSet(const VectorDynamic &warmStart)
        {
            return ResponseTimeOfTaskSet();
        }
        VectorDynamic ResponseTimeOfTaskSet()
        {
            BeginTimer(__func__);
            // prepare input
            std::stringstream tasksInput;
            tasksInput << dagNarsi_.ConvertTasksetToCsv();
            std::stringstream dagInput;
            dagInput << dagNarsi_.convertDAGsToCsv();
            std::stringstream abortsInput;
            tbb::task_scheduler_init init(tbb::task_scheduler_init::automatic);

            NP::Scheduling_problem<dtime_t> problem{
                NP::parse_file<dtime_t>(tasksInput),
                NP::parse_dag_file(dagInput),
                NP::parse_abort_file<dtime_t>(abortsInput),
                static_cast<unsigned int>(rt_num_opt::core_m_dag)};

            // Set common analysis options
            NP::Analysis_options opts;
            opts.timeout = 0;
            opts.max_depth = 0;
            opts.early_exit = true;
            opts.num_buckets = problem.jobs.size();
            opts.be_naive = 0;

            // Actually call the analysis engine
            auto space = NP::Global::State_space<dtime_t>::explore(problem, opts);

            // Extract the analysis results
            // std::vector<double> rta(dagNarsi_.tasks_.size(), INT32_MAX);
            VectorDynamic rta = GenerateVectorDynamic(dagNarsi_.tasks_.size());

            if (space.is_schedulable())
            {
                for (const auto &j : problem.jobs)
                {
                    Interval<dtime_t> finish = space.get_finish_times(j);
                    // std::cout << "[" << j.get_task_id() << ", " << j.get_job_id() << "] ";
                    // std::cout << std::max<long long>(0, (finish.from() - j.earliest_arrival())) << " ";
                    // std::cout << std::endl;
                    int globalJobId = j.get_job_id();
                    auto idTuple = dagNarsi_.IdsGlobal2Job(globalJobId);
                    // obtain WCRT from all the same job instances
                    size_t indexJobTaskLevel = dagNarsi_.IdJobTaskLevel(idTuple.taskId, idTuple.jobId);

                    double jobInstanceRT = (std::max<long long>(0, (finish.from() - j.earliest_arrival())));
                    if (jobInstanceRT > rta(indexJobTaskLevel))
                    {
                        rta(indexJobTaskLevel) = jobInstanceRT;
                    }
                }
            }
            else
            {
                for (long int i = 0; i < rta.rows(); i++)
                {
                    rta(i) = INT32_MAX;
                }
            }
            EndTimer(__func__);
            return rta;
        }
        bool CheckSchedulability(VectorDynamic warmStart,
                                 bool whetherPrint = false, double tol = 0)
        {
            return CheckSchedulability();
        }
        bool CheckSchedulability(bool whetherPrint = false)
        {
            VectorDynamic rta = ResponseTimeOfTaskSet();
            return CheckSchedulabilityDirect(rta);
        }
    };
}