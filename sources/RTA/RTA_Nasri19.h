#pragma once

#include "global/space.hpp"
#include "io.hpp"
#include "problem.hpp"
#include "sources/RTA/RTA_BASE.h"
#include "sources/TaskModel/DAG_Nasri19.h"
#include "sources/Tools/profilier.h"
#include "tbb/task_scheduler_init.h"

namespace rt_num_opt {
/**
 * @brief this class overrides all the methods of RTA_BASE except
 * CheckSchedulabilityDirect
 *
 */
VectorDynamic UnschedulableRTA(int n) {
    VectorDynamic rta = GenerateVectorDynamic(n);
    for (long int i = 0; i < rta.rows(); i++) {
        rta(i) = INT32_MAX;
    }
    return rta;
}
class RTA_Nasri19 : public RTA_BASE<DAG_Nasri19> {
   private:
    DAG_Nasri19 dagNasri_;  // TODO: avoid this copy later

   public:
    RTA_Nasri19(const DAG_Nasri19 &dagNasri) : RTA_BASE<DAG_Nasri19>(dagNasri) {
        dagNasri_ = dagNasri;
    }

    static std::string type() { return "Nasri19"; }

    double RTA_Common_Warm(double beginTime, int index) override {
        // CoutError(
        //     "This function should not be used: RTA_Common_Warm,
        //     RTA_Nasri19!");
        // return -1;
        return ResponseTimeOfTaskSet()(index, 0);
    }

    VectorDynamic ResponseTimeOfTaskSet(const VectorDynamic &warmStart) {
        return ResponseTimeOfTaskSet();
    }

    VectorDynamic ResponseTimeOfDag(size_t dag_index) {
        return GenerateVectorDynamic1D(0);
    }

    bool IsTasksValid() const {
        for (uint i = 0; i < dagNasri_.tasks_.size(); i++) {
            const Task &task_curr = dagNasri_.tasks_[i];
            if (task_curr.period < task_curr.executionTimeOrg ||
                task_curr.deadline < task_curr.executionTimeOrg)
                return false;
        }
        return true;
    }

    VectorDynamic ResponseTimeOfTaskSet(double time_out=rt_num_opt::Nasri19Param_timeout) {
        IncrementCallingTimes();
        if (dagNasri_.IsHyperPeriodOverflow())
            return UnschedulableRTA(dagNasri_.tasks_.size());
        if (!IsTasksValid())
            return GenerateInvalidRTA(dagNasri_.tasks_.size());

        BeginTimer(__func__);
        // prepare input
        std::stringstream tasksInput;
        tasksInput << dagNasri_.ConvertTasksetToCsv(debugMode == 1);
        std::stringstream dagInput;
        dagInput << dagNasri_.convertDAGsToCsv(debugMode == 1);
        std::stringstream abortsInput;
        tbb::task_scheduler_init init(tbb::task_scheduler_init::automatic);

        NP::Scheduling_problem<dtime_t> problem{
            NP::parse_file<dtime_t>(tasksInput), NP::parse_dag_file(dagInput),
            NP::parse_abort_file<dtime_t>(abortsInput),
            static_cast<unsigned int>(rt_num_opt::core_m_dag)};

        // Set common analysis options
        NP::Analysis_options opts;
        opts.timeout = time_out;
        opts.max_depth = rt_num_opt::Nasri19Param_max_depth;
        opts.early_exit = true;
        opts.num_buckets = problem.jobs.size();
        opts.be_naive = 0;

        // Actually call the analysis engine
        auto space = NP::Global::State_space<dtime_t>::explore(problem, opts);

        // Extract the analysis results
        // std::vector<double> rta(dagNasri_.tasks_.size(), INT32_MAX);
        VectorDynamic rta = GenerateVectorDynamic(dagNasri_.tasks_.size());

        if (space.is_schedulable()) {
            for (const auto &j : problem.jobs) {
                Interval<dtime_t> finish = space.get_finish_times(j);
                // std::cout << "[" << j.get_task_id() << ", " << j.get_job_id()
                // << "] "; std::cout << std::max<long long>(0, (finish.from() -
                // j.earliest_arrival())) << " "; std::cout << std::endl;
                int globalJobId = j.get_job_id();
                auto idTuple = dagNasri_.IdsGlobal2Job(globalJobId);
                // obtain WCRT from all the same job instances
                size_t indexJobTaskLevel =
                    dagNasri_.IdJobTaskLevel(idTuple.taskId, idTuple.jobId);

                double jobInstanceRT = (std::max<long long>(
                    0, (finish.from() - j.earliest_arrival())));
                if (jobInstanceRT > rta(indexJobTaskLevel)) {
                    rta(indexJobTaskLevel) = jobInstanceRT;
                }
            }
        } else {
            rta = UnschedulableRTA(dagNasri_.tasks_.size());
        }
        EndTimer(__func__);
        return rta;
    }

    bool CheckSchedulability(VectorDynamic warmStart, bool whetherPrint = false,
                             double tol = 0) {
        return CheckSchedulability();
    }
    bool CheckSchedulability(bool whetherPrint = false) {
        VectorDynamic rta = ResponseTimeOfTaskSet();
        return CheckSchedulabilityDirect(rta);
    }
    bool CheckSchedulabilityLongTimeOut() {
        VectorDynamic rta = ResponseTimeOfTaskSet(rt_num_opt::Nasri19Param_timeout*3);
        return CheckSchedulabilityDirect(rta);
    }
};

inline VectorDynamic GetNasri19RTA(const DAG_Nasri19 &dagNasri) {
    RTA_Nasri19 r(dagNasri);
    return r.ResponseTimeOfTaskSet();
}
}  // namespace rt_num_opt