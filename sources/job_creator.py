# Copyright (c) 2017 - 2020 Kiriti Nagesh Gowda, Inc. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

__author__ = "Sen Wang"
# __copyright__   = "Copyright 2018 - 2020, Sen Wang"
__license__ = "MIT"

# __version__     = "1.0.1"

import argparse
import random
import numpy as np
import os
# from utils import *




def clear_dataset(path_dataset):
    files_periodic = []
    for f in os.listdir(path_dataset):
        if (f.split('-')[0] == "periodic"):
            task_path = os.path.join(path_dataset, f)
            os.remove(task_path)


def Uunifast(N, uti_ave):
    """
    Taken from Uunifast paper
    :param N:
    :param uti_ave:
    :return:
    """
    sum_u = uti_ave
    uti_list = np.zeros(N)
    for i in range(1, N):
        next_sum_u = sum_u * np.power(random.random(), 1 / (N - i))
        uti_list[i - 1] = sum_u - next_sum_u
        sum_u = next_sum_u
    uti_list[-1] = next_sum_u
    return uti_list


if __name__ == "__main__":
    # import arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--num_tasksets', type=int, default=10,
                        help='Number of job sets to be created [type:INT range:1 to N] - optional (default:1)')
    parser.add_argument('--num_job', type=int, default=3,
                        help='Number of jobs in each job sets [type:INT range:1 to N] - optional (default:100)')
    parser.add_argument('--deadline_portion', type=float, default=1.0,
                        help='the portion of task deadline w.r.t. task period - optional (default:1.0)')
    parser.add_argument('--utilization_total', type=str, default=0.5,
                        help='Total job utilization, - optional (default:0.5)')
    parser.add_argument('--desperate_mode', type=str, default=0,
                        help='Whether desperate mode, 1 or 0')
    parser.add_argument('--desperate_portion', type=float, default=0.1,
                        help='Whether desperate mode, 1 or 0')
    parser.add_argument('--overhead_max', type=int, default=10,
                        help='maximum preemption overhead for each task')
    # *************************************************************************************
    parser.add_argument('--directory', type=str, default='../TaskData',
                        help='Output Directory - optional (default:~/SRTG_jobCreator)')
    parser.add_argument('--jobset_name', type=str, default='periodic-set',
                        help='Job set prefix name - optional (default:aperiodic-set)')
    # parser.add_argument('--job_lambda',     type=float, default=0.5, help='Job arrival rate: lambda [type:float range:0.001 to 1.0] - optional (default:0.5)')
    parser.add_argument('--period_min', type=str, default=10, help='Minimum job period, - optional (default:100)')
    parser.add_argument('--period_max', type=str, default=100, help='Maximum job period, - optional (default:100)')
    parser.add_argument('--utilization_min_per_job', type=str, default=0.03,
                        help='Minimum job utilization factor, - optional (default:0.03)')
    args = parser.parse_args()

    # get arguments
    numJobSet = args.num_tasksets
    numJobsPerSet = args.num_job
    deadline_portion = args.deadline_portion
    utilization_total = float(args.utilization_total)
    desperate_mode = int(args.desperate_mode)
    desperate_portion = args.desperate_portion
    overhead_max = int(args.overhead_max)

    outputDirectory = args.directory
    jobSetName = args.jobset_name
    period_min = int(args.period_min)
    period_max = int(args.period_max)
    utilization_min_per_job = args.utilization_min_per_job

    overhead_min = 0
    overhead_max = overhead_max

    clear_dataset(outputDirectory)

    # setup output directory
    jobCreatorDir = os.path.expanduser(outputDirectory)
    if not os.path.exists(jobCreatorDir):
        os.makedirs(jobCreatorDir)

    # create num job sets

    for s in range(numJobSet):
        fileName_Jobs = jobCreatorDir + '/' + jobSetName + '-' + str(s) + '-syntheticJobs.csv'

        utilization_left = utilization_total
        desperate_mode_ref = desperate_mode
        # create a job set - jobs.csv
        with open(fileName_Jobs, 'w+') as f:
            f.write('JobID,Offset,Period,Overhead,ExecutionTime,DeadLine\n')

            iteration_monitor = 0
            uti_list = Uunifast(numJobsPerSet, utilization_total)
            for x in range(numJobsPerSet):
                iteration_monitor = iteration_monitor + 1
                if iteration_monitor > numJobsPerSet * 100:
                    raise Exception("The given configuration is hard to find, the system gives up")

                # Job Number
                jobNumber = x

                # Job offset
                offset = 0

                # Job period
                period_curr = random.randint(period_min, period_max) * 10

                # Job Execution Time
                execution_time = int(np.ceil(period_curr * uti_list[x]))

                # job overhead
                overhead_job = random.randint(overhead_min, min(overhead_max, execution_time))

                # Job Deadline (relative)
                if desperate_mode_ref > 0:
                    deadline = random.randint(execution_time + 1, int((execution_time + 1) * (1 + desperate_portion)))
                    desperate_mode_ref = desperate_mode_ref - 1
                else:
                    deadline = random.randint(execution_time + 1, int((period_curr + 1) * deadline_portion + 1))
                deadline = min(deadline, period_curr)
                # Write Job Info
                f.write(
                    str(jobNumber) + ',' + str(offset) + ',' + str(period_curr) + ',' + str(overhead_job) + ',' + str(
                        execution_time) + ',' + str(deadline) + '\n')
    # print("One task set is generated and stored, the total utilization is: ", utilization_total)
