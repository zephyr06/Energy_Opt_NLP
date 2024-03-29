#include "sources/ControlOptimization/BatchControlOptimizeNasri19.h"
#include "sources/Tools/profilier.h"
using namespace rt_num_opt;
TEST(overall, batch) {
    std::string dataset_path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/"
        "ControlPerformance_Hybrid_DAG/N0/";
    ClearResultFiles(dataset_path);
    Obj_Pow = 2;
    enableReorder = 1;
    double obj_act = BatchOptimizeNasri19(0);
    EXPECT(obj_act <= 0.68);
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
