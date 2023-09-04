#include "sources/ControlOptimization/BatchControlOptimizeNasri19.h"
#include "sources/Tools/profilier.h"
using namespace rt_num_opt;
TEST(overall, batch) {
    double obj_act = BatchOptimizeNasri19(0);
    EXPECT(obj_act <= 0.4125);
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
