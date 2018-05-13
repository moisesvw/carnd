#include <gtest/gtest.h>
#include "../src/particle_filter.h"

using namespace std;

TEST(ParticleFilter, initializer) {
    ParticleFilter pf;
    ASSERT_TRUE( pf.initialized() == false );
}

TEST(ParticleFilter, init) {
    ParticleFilter pf;
    double std[3] = {2.0, 2.0, 0.05 };
    pf.init(1.0, 2.0, 3.9, std);
    ASSERT_TRUE(pf.particles.size() == 100);
}