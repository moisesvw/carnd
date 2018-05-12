#include <gtest/gtest.h>
#include "../src/particle_filter.h"

using namespace std;

TEST(ParticleFilter, initializer) {
    ParticleFilter pf;
    ASSERT_TRUE( pf.initialized() == false );
}
