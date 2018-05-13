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

TEST(ParticleFilter, prediction) {
    ParticleFilter pf;
    double std[3] = {2.0, 2.0, 0.05 };
    pf.init(1.0, 2.0, 3.9, std);
    Particle p = pf.particles[0];
    pf.prediction(0.1, std, 3, 3.8);
    double x = p.x + (3/3.8) * (sin(p.theta + 3.8 * 0.1) - sin(p.theta));

    ASSERT_TRUE(pf.particles[0].x <= p.x);
}