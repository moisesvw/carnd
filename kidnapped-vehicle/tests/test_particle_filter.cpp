#include <gtest/gtest.h>
#include "../src/particle_filter.h"
#include <vector>
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

TEST(ParticleFilter, dataAssociation) {
    ParticleFilter pf;

    LandmarkObs l1, l2, l3, l4, l5;
    l1.id = 1; l1.x = 5; l1.y = 3;
    l2.id = 2; l2.x = 2; l2.y = 1;
    l3.id = 3; l3.x = 6; l3.y = 1;
    l4.id = 4; l4.x = 7; l4.y = 4;
    l5.id = 5; l5.x = 4; l5.y = 7;
    vector<LandmarkObs> ll1;
    ll1.push_back(l1) ; ll1.push_back(l2);
    ll1.push_back(l3) ; ll1.push_back(l4);
    ll1.push_back(l5) ; 

    LandmarkObs o1, o2, o3;
    o1.x = 6.0 ; o1.y = 3.0;
    o2.x = 2.0; o2.y = 1.9999999999999998;
    o3.x = 0.0; o3.y = 5.0;
    vector<LandmarkObs> ll2;
    ll2.push_back(o1); ll2.push_back(o2); ll2.push_back(o3);

    pf.dataAssociation(ll1, ll2);

    ASSERT_TRUE(ll2[0].id == 1);
    ASSERT_TRUE(ll2[1].id == 2);
    ASSERT_TRUE(ll2[2].id == 2);
}

TEST(ParticleFilter, gaussMulti) {
    ParticleFilter pf;
    double result = pf.gaussMulti(-2.0, 4.0, 0.3, 0.3);
    ASSERT_TRUE(result == 9.831848741505932e-49);
}

TEST(ParticleFilter, updateParticleWeight) {
    ParticleFilter pf;

    LandmarkObs l1, l2, l3, l4, l5;
    l1.id = 1; l1.x = 5; l1.y = 3;
    l2.id = 2; l2.x = 2; l2.y = 1;
    l3.id = 3; l3.x = 6; l3.y = 1;
    l4.id = 4; l4.x = 7; l4.y = 4;
    l5.id = 5; l5.x = 4; l5.y = 7;
    vector<LandmarkObs> ll1;
    ll1.push_back(l1) ; ll1.push_back(l2);
    ll1.push_back(l3) ; ll1.push_back(l4);
    ll1.push_back(l5) ; 

    LandmarkObs o1, o2, o3;
    o1.x = 6.0 ; o1.y = 3.0; o1.id = 12;
    o2.x = 2.0; o2.y = 1.9999999999999998; o2.id = 4;
    o3.x = 0.0; o3.y = 5.0; o3.id = 12;
    
    vector<LandmarkObs> ll2;
    ll2.push_back(o1); ll2.push_back(o2); ll2.push_back(o3);
    vector<Particle> pts;
    Particle pt;
    pt.weight = 1.0;
    pt.x = 5.0;
    pt.y = 8.0;
    pt.theta = 4.0;
    pts.push_back(pt);
    ASSERT_TRUE(pts[0].weight==1);
    pf.updateParticleWeight(pts[0],ll2, ll1, 0.3, 0.3);
    ASSERT_TRUE(pts[0].weight== 1.8963176765383805e-74);
}