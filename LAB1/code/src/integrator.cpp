#include "integrator.h"

#include "configs.h"

void ExplicitEuler::integrate(const std::vector<Particles *> &particles, std::function<void(void)>) const {
  // TODO: Integrate velocity and acceleration
  //   1. Integrate velocity.
  //   2. Integrate acceleration.
  //   3. You should not compute position using acceleration. Since some part only update velocity. (e.g. impulse)
  // Note:
  //   1. You don't need the simulation function in explicit euler.
  //   2. You should do this first because it is very simple. Then you can chech your collision is correct or not.
  //   3. This can be done in 5 lines. (Hint: You can add / multiply all particles at once since it is a large matrix.)
  for (auto &p : particles) {
    //deltatime in config.h
    //change position first or it will get wrong position    
    p->position() += deltaTime * p->velocity();
    p->velocity() += deltaTime * p->acceleration();
  }
}

void ImplicitEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
    // TODO: Integrate velocity and acceleration
    //   1. Backup original particles' data.
    //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
    //   3. Compute refined Xn+1 using (1.) and (2.).
    // Note:
    //   1. Use simulateOneStep with modified position and velocity to get Xn+1.
    //step1
    std::vector<Particles> backup;
    for (int i = 0; i < particles.size(); ++i) {
      backup.push_back(*particles[i]);  // backup the particle
    }
    // step2
    for (auto &p : particles) {
        p->position() += deltaTime * p->velocity();
        p->velocity() += deltaTime * p->acceleration();
    }
    simulateOneStep();
    // step3
    for (int i = 0; i < particles.size(); ++i) {
        particles[i]->position() = backup[i].position() + particles[i]->velocity() * deltaTime;
        particles[i]->velocity() = backup[i].velocity() + particles[i]->acceleration() * deltaTime;
    }
}

void MidpointEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.
  // step1
  std::vector<Particles> backup;
  for (int i = 0; i < particles.size(); ++i) {
    backup.push_back(*particles[i]);  // backup the particle
  }
  simulateOneStep();
  // step2
  for (auto &p : particles) {
    p->position() += 0.5f*deltaTime * p->velocity();
    p->velocity() += 0.5f*deltaTime * p->acceleration();
  }
  // step3
  for (int i = 0; i < particles.size(); ++i) {
    particles[i]->position() = backup[i].position() + particles[i]->velocity() * deltaTime;
    particles[i]->velocity() = backup[i].velocity() + particles[i]->acceleration() * deltaTime;
  }
}

void RungeKuttaFourth::integrate(const std::vector<Particles *> &particles,
                                 std::function<void(void)> simulateOneStep) const {
    // TODO: Integrate velocity and acceleration
    //   1. Backup original particles' data.
    //   2. Compute k1, k2, k3, k4
    //   3. Compute refined Xn+1 using (1.) and (2.).
    // Note:
    //   1. Use simulateOneStep with modified position and velocity to get Xn+1.
    //backup
    std::vector<Particles> backup;
    

    for (int i = 0; i < particles.size(); ++i) {
        backup.push_back(*particles[i]);  // backup the particle
    }
    std::vector<Particles> k1(backup);
    std::vector<Particles> k2(backup);
    std::vector<Particles> k3(backup);
    std::vector<Particles> k4(backup);
    //k1
    for (int i = 0; i < backup.size(); ++i) {
        //update the particle
        particles[i]->position() = backup[i].position() + (particles[i]->velocity() * deltaTime * 0.5f);
        particles[i]->velocity() = backup[i].velocity() + (particles[i]->acceleration() * deltaTime * 0.5f);
        //store k1
        k1[i].position() = particles[i]->velocity()*deltaTime;
        k1[i].velocity() = particles[i]->acceleration() * deltaTime;
    }
    simulateOneStep();
    for (int i = 0; i < backup.size(); ++i) {
      // update the particle
      particles[i]->position() = backup[i].position() + (particles[i]->velocity() * deltaTime * 0.5f);
      particles[i]->velocity() = backup[i].velocity() + (particles[i]->acceleration() * deltaTime * 0.5f);
      // store k2
      k2[i].position() = particles[i]->velocity() * deltaTime;
      k2[i].velocity() = particles[i]->acceleration() * deltaTime;
    }
    simulateOneStep();
    for (int i = 0; i < backup.size(); ++i) {
      // update the particle
      particles[i]->position() = backup[i].position() + (particles[i]->velocity() * deltaTime * 0.5f);
      particles[i]->velocity() = backup[i].velocity() + (particles[i]->acceleration() * deltaTime * 0.5f);
      // store k3
      k3[i].position() = particles[i]->velocity() * deltaTime;
      k3[i].velocity() = particles[i]->acceleration() * deltaTime;
    }
    simulateOneStep();
    for (int i = 0; i < backup.size(); ++i) {
      // store k4
      k4[i].position() = particles[i]->velocity() * deltaTime;
      k4[i].velocity() = particles[i]->acceleration() * deltaTime;
    }
    for (int i = 0; i < backup.size(); ++i) {
      //  Runge-Kutta
      particles[i]->position() =
          backup[i].position() +
          (k1[i].position() + 2.0f * k2[i].position() + 2.0f * k3[i].position() + k4[i].position()) / 6.0f;

      particles[i]->velocity() =
          backup[i].velocity() +
          (k1[i].velocity() + 2.0f * k2[i].velocity() + 2.0f * k3[i].velocity() + k4[i].velocity()) / 6.0f;
    }


}
