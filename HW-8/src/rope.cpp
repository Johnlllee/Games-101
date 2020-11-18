#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D vec = (end - start) / (num_nodes - 1);
        Mass* prev = nullptr;
        for (int i = 0; i < num_nodes; i++) {
            Mass* mass = new Mass(start + vec * i, node_mass, false);
            if(prev==nullptr) {
                prev = mass;
            } else {
                Spring* spring = new Spring(prev, mass, k);
                springs.push_back(spring);
                prev = mass;
            }
            masses.push_back(mass);
        }

        // Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {

        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto v1 = s->m1->position;
            auto v2 = s->m2->position;
            auto v12_unit = (v2 - v1).unit();
            auto f = s->k * v12_unit * ((v2 - v1).norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces += -f;

            // TODO (Part 2.1): add internal damping force
            float k_d = 500;
            auto ve1 = s->m1->velocity;
            auto ve2 = s->m2->velocity;
            Vector2D f_d = k_d * dot(v12_unit, (ve2 - ve1)) * v12_unit;
            s->m1->forces += f_d;
            s->m2->forces += -f_d;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity
                m->forces += gravity * m->mass;
                // TODO (Part 2): Add global damping
                float k_d_global = 0.01;
                m->forces += - k_d_global * m->velocity;
                // TODO (Part 2): Then compute the new velocity and position
                Vector2D a = m->forces / m->mass;
                m->position += m->velocity * delta_t;
                m->velocity += a * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateSemiImplicitEuler(float delta_t, Vector2D gravity)
    {

        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto v1 = s->m1->position;
            auto v2 = s->m2->position;
            auto v12_unit = (v2 - v1).unit();
            auto f = s->k * v12_unit * ((v2 - v1).norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces += -f;

            // TODO (Part 2.1): add internal damping force
            float k_d = 500;
            auto ve1 = s->m1->velocity;
            auto ve2 = s->m2->velocity;
            Vector2D f_d = k_d * dot(v12_unit, (ve2 - ve1)) * v12_unit;
            s->m1->forces += f_d;
            s->m2->forces += -f_d;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity
                m->forces += gravity * m->mass;
                // TODO (Part 2): Add global damping
                float k_d_global = 0.011;
                m->forces += - k_d_global * m->velocity;
                // TODO (Part 2): Then compute the new velocity and position
                Vector2D a = m->forces / m->mass;
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            auto v1 = s->m1->position;
            auto v2 = s->m2->position;
            auto v12_unit = (v2 - v1).unit();
            auto f = s->k * v12_unit * ((v2 - v1).norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces += -f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;

                // TODO (Part 4): Add global Verlet damping
                float damping_factor = 0.000005f;
                m->position = m->position + (1 - damping_factor)*(m->position - m->last_position) + a*delta_t*delta_t;
                m->last_position = temp_position;
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
