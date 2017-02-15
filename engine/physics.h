/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#ifndef PHYSICS_ENGINE_PHYSICS_H
#define PHYSICS_ENGINE_PHYSICS_H

#include <chrono>
#include <iomanip>
#include "linalg.h"


/**
 * Try modelling:
 *
 * Simple collisions [done]
 * Glancing collision [done]
 * Drag (through air)
 * Inelastic collisions (damping)
 * Collisions between spheres with fricative surfaces (imparting angular momentum) [done]
 * Gravity
 * Friction (on surfaces, rolling friction) and contact physics
 */

enum class ObjectType {
    PARTICLE,
    BIG_PARTICLE,
    SOLID_SPHERE,
    ROTOR
};


class PhysicsObject {
public:
//    virtual ~PhysicsObject() = 0;

    virtual ObjectType GetType() const = 0;

    /**
     * Updates the kinematics of the object.
     *
     * @param dt
     */
    virtual void Update(double dt) = 0;

    /**
     * Returns the position of the object.
     *
     * @return
     */
    virtual Vector3 const &GetPosition() const = 0;

    /**
     * Returns the orientation of the object as a vector, whose direction defines an axis, and whose magnitude defines
     * the anticlockwise rotation of the object from its original orientation about this axis.
     *
     * @return
     */
    virtual Vector3 const &GetRotation() const = 0;

    virtual void ClearForces() = 0;
};


class ParticleInterface : public PhysicsObject {
public:
//    virtual ~ParticleInterface() = 0;

    virtual void SetPosition(Vector3 const &position) = 0;

    virtual double GetMass() const = 0;

    virtual void SetMass(double mass) = 0;

    virtual const Vector3 &GetVelocity() const = 0;

    virtual void SetVelocity(Vector3 const &velocity) = 0;

    virtual void ApplyForce(Vector3 const &force) = 0;
};


class RotorInterface {
public:
    virtual void SetRotation(Vector3 const &rotation) = 0;

    virtual Matrix3 const &GetInertiaTensor() const = 0;

    virtual void SetInertiaTensor(Matrix3 const &inertia_tensor) = 0;

    virtual const Vector3 &GetAngularVelocity() const = 0;

    virtual void SetAngularVelocity(Vector3 const &angular_velocity) = 0;

    virtual void ApplyTorque(Vector3 const &torque) = 0;
};


class CollisionInterface {
public:
    /**
     * Reverts kinematics to before the collision.
     */
    virtual void Revert() = 0;
};


class Particle : public ParticleInterface {
private:
    double mass_;
    Vector3 position_;
    Vector3 velocity_;
    Vector3 total_force_;
    Vector3 rotation_; // Unused

public:
    Particle(double mass) : mass_{mass} {
    }

    Particle &operator=(Particle const &rhs) {
        this->mass_ = rhs.mass_;
        this->position_ = rhs.position_;
        this->velocity_ = rhs.velocity_;
        this->total_force_ = rhs.total_force_;
        this->rotation_ = rhs.rotation_;
        return (*this);
    }

    ObjectType GetType() const override {
        return ObjectType::PARTICLE;
    }

    /**
     * F = m * a
     *
     * Under constant acceleration, the following equations are exact, even if dt is not small:
     * x = x0 + v0 * dt + 1/2 * a * dt^2
     * v = v0 + a * dt
     *
     * @param dt
     */
    void Update(double dt) override {
        // Take the current values of r_ and v_ to be the initial position and velocity x0 and v0
        this->position_ += dt * this->velocity_ + dt * dt / 2.0 / this->mass_ * this->total_force_;

        // Now update v_
        this->velocity_ += dt / this->mass_ * this->total_force_;
    }

    Vector3 const &GetPosition() const override {
        return this->position_;
    }

    void SetPosition(Vector3 const &position) override {
        this->position_ = position;
    }

    Vector3 const &GetRotation() const override {
        return this->rotation_;
    }

    double GetMass() const override {
        return this->mass_;
    }

    void SetMass(double mass) override {
        if (mass <= 0.0) {
            throw std::invalid_argument("Mass must be greater than zero");
        }
        this->mass_ = mass;
    }

    const Vector3 &GetVelocity() const override {
        return this->velocity_;
    }

    void SetVelocity(Vector3 const &velocity) override {
        this->velocity_ = velocity;
    }

    void ApplyForce(Vector3 const &force) override {
        // Exploits the principle of superposition.
        this->total_force_ += force;
    }

    void ClearForces() override {
        this->total_force_ = Vector3();
    }
};


class BigParticle : public ParticleInterface, public CollisionInterface {
private:
    Particle particle_;
    Particle particle_prev_;
    double radius_;

public:
    BigParticle(double mass, double radius) : particle_(mass), particle_prev_(mass), radius_{1.0} {
    }

    ObjectType GetType() const override {
        return ObjectType::BIG_PARTICLE;
    }

    void Update(double dt) override {
        this->particle_prev_ = this->particle_;
        this->particle_.Update(dt);
    }

    double GetMass() const override {
        return this->particle_.GetMass();
    }

    void SetMass(double mass) override {
        this->particle_.SetMass(mass);
    }

    double GetRadius() const {
        return this->radius_;
    }

    void SetRadius(double radius) {
        if (radius <= 0.0) {
            throw std::invalid_argument("Radius must be greater than zero");
        }
        this->radius_ = radius;
    }

    const Vector3 &GetPosition() const override {
        return this->particle_.GetPosition();
    }

    const Vector3 &GetVelocity() const override {
        return this->particle_.GetVelocity();
    }

    void SetVelocity(Vector3 const &v) override {
        this->particle_.SetVelocity(v);
    }

    void ApplyForce(Vector3 const &f) override {
        this->particle_.ApplyForce(f);
    }

    void ClearForces() override {
        this->particle_.ClearForces();
    }

    virtual void Revert() override {
        this->particle_ = this->particle_prev_;
    }

    const Vector3 &GetRotation() const override {
        return this->particle_.GetRotation();
    }

    void SetPosition(Vector3 const &position) override {
        this->particle_.SetPosition(position);
    }
};


/**
 * Simple elastic collision between two masses, ignoring the size of the objects.
 *
 * @param bp
 */
void CollideObjects(BigParticle *obj1, BigParticle *obj2, double dt) {
    double r1 = obj1->GetRadius();
    double r2 = obj2->GetRadius();

    Vector3 r = obj2->GetPosition() - obj1->GetPosition();

    if ((r1 + r2) * (r1 + r2) >= Dot(r, r)) {
        obj1->Revert();
        obj2->Revert();

        double m1 = obj1->GetMass();
        double m2 = obj2->GetMass();
        auto v1 = obj1->GetVelocity();
        auto v2 = obj2->GetVelocity();
        auto two_com_v = 2.0 / (m1 + m2) * (m1 * v1 + m2 * v2);
        
        obj1->SetVelocity(two_com_v - obj1->GetVelocity());
        obj2->SetVelocity(two_com_v - obj2->GetVelocity());
    }
}


class UniformSphericalRotor : public RotorInterface, public PhysicsObject {
private:
    Matrix3 inertia_tensor_; // Moment of inertia tensor
    Vector3 omega_; // Angular velocity
    Vector3 theta_; // Rotation
    Vector3 total_torque_;
    Matrix3 inverse_inertia_tensor;
    Vector3 position_; // Unused

public:
    UniformSphericalRotor(double radius, double mass) :
            inertia_tensor_(2.0 * radius * radius / 5.0 * mass),
            inverse_inertia_tensor(1.0 / (2.0 * radius * radius / 5.0 * mass)) {
    }

    ObjectType GetType() const override {
        return ObjectType::ROTOR;
    }

    void Update(double dt) override {
        Vector3 ang_acc = Dot(this->inverse_inertia_tensor, this->total_torque_);
        this->theta_ += dt * this->omega_ + dt * dt / 2.0 * ang_acc;
        this->omega_ += dt * ang_acc;
    }

    const Vector3 &GetPosition() const override {
        return this->position_;
    }

    const Vector3 &GetRotation() const override {
        return this->theta_;
    }

    void SetRotation(Vector3 const &rotation) override {
        this->theta_ = rotation;
    }

    const Matrix3 &GetInertiaTensor() const override {
        return this->inertia_tensor_;
    }

    void SetInertiaTensor(Matrix3 const &inertia_tensor) override {
        this->inertia_tensor_ = inertia_tensor;
    }

    const Vector3 &GetAngularVelocity() const override {
        return this->omega_;
    }

    void SetAngularVelocity(Vector3 const &angular_velocity) override {
        this->omega_ = angular_velocity;
    }

    void ApplyTorque(Vector3 const &torque) override {
        this->total_torque_ += torque;
    }

    void ClearForces() override {
        this->total_torque_ = Vector3();
    }
};


class SolidSphere : public ParticleInterface, public RotorInterface, public CollisionInterface {
private:
    Particle com_;
    Particle com_prev_;

    UniformSphericalRotor rotor_;
    UniformSphericalRotor rotor_prev_;


    double radius_;

public:
    SolidSphere(double mass, double radius) : radius_{radius},
                                              com_(mass), com_prev_(mass), rotor_(radius, mass),
                                              rotor_prev_(radius, mass) {
    }

    double GetRadius() const {
        return this->radius_;
    }

    void SetRadius(double radius) {
        if (radius <= 0.0) {
            throw std::invalid_argument("Radius must be greater than zero");
        }
        this->radius_ = radius;
    }

    double GetMass() const override {
        return this->com_.GetMass();
    }

    void SetMass(double mass) override {
        this->com_.SetMass(mass);
    }

    void SetRotation(Vector3 const &rotation) override {
        this->rotor_.SetRotation(rotation);
    }

    void Revert() override {
        this->com_ = this->com_prev_;
        this->rotor_ = this->rotor_prev_;
    }

    void SetPosition(Vector3 const &position) override {
        this->com_.SetPosition(position);
    }

    void ApplyForce(Vector3 const &force) {
        this->com_.ApplyForce(force);
    }

    void ApplyTorque(Vector3 const &torque) {
        this->rotor_.ApplyTorque(torque);
    }

    void Update(double dt) {
        com_prev_ = com_;
        com_.Update(dt);

        rotor_prev_ = rotor_;
        rotor_.Update(dt);
    }

    ObjectType GetType() const {
        return ObjectType::SOLID_SPHERE;
    }

    const Vector3 &GetPosition() const {
        return com_.GetPosition();
    }


    const Vector3 &GetVelocity() const {
        return com_.GetVelocity();
    }

    void SetVelocity(Vector3 const &v) {
        com_.SetVelocity(v);
    }


    Vector3 const &GetRotation() const {
        return rotor_.GetRotation();
    }


    const Vector3 &GetAngularVelocity() const {
        return rotor_.GetAngularVelocity();
    }

    void SetAngularVelocity(Vector3 const &omega) {
        rotor_.SetAngularVelocity(omega);
    }

    const Matrix3 &GetInertiaTensor() const {
        return rotor_.GetInertiaTensor();
    }

    void SetInertiaTensor(Matrix3 const &I) {
        rotor_.SetInertiaTensor(I);
    }

    void ClearForces() {
        com_.ClearForces();
        rotor_.ClearForces();
    }
};


/**
 * Calculations for collisions
 */

Vector3 vs_out(Vector3 vs_this, Vector3 vs_other, double I1, double I2, double r1, double r2, double alpha) {
    return alpha * (I1 / r1 * vs_this + I2 / r2 * vs_other) / (I1 / r1 + I2 / r2) + (1.0 - alpha) * vs_this;
}

Vector3 omega_out(Vector3 vs_out, Vector3 v_out, Vector3 rhat, double r) {
    return Cross(vs_out - v_out, rhat) / r;
}

/**
 * Compute the velocities and angular velocities of two massive rigid spheres after a collision in a rotated
 * centre-of-momentum frame.  The rotation is such that the axis of the collision is in the (1, 0, 0) direction.
 *
 * @param mass1
 * @param mass2
 * @param inertia1
 * @param inertia2
 * @param radius1
 * @param radius2
 * @param v1_in
 * @param v2_in
 * @param omega1_in
 * @param omega2_in
 * @param v1_out
 * @param v2_out
 * @param omega1_out
 * @param omega2_out
 */
void ComputeCollisionSpheres(
        double mass1, double mass2, double inertia1, double inertia2, double radius1, double radius2,
        const Vector3 &v1_in, const Vector3 &v2_in, const Vector3 &omega1_in, const Vector3 &omega2_in,
        Vector3 &v1_out, Vector3 &v2_out, Vector3 &omega1_out, Vector3 &omega2_out
) {
    // Total angular momentum before the collision
    Vector3 rHatCrossL = -mass2 * (radius1 + radius2) * v2_in +
                         inertia1 * Cross(Vector3(1.0, 0.0, 0.0), omega1_in) +
                         inertia2 * Cross(Vector3(1.0, 0.0, 0.0), omega2_in);

    // Total kinetic energy before the collision
    double T = mass1 * Dot(v1_in, v1_in) / 2.0 +
               mass2 * Dot(v2_in, v2_in) / 2.0 +
               inertia1 * Dot(omega1_in, omega1_in) / 2.0 +
               inertia2 * Dot(omega2_in, omega2_in) / 2.0;

    // Surface velocities at the collision point just before the collision
    Vector3 vs1_in = v1_in + radius1 * Cross(Vector3(1, 0, 0), omega1_in);
    vs1_in.x = 0.0;
    Vector3 vs2_in = v2_in - radius2 * Cross(Vector3(1, 0, 0), omega2_in);
    vs2_in.x = 0.0;

    // Surface velocities at the collision point just after the collision
    Vector3 vs1_out = vs_out(vs1_in, vs2_in, inertia1, inertia2, radius1, radius2, 1.0);
    Vector3 vs2_out = vs_out(vs2_in, vs1_in, inertia1, inertia2, radius1, radius2, 1.0);

    double div = inertia1 / radius1 * mass2 / mass1 + inertia2 / radius2 - mass2 * (radius1 + radius2);
    Vector3 match = inertia2 / radius2 * vs2_out - inertia1 / radius1 * vs1_out;
    v2_out = (rHatCrossL + match) / div;
    v2_out.x = 0.0;
    v1_out = -mass2 / mass1 * v2_out;

    omega1_out = omega_out(vs1_out, v1_out, -1.0 * Vector3(1.0, 0.0, 0.0), radius1);
    omega1_out.x = omega1_in.x;
    omega2_out = omega_out(vs2_out, v2_out, Vector3(1.0, 0.0, 0.0), radius2);
    omega2_out.x = omega2_in.x;

    double T_out_ang = inertia1 * Dot(omega1_out, omega1_out) / 2.0 +
                       inertia2 * Dot(omega2_out, omega2_out) / 2.0;

    double T_out_perp = mass1 * Dot(v1_out, v1_out) / 2.0 +
                        mass2 * Dot(v2_out, v2_out) / 2.0;

    double T_out_par = T - T_out_ang - T_out_perp;

    v1_out.x = std::sqrt(2.0 * T_out_par * mass2 / (mass1 * (mass1 + mass2)));
    if (v1_in.x > 0.0) v1_out.x *= -1.0;
    v2_out.x = -mass1 / mass2 * v1_out.x;
}


struct TwoSpheres {
    Vector3 v1;
    Vector3 v2;
    Vector3 omega1;
    Vector3 omega2;
};


TwoSpheres CollisionBetweenTwoSpheres(double m1, double m2, double I1, double I2, double r1, double r2,
                                      Vector3 p1, Vector3 p2, Vector3 o1, Vector3 o2, Vector3 v1, Vector3 v2) {
    Vector3 v_com = (m1 * v1 + m2 * v2) / (m1 + m2);

    Vector3 r = p2 - p1;
    Vector3 r_hat = r / std::sqrt(Dot(r, r));
    // TODO: Be careful here: what if r_hat || (0, 0, 1) ?
    Vector3 s_hat = Cross(r_hat, Vector3{0, 0, 1});
    Vector3 t_hat = Cross(r_hat, s_hat);
    Matrix3 rot(r_hat, s_hat, t_hat);

    Vector3 v1_ = Dot(Transpose(rot), v1 - v_com);
    Vector3 v2_ = Dot(Transpose(rot), v2 - v_com);
    Vector3 o1_ = Dot(Transpose(rot), o1);
    Vector3 o2_ = Dot(Transpose(rot), o2);


    TwoSpheres out;
    ComputeCollisionSpheres(m1, m2, I1, I2, r1, r2, v1_, v2_, o1_, o2_, out.v1, out.v2, out.omega1, out.omega2);
    out.v1 = Dot(rot, out.v1) + v_com;
    out.v2 = Dot(rot, out.v2) + v_com;
    out.omega1 = Dot(rot, out.omega1);
    out.omega2 = Dot(rot, out.omega2);

    return out;
}


/**
 * Computes and applies the collision forces between two frictionless massive spheres.
 *
 * @param obj1
 * @param obj2
 * @param dt
 */

void CollideObjects(SolidSphere *obj1, SolidSphere *obj2, double dt) {
    double r1 = obj1->GetRadius();
    double r2 = obj2->GetRadius();

    Vector3 r = obj2->GetPosition() - obj1->GetPosition();

    if ((r1 + r2) * (r1 + r2) >= Dot(r, r)) {
        obj1->Revert();
        obj2->Revert();

        TwoSpheres out = CollisionBetweenTwoSpheres(
                obj1->GetMass(),
                obj2->GetMass(),
                obj1->GetInertiaTensor().xx,
                obj2->GetInertiaTensor().xx,
                obj1->GetRadius(),
                obj2->GetRadius(),
                obj1->GetPosition(),
                obj2->GetPosition(),
                obj1->GetAngularVelocity(),
                obj2->GetAngularVelocity(),
                obj1->GetVelocity(),
                obj2->GetVelocity()
        );

        obj1->SetVelocity(out.v1);
        obj2->SetVelocity(out.v2);
        obj1->SetAngularVelocity(out.omega1);
        obj2->SetAngularVelocity(out.omega2);
    }
}


class PhysicsEngine {
private:
    // Universal concepts
    std::chrono::time_point<std::chrono::steady_clock> time_;

    // All that exists
    std::vector<PhysicsObject *> particle_arr_;

public:
    PhysicsEngine() : time_(std::chrono::steady_clock::now()) {
    }

    void Update() {
        // Get time since last update
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::ratio<1, 1>> dt;
        dt = now - this->time_;
        this->time_ = now;

        // For each particle, simulate motion in the next time step
        // Basically use forces to update velocities
        for (PhysicsObject *particle : this->particle_arr_) {
            particle->Update(dt.count());
            particle->ClearForces();
        }

        // Detect collisions and update particle kinematics
        // Beware of applying any new forces in this step, as the time interval during which they will act will likely
        // not be the same in the next step.
        this->DetectCollisions(dt.count());
    }

    /**
     * Tests all distinct pairs of objects for collisions and updates kinematics as necessary.
     */
    void DetectCollisions(double dt) {
        if (this->particle_arr_.size() >= 2) {
            for (int i = 0; i < this->particle_arr_.size() - 1; ++i) {
                for (int j = i + 1; j < this->particle_arr_.size(); ++j) {
                    if (particle_arr_[i]->GetType() == ObjectType::BIG_PARTICLE &&
                        particle_arr_[j]->GetType() == ObjectType::BIG_PARTICLE) {
                        BigParticle *p1 = static_cast<BigParticle *>(particle_arr_[i]);
                        BigParticle *p2 = static_cast<BigParticle *>(particle_arr_[j]);
                        CollideObjects(p1, p2, dt);
                    }
                    if (particle_arr_[i]->GetType() == ObjectType::SOLID_SPHERE &&
                        particle_arr_[j]->GetType() == ObjectType::SOLID_SPHERE) {
                        SolidSphere *p1 = static_cast<SolidSphere *>(particle_arr_[i]);
                        SolidSphere *p2 = static_cast<SolidSphere *>(particle_arr_[j]);
                        CollideObjects(p1, p2, dt);
                    }
                }
            }
        }
    }

    void AddObject(PhysicsObject *p) {
        this->particle_arr_.push_back(p);
    }
};


#endif //PHYSICS_ENGINE_PHYSICS_H
