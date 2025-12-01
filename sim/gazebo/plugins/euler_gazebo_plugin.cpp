/**
 * Euler Swarm Gazebo Plugin
 * Bridges Gazebo simulation with Euler swarm coordination library.
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>

extern "C" {
#include "euler_comm.h"
#include "euler_control.h"
#include "euler_types.h"
}

namespace gazebo {

class EulerSwarmPlugin : public ModelPlugin {
public:
    EulerSwarmPlugin() : drone_id_(0), port_(15560), update_rate_(10.0) {}
    
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
        model_ = model;
        world_ = model->GetWorld();
        
        if (sdf->HasElement("drone_id")) {
            drone_id_ = static_cast<uint8_t>(sdf->Get<int>("drone_id"));
        }
        if (sdf->HasElement("broadcast_port")) {
            port_ = static_cast<uint16_t>(sdf->Get<int>("broadcast_port"));
        }
        if (sdf->HasElement("update_rate")) {
            update_rate_ = sdf->Get<double>("update_rate");
        }
        
        if (euler_comm_init(&comm_ctx_, port_) != COMM_OK) {
            gzerr << "Failed to initialize Euler comm for drone " << (int)drone_id_ << "\n";
            return;
        }
        
        euler_comm_start_receiver(&comm_ctx_);
        euler_control_init(&control_ctx_);
        
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&EulerSwarmPlugin::OnUpdate, this));
        
        last_broadcast_ = world_->SimTime();
        
        gzmsg << "Euler plugin loaded for drone " << (int)drone_id_ 
              << " on port " << port_ << "\n";
    }
    
    void OnUpdate() {
        common::Time current = world_->SimTime();
        double dt = (current - last_broadcast_).Double();
        
        if (dt < 1.0 / update_rate_) return;
        last_broadcast_ = current;
        
        ignition::math::Pose3d pose = model_->WorldPose();
        ignition::math::Vector3d vel = model_->WorldLinearVel();
        
        DroneState state = {};
        state.drone_id = drone_id_;
        state.status = STATUS_ARMED | STATUS_FLYING;
        state.battery_pct = 85;
        state.pos_x = static_cast<float>(pose.Pos().X());
        state.pos_y = static_cast<float>(pose.Pos().Y());
        state.pos_z = static_cast<float>(pose.Pos().Z());
        state.vel_x = static_cast<float>(vel.X());
        state.vel_y = static_cast<float>(vel.Y());
        state.vel_z = static_cast<float>(vel.Z());
        state.sequence = seq_++;
        
        euler_comm_broadcast(&comm_ctx_, &state);
        
        DroneState neighbor;
        while (euler_comm_pop_neighbor(&comm_ctx_, &neighbor)) {
            if (neighbor.drone_id != drone_id_) {
                euler_neighbor_update(&swarm_ctx_, &neighbor, 
                    static_cast<uint32_t>(current.Double() * 1000));
            }
        }
        
        Vec3 my_pos = {state.pos_x, state.pos_y, state.pos_z};
        Vec3 avoidance = euler_collision_avoidance(&swarm_ctx_, my_pos);
        
        if (vec3_length(avoidance) > 0.1f) {
            physics::LinkPtr link = model_->GetLink("base_link");
            if (link) {
                link->AddForce(ignition::math::Vector3d(
                    avoidance.x * 10.0, avoidance.y * 10.0, avoidance.z * 10.0));
            }
        }
    }
    
    ~EulerSwarmPlugin() {
        euler_comm_stop_receiver(&comm_ctx_);
        euler_comm_shutdown(&comm_ctx_);
    }

private:
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    event::ConnectionPtr update_connection_;
    
    CommContext comm_ctx_;
    ControlContext control_ctx_;
    SwarmContext swarm_ctx_;
    
    uint8_t drone_id_;
    uint16_t port_;
    double update_rate_;
    uint16_t seq_ = 0;
    common::Time last_broadcast_;
};

GZ_REGISTER_MODEL_PLUGIN(EulerSwarmPlugin)

}

