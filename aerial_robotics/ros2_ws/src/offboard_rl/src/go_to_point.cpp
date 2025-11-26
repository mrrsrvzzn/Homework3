// go_to_viapoints_quintic.cpp

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <thread>
#include <Eigen/Dense>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rmw/qos_profiles.h>
#include "offboard_rl/utils.h"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct Waypoint {
    Eigen::Vector4d pos;    // x, y, z, yaw
    double T;               // in quanto tempo si deve percorrere il segmento
    double speed;           // velocità che desidero nel waypoint
};

class QuinticPlanner : public rclcpp::Node {
public:
    QuinticPlanner() : Node("go_to_viapoints_quintic"), 
                       waypoints_(N),
                       segments_(N)
    
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        local_pos_sub_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&QuinticPlanner::local_position_cb, this, std::placeholders::_1));

        attitude_sub_ = this->create_subscription<VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&QuinticPlanner::attitude_cb, this, std::placeholders::_1));

        offboard_mode_pub_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        vehicle_cmd_pub_   = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        traj_pub_          = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        timer_offboard_   = this->create_wall_timer(
            100ms, std::bind(&QuinticPlanner::enable_offboard_and_arm, this));
        timer_publish_    = this->create_wall_timer(
            20ms, std::bind(&QuinticPlanner::publish_loop, this));
        timer_trajectory_ = this->create_wall_timer(
            100ms, std::bind(&QuinticPlanner::generate_trajectory, this));
    }

private:
    // --- ROS interfaces
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr  local_pos_sub_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr       attitude_sub_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr      offboard_mode_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr           vehicle_cmd_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr       traj_pub_;
    rclcpp::TimerBase::SharedPtr                           timer_offboard_;
    rclcpp::TimerBase::SharedPtr                           timer_publish_;
    rclcpp::TimerBase::SharedPtr                           timer_trajectory_;
    

    // --- State
    VehicleLocalPosition current_position_{};
    VehicleAttitude      current_attitude_{};
    Eigen::Vector4d pos_i;
    bool                 offboard_active_{false};
    bool                 trajectory_ready_{false};
    int                  offboard_counter_{0};

    const size_t N = 8;
    std::vector<Waypoint> waypoints_;

    struct QuinticCoeffs {
        Eigen::Vector<double, 6> c;
        double T;
        Eigen::Vector4d start_pos;  // x, y, z, yaw start
        double cartesian_error_norm;
        double delta_yaw; 
        Eigen::Vector3d segment_direction_xyz;
    };

    std::vector<QuinticCoeffs> segments_; // ogni segmento è definito da posizione iniziale + finale (delta?) tempo di percorrenza e i coefficienti del polinomio
    double                     t_{0.0};
    size_t                     current_segment_idx_{0};
    double                     dt_{0.02};  // 50 Hz

    // --- Callbacks / helpers
    void local_position_cb(const VehicleLocalPosition::SharedPtr msg) {
        current_position_ = *msg;
    }

    void attitude_cb(const VehicleAttitude::SharedPtr msg) {
        current_attitude_ = *msg;
    }

    // 
    void enable_offboard_and_arm() {
        // Keep publishing OffboardControlMode
        OffboardControlMode offboard_msg{};
        offboard_msg.position     = true;
        offboard_msg.velocity     = false;
        offboard_msg.acceleration = false;
        offboard_msg.attitude     = false;
        offboard_msg.body_rate    = false;
        offboard_msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
        offboard_mode_pub_->publish(offboard_msg);
        

        // send mode change and arm sequence after a short warmup
        if (offboard_counter_ == 10) {
            VehicleCommand cmd{};
            cmd.command          = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
            cmd.param1           = 1;
            cmd.param2           = 6;
            cmd.target_system    = 1;
            cmd.target_component = 1;
            cmd.from_external    = true;
            cmd.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_cmd_pub_->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "Requested mode change to OFFBOARD.");

            VehicleCommand arm{};
            arm.command          = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            arm.param1           = 1;
            arm.target_system    = 1;
            arm.target_component = 1;
            arm.from_external    = true;
            arm.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_cmd_pub_->publish(arm);
            RCLCPP_INFO(this->get_logger(), "Requested ARM.");

            // Set initial position
            pos_i(0) = current_position_.x;
            pos_i(1) = current_position_.y;
            pos_i(2) = current_position_.z;

            auto rpy = utilities::quatToRpy( Vector4d( current_attitude_.q[0], current_attitude_.q[1], current_attitude_.q[2], current_attitude_.q[3] ) );
            pos_i(3) = rpy[2];

            RCLCPP_INFO(this->get_logger(),
                        "pos_i = [%.3f, %.3f, %.3f, yaw=%.3f rad]",
                        pos_i(0), pos_i(1), pos_i(2), pos_i(3));


            offboard_active_     = true;
            current_segment_idx_ = 0;
        }

        if (offboard_counter_ < 11) offboard_counter_++;
    }

    // funzione per calcolare i coefs di un polinomio dato un segmento di traiettoria che si vuole percorrere
    Eigen::Matrix<double, 6, 1> compute_quintic(double s0, double v0, double a0,
                                                double sf, double vf, double af, double T) {
        Eigen::Matrix<double, 6, 6> A;
        Eigen::Vector<double, 6> b;

        A <<  std::pow(T,5),  std::pow(T,4),  std::pow(T,3),  std::pow(T,2), T, 1,
              5*std::pow(T,4), 4*std::pow(T,3), 3*std::pow(T,2), 2*T,       1, 0,
              20*std::pow(T,3),12*std::pow(T,2),6*T,           1,           0, 0,
              0,              0,              0,              0,           0, 1,
              0,              0,              0,              0,           1, 0,
              0,              0,              0,              1,           0, 0;

        b << sf, vf, af, s0, v0, a0;

        Eigen::Vector<double, 6> coeffs = A.inverse() * b;
        return coeffs;
    }

    // funzione che ricava s, sdot, sddot da un polinomio
    void eval_quintic(const QuinticCoeffs& qc, double t, double& s, double& sdot, double& sddot) {
        if (t < 0)   t = 0;
        if (t > qc.T) t = qc.T;

        const auto& c = qc.c;

        double t1 = t;
        double t2 = t1 * t1;
        double t3 = t2 * t1;
        double t4 = t3 * t1;
        double t5 = t4 * t1;

        s     = c(0) * t5 + c(1) * t4 + c(2) * t3 + c(3) * t2 + c(4) * t1 + c(5);
        sdot  = 5 * c(0) * t4 + 4 * c(1) * t3 + 3 * c(2) * t2 + 2 * c(3) * t1 + c(4);
        sddot = 20 * c(0) * t3 + 12 * c(1) * t2 + 6 * c(2) * t1 + c(3);
    }


    // esecuzione della traiettoria
    void publish_loop() {
        // must have offboard active and user requested start
        if (!offboard_active_ || !trajectory_ready_) return;

        if (current_segment_idx_ >= segments_.size()) {
            const auto& last = waypoints_.back().pos;

            TrajectorySetpoint m{};
            m.position  = {float(last(0)), float(last(1)), float(last(2))};
            m.yaw       = float(last(3));
            m.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            traj_pub_->publish(m);
            return;
        }

        // copia il tratto di traj corrente da seguire e i suoi parametri
        const QuinticCoeffs& seg    = segments_[current_segment_idx_];
        Eigen::Vector3d dir         = seg.segment_direction_xyz;
        double dist                 = seg.cartesian_error_norm;
        Eigen::Vector4d p_i         = seg.start_pos;
        double d_yaw                = seg.delta_yaw;

        // calcola s, sdot, sddot
        double s, sdot, sddot;
        eval_quintic(seg, t_, s, sdot, sddot);

        bool pure_yaw = (dist < 1e-6);


        Eigen::Vector4d ref_pos;
        if (!pure_yaw) {
            RCLCPP_INFO(this->get_logger(),
                        "p_i(3) = %.4f rad,",
                        p_i(3));
            Eigen::Vector3d p = p_i.head<3>() + dir * s;
            double yaw = p_i(3) + (d_yaw * (s / std::max(dist, 1e-12)));
            RCLCPP_INFO(this->get_logger(),
                        "yaw computed by the formula = %.4f rad,",
                        yaw);

            while (yaw > M_PI)  yaw -= 2 * M_PI;
            while (yaw < -M_PI) yaw += 2 * M_PI;

            ref_pos << p(0), p(1), p(2), yaw;
        } else {
            double yaw = p_i(3) + ((d_yaw >= 0 ? 1.0 : -1.0) * s);

            while (yaw > M_PI)  yaw -= 2 * M_PI;
            while (yaw < -M_PI) yaw += 2 * M_PI;

            ref_pos << p_i(0), p_i(1), p_i(2), yaw;
        }

        RCLCPP_INFO(this->get_logger(),
            "[SETPOINT] yaw raw = %.4f rad,",
            ref_pos(3));

        Eigen::Vector3d vel3 = Eigen::Vector3d::Zero();
        if (!pure_yaw) vel3 = dir * sdot;

        Eigen::Vector3d acc3 = Eigen::Vector3d::Zero();
        if (!pure_yaw) acc3 = dir * sddot;

        TrajectorySetpoint msg{};
        msg.position     = {float(ref_pos(0)), float(ref_pos(1)), float(ref_pos(2))};
        msg.velocity     = {float(vel3(0)), float(vel3(1)), float(vel3(2))};
        msg.acceleration = {float(acc3(0)), float(acc3(1)), float(acc3(2))};
        msg.yaw          = float(ref_pos(3));
        msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
        traj_pub_->publish(msg);

        
        t_ += dt_;

        if (t_ >= seg.T - 1e-6) {
            // aggiorno indice e resetto il tempo per passare al segmento sccessivo
            current_segment_idx_++;
            t_ = 0.0;

            if (current_segment_idx_ < segments_.size()) {
                RCLCPP_INFO(this->get_logger(),
                            "Advancing to segment %zu / %zu",
                            current_segment_idx_ + 1, segments_.size());
            } else {
                RCLCPP_INFO(this->get_logger(), "Trajectory finished.");
            }
        }

        return;
        
    }

    void generate_trajectory() {      
        if (!offboard_active_  || trajectory_ready_){
            RCLCPP_INFO(this->get_logger(),
                        "Waiting for offboard activation… pos_i = [%.3f %.3f %.３f yaw=%.3f]",
                        pos_i(0), pos_i(1), pos_i(2), pos_i(3));
            return;
        }

        // tempo di esecuzione e velocità
        double segment_time = 4.0;

        // punti del percorso
        std::vector<Eigen::Vector3d> vertices = {
            {pos_i(0),      pos_i(1),      pos_i(2)},
            {pos_i(0) + 1,  pos_i(1) + 2,  pos_i(2) - 2},
            {pos_i(0) + 2,  pos_i(1),      pos_i(2) - 4},
            {pos_i(0),      pos_i(1),      pos_i(2) - 6},
            {pos_i(0) + 1,  pos_i(1) + 2,  pos_i(2) - 8},
            {pos_i(0) + 2,  pos_i(1),      pos_i(2) - 10},
            {pos_i(0),      pos_i(1),      pos_i(2) - 12},
            {pos_i(0) + 1,  pos_i(1) + 2,  pos_i(2) - 14},
            {pos_i(0) + 2,  pos_i(1),      pos_i(2) - 16}
         };

        // calcolo yaw e velocità per ogni segmento
        std::vector<double> yaws(N+1);
        std::vector<double> waypoint_speeds_(N);
        std::vector<double> seg_len(N);
        std::vector<double> seg_speed(N); 

        // yaws
        yaws[0] = pos_i(3); // il primo valore di yaw è letto dai sensori
        for (size_t i = 1; i < N + 1; i++) {
            if (i < N) {
                double dx = vertices[i+1].x() - vertices[i].x();
                double dy = vertices[i+1].y() - vertices[i].y();
                yaws[i] = std::atan2(dy, dx);       // rad
                double gen_yaw = yaws[i];
                RCLCPP_INFO(this->get_logger(),
                    "yaw generated: %.4f",
                    gen_yaw);
            } else {
                // ultimo waypoint: mantieni stessa direzione dell'ultimo segmento
                double dx = vertices[i].x() - vertices[i-1].x();
                double dy = vertices[i].y() - vertices[i-1].y();
                yaws[i] = std::atan2(dy, dx);
            }
        }

        // velocità: assegno ai segmenti
        for (size_t i = 0; i < N - 1; i++) {
            Eigen::Vector3d p0 = vertices[i+1];
            Eigen::Vector3d p1 = vertices[i+2];
            double d = (p1 - p0).norm();
            seg_len[i] = d;

            double s = std::max(1e-6, d / std::max(segment_time, 1e-3));  // avoid zero division
            seg_speed[i] = s;
        }

        // velocità media tra quelle dei segmenti per ogni waypoint
        waypoint_speeds_[N-1] = 0.0;
        for (size_t k = 0; k < N - 1; k++) {
            waypoint_speeds_[k] = 0.5 * (seg_speed[k - 1] + seg_speed[k]);
            if (waypoint_speeds_[k] < 1e-4) waypoint_speeds_[k] = 0.1;
        }

        // aggiungo waypoints al vettore principale
        for (size_t i = 0; i < N; i++) {
            Eigen::Vector4d p;
            p << vertices[i + 1].x(), vertices[i + 1].y(), vertices[i + 1].z(), yaws[i + 1];

            if(i == 0){
                waypoints_[i] = {p, segment_time, waypoint_speeds_[i]};
            }else{
                waypoints_[i] = {p, segment_time, waypoint_speeds_[i]};
            }
        }

        RCLCPP_INFO(this->get_logger(),
                    "Generated %zu waypoints (auto yaw).",
                    waypoints_.size());

        // GENERO I SEGMENTI
        for (size_t i = 0; i < N; i++) {
            QuinticCoeffs qc;

            double T = waypoints_[i].T;
            qc.T = T;

            qc.start_pos << vertices[i].x(), vertices[i].y(), vertices[i].z(), yaws[i];
            RCLCPP_INFO(this->get_logger(),
                        "segment starting position coordinates = [%.4f, %.4f, %.4f, %.4f]",
                        qc.start_pos(0), qc.start_pos(1), qc.start_pos(2), qc.start_pos(3));


            Eigen::Vector4d end;
            end << vertices[i + 1].x(), vertices[i + 1].y(), vertices[i + 1].z(), yaws[i + 1];


            Eigen::Vector4d delta = end - qc.start_pos;
            
            double yawd = yaws[i+1] - yaws[i];
            while (yawd > M_PI)  yawd -= 2 * M_PI;
            while (yawd < -M_PI) yawd += 2 * M_PI;

            delta(3)  = yawd;

            double s0 = 0.0;
            double sf = delta.head<3>().norm(); // sf = lunghezza del segmento. Se pure_yaw=true, sf è la variazione di yaw

            bool pure_yaw = false;
            if (sf < 1e-6) {
                pure_yaw = true;
                sf = std::fabs(delta(3)); 
            }

            if (!pure_yaw){
                qc.cartesian_error_norm     = sf;    
                qc.segment_direction_xyz    = delta.head<3>() / std::max(sf, 1e-12);
                qc.delta_yaw                = delta(3);
            }else{
                qc.cartesian_error_norm     = 0.0;    
                qc.segment_direction_xyz    = Eigen::Vector3d::Zero();
                qc.delta_yaw                = delta(3);
            }

            double v0, vf;
            if(i==0){
                v0 = 0;
                vf = waypoint_speeds_[i];
            }else{
                v0 = waypoint_speeds_[i - 1];
                vf = waypoint_speeds_[i];             
            }

            double a0 = 0.0;
            double af = 0.0;

            Eigen::Vector<double, 6> coeffs =
                compute_quintic(s0, v0, a0, sf, vf, af, std::max(1e-6, T));

            qc.c = coeffs;
            segments_[i] = qc;    // ho calcolato i polinomi quintici per ogni segmento
        }

        RCLCPP_INFO(this->get_logger(),
                    "Prepared %zu segments (waypoints %zu).",
                    segments_.size(), waypoints_.size());

        trajectory_ready_= true;

    }
}; // end class

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QuinticPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
