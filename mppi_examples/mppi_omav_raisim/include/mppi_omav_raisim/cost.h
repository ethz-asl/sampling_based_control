/*!
 * @file     cost.h
 * @author   Matthias Studiger
 * @date     22.03.2021
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <math.h>
#include <mppi/cost/cost_base.h>
#include <ros/ros.h>

#include <ros/package.h>

namespace omav_raisim {

    struct OMAVRaisimCostParam {
        double Q_distance_x; // Distance to reference Cost
        double Q_distance_y;
        double Q_distance_z;

        double Q_orientation; // Orientation Cost

        double Q_leafing_field; // Leafing Field Costs
        double x_limit;
        double y_limit;
        double z_limit;

        double Q_obstacle; // Obstacle Cost
        double obstacle_x;
        double obstacle_y;
        double obstacle_radius;

        double Q_velocity_max; // Maximum velocity cost
        double max_velocity;

        double Q_thrust_max; // Maximum thrust cost
        double max_thrust;

        double Q_omega;
        double max_omega;

        bool parse_from_ros(const ros::NodeHandle &nh);
    };

    class OMAVRaisimCost : public mppi::CostBase {
    public:
        OMAVRaisimCost() : OMAVRaisimCost("", OMAVRaisimCostParam()){};
        OMAVRaisimCost(const std::string& robot_description, const OMAVRaisimCostParam& param);
        ~OMAVRaisimCost() = default;

    private:
        std::string robot_description_;
        OMAVRaisimCostParam param_;

    private:
        cost_ptr create() override {
            return std::make_shared<OMAVRaisimCost>(robot_description_, param_);
        }
        cost_ptr clone() const override {return std::make_shared<OMAVRaisimCost>(*this);}

        cost_t compute_cost(const mppi::observation_t& x,
                            const mppi::reference_t& ref, const double t) override;

    };

} // namespace omav_raisim

std::ostream& operator<<(std::ostream& os, const omav_raisim::OMAVRaisimCostParam& param);

