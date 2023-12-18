//ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_kdl/tf2_kdl.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <kdl/frames.hpp>

// Sigma
#include <sigma7/dhdc.h>
#include <sigma7/drdc.h>
#include <sensor_msgs/msg/joy.hpp>

#include <sstream>

template<typename T>
constexpr const T& clamp(const T& a_val, const T& a_minVal, const T& a_maxVal)
{
    return (a_val < a_minVal) ? a_minVal : (a_maxVal < a_val) ? a_maxVal : a_val;
}

int moveToCenter(double a_moveBase,
                 double a_moveWrist,
                 double a_moveGripper)
{
    // Enable/disable base regulation.
    if (drdRegulatePos(a_moveBase) < 0)
    {
        std::cout << "error: failed to set base regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Enable/disable wrist regulation.
    if (dhdHasActiveWrist() && drdRegulateRot(a_moveWrist) < 0)
    {
        std::cout << "error: failed to set wrist regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Enable/disable gripper regulation.
    if (dhdHasActiveGripper() && drdRegulateGrip(a_moveGripper) < 0)
    {
        std::cout << "error: failed to set gripper regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Start robotic regulation.
    if (drdStart() < 0)
    {
        std::cout << "error: failed to start robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Move to the center of the workspace.
    double positionCenter[DHD_MAX_DOF] = {};
    if (drdMoveTo(positionCenter, true) < 0)
    {
        std::cout << "error: failed to move the device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Stop the regulation thread but leaves the forces enabled on the device.
    if (drdStop(true) < 0)
    {
        std::cout << "error: failed to stop robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    return 0;
}



void CheckAvailableDevices(int &devs) {

    while(rclcpp::ok() && devs==0) {
        for (int i = 0; i < 2; i++) {
            if (drdOpenID((char) i) > -1)
                devs = i+1;
        }
        rclcpp::Rate r(0.5);
        r.sleep();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Looking for connected devices...");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Found %i Device(s)", devs);
};

class SigmaDevice{
    public:
        SigmaDevice(std::shared_ptr<rclcpp::Node> node, const std::string name_space){
            id++;
            node_ = node;
            // setup the publishers and the subscriber
            pub_pose = node_->create_publisher<geometry_msgs::msg::PoseStamped>(name_space + "/pose", 10);
            pub_twist = node_->create_publisher<geometry_msgs::msg::TwistStamped>(name_space + "/twist", 10);
            pub_gripper = node_->create_publisher<std_msgs::msg::Float32>(name_space + "/gripper_angle", 10);
            pub_buttons = node_->create_publisher<sensor_msgs::msg::Joy>(name_space + "/buttons", 10);
            pub_pose_delta = node_->create_publisher<geometry_msgs::msg::PoseStamped>(name_space + "/pose_delta", 10);
            
            std::string wrench_topic("/sigma/force_feedback");
            node_->get_parameter("wrench_topic", wrench_topic);

            sub_wrench = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(wrench_topic,10,
                                      std::bind(&SigmaDevice::WrenchCallback,this,std::placeholders::_1));
            // params
            bool enable_gripper_button = 0;
            bool lock_orient = 1;
            node_->get_parameter("enable_gripper_button", enable_gripper_button);
            node_->get_parameter("lock_orientation", lock_orient);

            // calibrate the devices
            if(CalibrateDevice() == -1)
                rclcpp::shutdown();
            
            // move to center;
            moveToCenter(true, true, true);

            buttons_msg.buttons.push_back(0);
            buttons_msg.buttons.push_back(0);

            double p[7];
            dhdGetPositionAndOrientationRad(&p[0],&p[1],&p[2],&p[3],&p[4],&p[5]);
            // Initilize current pose
            current_pose_msg.pose.position.x = p[0];
            current_pose_msg.pose.position.y = p[1];
            current_pose_msg.pose.position.z = p[2];
            tf2::Quaternion q;
            q.setRPY(p[3]+M_PI, p[4], p[5]);
            current_pose_msg.pose.orientation.w = q.getW();
            current_pose_msg.pose.orientation.x = q.getX();
            current_pose_msg.pose.orientation.y = q.getY();
            current_pose_msg.pose.orientation.z = q.getZ();
            dhdGetGripperAngleDeg(&p[6]);
            gripper_angle.data = p[6];
        }

        int ReadMeasurementsFromDevice()
        {

            double px = 0;
            double py = 0;
            double pz = 0;
            double ra = 0;
            double rb = 0;
            double rg = 0;
            double pg = 0;
            dhdGetPositionAndOrientationRad(&px, &py, &pz, &ra, &rb, &rg);
            current_pose_msg.header.stamp = node_->get_clock()->now();

            tf2::Quaternion q;
            q.setRPY(M_PI + ra, rb, rg);
            // pose delta
            current_pose_msg.pose.position.x = px;
            current_pose_msg.pose.position.y = py;
            current_pose_msg.pose.position.z = pz;
            current_pose_msg.pose.orientation.w = q.getW();
            current_pose_msg.pose.orientation.x = q.getX();
            current_pose_msg.pose.orientation.y = q.getY();
            current_pose_msg.pose.orientation.z = q.getZ();

            // -----------------------------
            // Twist
            double v[6];

            dhdGetLinearVelocity(&v[0], &v[1], &v[2], (char)id);
            dhdGetAngularVelocityRad(&v[3], &v[4], &v[5], (char)id);
            // convert to twist message
            current_twist_msg.twist.linear.x = v[0];
            current_twist_msg.twist.linear.y = v[1];
            current_twist_msg.twist.linear.z = v[2];
            current_twist_msg.twist.angular.x = v[3];
            current_twist_msg.twist.angular.y = v[4];
            current_twist_msg.twist.angular.z = v[5];
            // stamp the msg
            current_twist_msg.header.stamp = node_->get_clock()->now();

            // ------------------------------
            // gripper
            double temp;
            dhdGetGripperAngleRad(&temp);
            gripper_angle.data = (float)temp;
            // ------------------------------
            // buttons
            // saving the previous states of gripper button and pedal
            for (int i = 0; i < 2; ++i)
            {
                buttons_previous_state[i] = buttons_state[i];
                buttons_state[i] = dhdGetButton(i, (char)id);
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "Angular measured: (r: %.2f, p: %.2f, r: %.2f, g: %.2f)",
                        static_cast<float>(ra), static_cast<float>(rb), static_cast<float>(rg), gripper_angle.data);

            return 0;
        }

        void PublishPoseTwistButtonPedal()
        {

            pub_pose->publish(current_pose_msg);
            pub_twist->publish(current_twist_msg);
            // pose delta
            // pub_pose_delta->publish(pose_delta_msg);

            // Publish gripper angle
            pub_gripper->publish(gripper_angle);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "Position published: (x: %.2f, y: %.2f, z: %.2f)",
                        current_pose_msg.pose.position.x,
                        current_pose_msg.pose.position.y,
                        current_pose_msg.pose.position.z);

            // publish buttons when there is a change
            if ((buttons_state[0] != buttons_previous_state[0]) ||
                (buttons_state[1] != buttons_previous_state[1]))
            {

                // populate the message
                buttons_msg.buttons[0] = buttons_state[0];
                buttons_msg.buttons[1] = buttons_state[1];
                // publish it
                pub_buttons->publish(buttons_msg);
            }
        }

    private:
        // CalibrateDevice
        int CalibrateDevice()
        {

            RCLCPP_INFO(node_->get_logger(), "Calibrating device %i ...", id);

            // open device
            if (drdOpenID((char)id) < 0)
            {
                RCLCPP_ERROR(node_->get_logger(), "No device %i found. dhd says: %s", id, dhdErrorGetLastStr());
                dhdSleep(2.0);
                drdClose((char)id);
                return -1;
            }

            // Calibrate the device if it is not already calibrated;
            if (drdIsInitialized((char)id))
            {
                RCLCPP_INFO(node_->get_logger(), "Device %i is already calibrated.", id);
            }
            else if (drdAutoInit((char)id) < 0)
            {
                RCLCPP_ERROR(node_->get_logger(), "Initialization of device %i failed. dhd says: (%s)", id,
                             dhdErrorGetLastStr());
                dhdSleep(2.0);
            }

            // center of workspace
            double nullPose[DHD_MAX_DOF] = {0.0, 0.0, 0.0, // base  (translations)
                                            0.0, 0.0, 0.0, // wrist (rotations)
                                            0.0};          // gripper
            // move to center
            drdMoveTo(nullPose);

            // stop regulation (and leave force enabled)
            drdStop(true, (char)id);

            // enable force
            dhdEnableForce(DHD_ON, (char)id);

            //    dhdSetGravityCompensation(DHD_ON, (char)id);
            dhdSleep(5);
            // Enable the gripper button
            if (enable_gripper_button)
                dhdEmulateButton(DHD_ON, (char)id);

            RCLCPP_INFO(node_->get_logger(), "Device %i ready.", id);
            return 0;
        }

        // WrenchCallback
        void WrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        {
            // newDataDirect = true; // If needed in ROS2 context
            current_wrench.wrench = msg->wrench;
            new_wrench_msg = true;
            // Additional processing can be added here
        }

    private:
        std::shared_ptr<rclcpp::Node> node_;

        int id = -1;
        bool enable_gripper_button = 0;
        bool lock_orient = 0;
        double positionCenter[DHD_MAX_DOF] = {};

        geometry_msgs::msg::PoseStamped current_pose_msg; // position
        geometry_msgs::msg::TwistStamped current_twist_msg; // velocity
        geometry_msgs::msg::WrenchStamped current_wrench; // force

        bool new_wrench_msg;
        std_msgs::msg::Float32 gripper_angle; // [0,PI/6] [0,30]

        // the gripper button and pedal state and their previous state
        int buttons_state[2];
        int buttons_previous_state[2];

        sensor_msgs::msg::Joy buttons_msg; // two elements, 0 is gripper button, 1
        // is pedal
        int pedal_previous_state;

        // publishers and subscribers
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_gripper;
        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_buttons;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_delta;

};

int holdOnCenter(SigmaDevice* sigma_device){
    // Enable expert mode to allow wrist joint torques control.
    dhdEnableExpertMode();
    // Move to the center of the workspace.
    double positionCenter[DHD_MAX_DOF] = {};
    drdMoveTo(positionCenter);

    // Display user instructions.
    std::cout << "press 'q' to quit" << std::endl << std::endl;


    // Allocate and initialize display control variables.
    double lastDisplayUpdateTime = dhdGetTime();

    // Allocate and initialize user button state.
    bool previousUserButton = false;

    // Allocate and initialize device position (base, wrist and gripper).
    double px = 0;
    double py = 0;
    double pz = 0;
    double ra = 0;
    double rb = 0;
    double rg = 0;
    double pg = 0;

    // Allocate and initialize device velocity (base, wrist and gripper).
    double jointAnglesVelocity[DHD_MAX_DOF] = {};
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double wa = 0.0;
    double wb = 0.0;
    double wg = 0.0;
    double vg = 0.0;

    // Allocate and initialize device holding position (base, wrist and gripper).
    double pxHold = 0;
    double pyHold = 0;
    double pzHold = 0;
    double raHold = 0;
    double rbHold = 0;
    double rgHold = 0;
    double pgHold = 0;

    // Allocate and initialize device force and torque (base, wrist and gripper).
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    double qa = 0.0;
    double qb = 0.0;
    double qg = 0.0;
    double fg = 0.0;

    // Run the haptic loop.
    while (true)
    {
        double p[7];
        dhdGetPositionAndOrientationRad(&p[0],&p[1],&p[2],&p[3],&p[4],&p[5]);
        sigma_device->ReadMeasurementsFromDevice();
        sigma_device->PublishPoseTwistButtonPedal();
        // std::cout << p[0] << ", " << p[1] << ", " << p[2] << std::endl;

        // Retrieve base, wrist and gripper position.
        if (dhdGetPositionAndOrientationDeg(&px, &py, &pz, &ra, &rb, &rg) < 0)
        {
            std::cout << "error: failed to retrieve device position (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }
        if (dhdGetGripperGap(&pg) < 0)
        {
            std::cout << "error: failed to retrieve device gripper opening (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Retrieve the base an gripper velocity.
        if (dhdGetLinearVelocity(&vx, &vy, &vz) < 0)
        {
            std::cout << "error: failed to retrieve linear velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }
        if (dhdGetGripperLinearVelocity(&vg) < 0)
        {
            std::cout << "error: failed to retrieve gripper velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Retrieve the wrist joint angles velocity.
        if (dhdGetJointVelocities(jointAnglesVelocity) < 0)
        {
            std::cout << "error: failed to retrieve angular velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }
        wa = jointAnglesVelocity[3];
        wb = jointAnglesVelocity[4];
        wg = jointAnglesVelocity[5];

        // Detect user button events.
        bool userButton = (dhdGetButton(0) != 0);
        bool userButtonPressed = (userButton && !previousUserButton);
        bool userButtonReleased = (!userButton && previousUserButton);
        previousUserButton = userButton;

        // When the button is released, store the current position as a holding target.
        if (userButtonReleased)
        {
            // Store base position.
            pxHold = px;
            pyHold = py;
            pzHold = pz;

            // Store wrist joint angles.
            raHold = ra;
            rbHold = rb;
            rgHold = rg;

            // Store gripper opening.
            pgHold = pg;
        }

        // While the button is pressed, let the device move freely.
        if (userButton)
        {
            fx = 0.0;
            fy = 0.0;
            fz = 0.0;
            qa = 0.0;
            qb = 0.0;
            qg = 0.0;
            fg = 0.0;
        }

        // If the button is not pressed, apply a light spring to hold the device in place.
        else
        {
            // Base position spring stiffness in [N/m]
            constexpr double Kp = 100.0;

            // Base position spring damping in [N/(m/s)]
            constexpr double Kv = 20.0;

            // Base position maximum force in [N]
            constexpr double MaxForce = 2.0;

            // Wrist joint angular spring stiffness in [Nm/deg]
            constexpr double Kr = 0.01;

            // Wrist joint angular spring damping in [Nm/(deg/s)]
            constexpr double Kw = 0.04;

            // Wrist joint maximum torque in [Nm]
            constexpr double MaxTorque = 0.02;

            // Gripper spring stiffness in [N/m]
            constexpr double Kgp = 100.0;

            // Gripper spring damping in [N/(m/s)]
            constexpr double Kgv = 5.0;

            // Gripper maximum force in [N]
            constexpr double MaxGripperForce = 1.0;

            // Compute base position spring.
            fx = Kp * (pxHold - px) - Kv * vx;
            fy = Kp * (pyHold - py) - Kv * vy;
            fz = Kp * (pzHold - pz) - Kv * vz;

            // Make sure the Cartesian maximum force is not exceeded.
            double forceMagnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
            if (forceMagnitude > MaxForce)
            {
                double forceRatio = MaxForce / forceMagnitude;
                fx *= forceRatio;
                fy *= forceRatio;
                fz *= forceRatio;
            }

            // Compute wrist joint springs, and limit each joint torque to the set maximum.
            qa = clamp(Kr * (raHold - ra) - Kw * wa, -MaxTorque, MaxTorque);
            qb = clamp(Kr * (rbHold - rb) - Kw * wb, -MaxTorque, MaxTorque);
            qg = clamp(Kr * (rgHold - rg) - Kw * wg, -MaxTorque, MaxTorque);

            // Compute gripper position spring, and limit it to its set maximum.
            fg = clamp(Kgp * (pgHold - pg) - Kgv * vg, -MaxGripperForce, MaxGripperForce);
        }

        // Apply the required forces and torques.
        if (dhdSetForceAndWristJointTorquesAndGripperForce(fx, fy, fz, qa, qb, qg, fg) < 0)
        {
            std::cout << "error: failed to apply forces (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        // Periodically display the haptic device status and flush stdout.
        double time = dhdGetTime();
        if (time - lastDisplayUpdateTime > 0.1)
        {
            lastDisplayUpdateTime = time;       
            
            if (userButton)
            {
                std::cout << "current status: in use   \r";
            }
            else
            {
                std::cout << "current status: holding  \r";
            }
            std::cout.flush();
        }

        // Process user input.
        if (dhdKbHit() && dhdKbGet() == 'q')
        {
            std::cout << std::endl << std::endl << "exiting at user's request" << std::endl;
            break;
        }
    }

    // Close the connection to the haptic device.
    if (drdClose() < 0)
    {
        std::cout << "error: failed to close the connection (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }
    return 0;
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sigma7");

    if (drdOpen() < 0)
    {
        std::cout << "error: failed to open device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }


    // Locking call looking for connected devices.
    // devs is the number of available devices
    int devs= 0;
    CheckAvailableDevices(devs);

    // Initialize devices
    std::stringstream dev_names;
    dev_names << "sigma"<< 1;
    SigmaDevice* sigma = new SigmaDevice(node, dev_names.str());


    // get the frequency parameter
    double rate;
    node->declare_parameter<double>("frequency", 10.0);
    node->get_parameter("frequency", rate);
    //node->set_parameter("frequency", rate);
    RCLCPP_INFO(node->get_logger(),"Set frequency: %f", rate);
    std::shared_ptr<rclcpp::Rate> loop_rate;
    loop_rate = std::make_shared<rclcpp::Rate>(rate);

    holdOnCenter(sigma);

    RCLCPP_INFO(node->get_logger(),"Initialization done.");

    while (rclcpp::ok()) {}

    rclcpp::shutdown();
    return 0;
}







