#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/msgs/twist.pb.h>
#include <map>

namespace ignition
{
namespace gazebo
{
struct JointState
{
    double position = 0.0;
    double velocity = 0.0;
    double targetVelocity = 0.0;
};

struct PrivateData
{
    bool configured = false;
    std::map<Entity, JointState> joints;
    double wheel_to_center = 0.0;
    double wheel_radius = 0.0;
};

class OmniwheelPlugin : public System,
                        public ISystemConfigure,
                        public ISystemPreUpdate
{
public:
    OmniwheelPlugin() : dataPtr(std::make_unique<PrivateData>()) {}

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element>&_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override
    {
        this->model = _entity;

        if (!_sdf->HasElement("wheeljoint1") || !_sdf->HasElement("wheeljoint2") ||
            !_sdf->HasElement("wheeljoint3") || !_sdf->HasElement("wheeljoint4") ||
            !_sdf->HasElement("wheel_to_center") || !_sdf->HasElement("wheel_radius"))
        {
            ignerr << "Missing required elements in SDF file for OmniwheelPlugin" << std::endl;
            return;
        }

        this->front_left_joint = _sdf->Get<std::string>("wheeljoint1");
        this->front_right_joint = _sdf->Get<std::string>("wheeljoint2");
        this->rear_left_joint = _sdf->Get<std::string>("wheeljoint3");
        this->rear_right_joint = _sdf->Get<std::string>("wheeljoint4");

        this->dataPtr->wheel_to_center = _sdf->Get<double>("wheel_to_center");
        this->dataPtr->wheel_radius = _sdf->Get<double>("wheel_radius");

        auto initJoint = [&](const std::string &jointName) {
            auto jointEntity = _ecm.EntityByComponents(components::Joint(), components::Name(jointName));
            if (jointEntity != kNullEntity)
                this->dataPtr->joints[jointEntity] = JointState();
            else
                ignerr << "Joint " << jointName << " not found" << std::endl;
        };

        initJoint(this->front_left_joint);
        initJoint(this->front_right_joint);
        initJoint(this->rear_left_joint);
        initJoint(this->rear_right_joint);

        if (this->dataPtr->joints.size() != 4) {
            ignerr << "Not all required joints were found. Plugin will not function correctly." << std::endl;
            return;
        }

        this->node = std::make_unique<transport::Node>();
        this->node->Subscribe("/cmd_vel", &OmniwheelPlugin::OnCmdVel, this);

        this->dataPtr->configured = true;
    }

    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) override 
    {
        if (!this->dataPtr->configured)
            return;

        const auto currentTime = _info.simTime;

        for (auto &joint : this->dataPtr->joints)
        {
            auto jointEntity = joint.first;
            auto &jointState = joint.second;
            if (jointEntity != kNullEntity)
            {
                auto jointPosComp = _ecm.Component<components::JointPosition>(jointEntity);
                if (jointPosComp != nullptr && !jointPosComp->Data().empty())
                    jointState.position = jointPosComp->Data()[0];

                auto jointVelComp = _ecm.Component<components::JointVelocity>(jointEntity);
                if (jointVelComp != nullptr && !jointVelComp->Data().empty())
                    jointState.velocity = jointVelComp->Data()[0];
            }
        }
        this->CalculateWheelVelocities(_ecm);
        for (const auto &joint : this->dataPtr->joints)
        {
            const auto jointEntity = joint.first;
            const auto &jointState = joint.second;

            if (jointEntity != kNullEntity)
            {
                auto velocityComp = _ecm.Component<components::JointVelocityCmd>(jointEntity);
                if (velocityComp == nullptr)
                {
                    auto newComp = _ecm.CreateComponent(jointEntity, components::JointVelocityCmd({jointState.targetVelocity}));
                    if (newComp != nullptr && !newComp->Data().empty())
                        newComp->Data()[0] = jointState.targetVelocity;
                }
                else
                {
                    if (!velocityComp->Data().empty())
                        velocityComp->Data()[0] = jointState.targetVelocity;
                }
            }
        }
    }

private:
    void OnCmdVel(const ignition::msgs::Twist &_msg)
    {
        this->cmdVel.X() = _msg.linear().x();
        this->cmdVel.Y() = _msg.linear().y();
        this->cmdVel.Z() = _msg.angular().z();
    }       

    void CalculateWheelVelocities(EntityComponentManager &_ecm)
    {
        double Vx = this->cmdVel.X();
        double Vy = this->cmdVel.Y();
        double Wz = this->cmdVel.Z();

        double L = this->dataPtr->wheel_to_center;
        double r = this->dataPtr->wheel_radius;

        for (auto &joint : this->dataPtr->joints)
        {
            const auto &jointName = _ecm.Component<components::Name>(joint.first)->Data();
            if (jointName == this->front_left_joint)
                joint.second.targetVelocity = (-Vx + Vy + L*Wz) / r;
            else if (jointName == this->front_right_joint)
                joint.second.targetVelocity = (Vx + Vy + L*Wz) / r;
            else if (jointName == this->rear_left_joint)
                joint.second.targetVelocity = (Vx - Vy + L*Wz) / r;
            else if (jointName == this->rear_right_joint)
                joint.second.targetVelocity = (-Vx -Vy + L*Wz) / r;
        }
    }

    Entity model{kNullEntity};
    std::unique_ptr<PrivateData> dataPtr;
    std::unique_ptr<transport::Node> node;
    math::Vector3d cmdVel;

    std::string front_left_joint;
    std::string front_right_joint;
    std::string rear_left_joint;
    std::string rear_right_joint;
};

using OmniwheelPluginPtr = std::shared_ptr<OmniwheelPlugin>;

}
}

IGNITION_ADD_PLUGIN(
    ignition::gazebo::OmniwheelPlugin,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::OmniwheelPlugin, "test_package::OmniwheelPlugin")

