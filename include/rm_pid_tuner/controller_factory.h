/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024
 * All rights reserved.
 *******************************************************************************/

#pragma once

#include <string>
#include <memory>
#include <functional>
#include <map>

namespace rm_pid_tuner
{
// 前向声明
struct JointConfig;
struct TuningConfig;

/**
 * @brief 控制器类型枚举
 */
enum class ControllerType
{
  GIMBAL,
  CHASSIS,
  SHOOTER,
  CUSTOM
};

/**
 * @brief 控制器配置接口
 */
struct IControllerConfig
{
  virtual ~IControllerConfig() = default;

  virtual ControllerType getType() const = 0;
  virtual std::string getTypeName() const = 0;

  // 获取参数前缀
  virtual std::string getParamPrefix(const std::string& controller_name,
                                      const std::string& joint_name) const = 0;

  // 获取话题名称
  virtual std::string getPosStateTopic(const std::string& controller_name) const = 0;
  virtual std::string getErrorTopic(const std::string& controller_name) const = 0;

  // 获取支持的关节列表
  virtual std::vector<std::string> getSupportedJoints() const = 0;

  // 克隆
  virtual std::unique_ptr<IControllerConfig> clone() const = 0;
};

/**
 * @brief 云台控制器配置
 */
class GimbalControllerConfig : public IControllerConfig
{
public:
  ControllerType getType() const override { return ControllerType::GIMBAL; }
  std::string getTypeName() const override { return "gimbal"; }

  std::string getParamPrefix(const std::string& controller_name,
                            const std::string& joint_name) const override
  {
    return "/" + controller_name + "/controllers/" + joint_name + "/pid_pos";
  }

  std::string getPosStateTopic(const std::string& controller_name) const override
  {
    return "/" + controller_name + "/pos_state";
  }

  std::string getErrorTopic(const std::string& controller_name) const override
  {
    return "/" + controller_name + "/error";
  }

  std::vector<std::string> getSupportedJoints() const override
  {
    return {"yaw", "pitch", "yaw_joint", "pitch_joint"};
  }

  std::unique_ptr<IControllerConfig> clone() const override
  {
    return std::make_unique<GimbalControllerConfig>(*this);
  }
};

/**
 * @brief 底盘控制器配置
 */
class ChassisControllerConfig : public IControllerConfig
{
public:
  explicit ChassisControllerConfig(const std::string& pid_suffix = "pid_follow")
    : pid_suffix_(pid_suffix) {}

  ControllerType getType() const override { return ControllerType::CHASSIS; }
  std::string getTypeName() const override { return "chassis"; }

  std::string getParamPrefix(const std::string& controller_name,
                            const std::string& joint_name) const override
  {
    return "/" + controller_name + "/" + pid_suffix_;
  }

  std::string getPosStateTopic(const std::string& controller_name) const override
  {
    return "/" + controller_name + "/odom";
  }

  std::string getErrorTopic(const std::string& controller_name) const override
  {
    return "/" + controller_name + "/error";
  }

  std::vector<std::string> getSupportedJoints() const override
  {
    return {"follow", "linear_x", "linear_y", "angular_z"};
  }

  void setPidSuffix(const std::string& suffix) { pid_suffix_ = suffix; }

  std::unique_ptr<IControllerConfig> clone() const override
  {
    return std::make_unique<ChassisControllerConfig>(*this);
  }

private:
  std::string pid_suffix_;
};

/**
 * @brief 发射器控制器配置
 */
class ShooterControllerConfig : public IControllerConfig
{
public:
  ControllerType getType() const override { return ControllerType::SHOOTER; }
  std::string getTypeName() const override { return "shooter"; }

  std::string getParamPrefix(const std::string& controller_name,
                            const std::string& joint_name) const override
  {
    return "/" + controller_name + "/controllers/" + joint_name + "/pid";
  }

  std::string getPosStateTopic(const std::string& controller_name) const override
  {
    return "/" + controller_name + "/state";
  }

  std::string getErrorTopic(const std::string& controller_name) const override
  {
    return "/" + controller_name + "/error";
  }

  std::vector<std::string> getSupportedJoints() const override
  {
    return {"friction_left", "friction_right", "trigger"};
  }

  std::unique_ptr<IControllerConfig> clone() const override
  {
    return std::make_unique<ShooterControllerConfig>(*this);
  }
};

/**
 * @brief 自定义控制器配置
 */
class CustomControllerConfig : public IControllerConfig
{
public:
  CustomControllerConfig(const std::string& type_name,
                         const std::string& param_format,
                         const std::string& state_topic_format)
    : type_name_(type_name)
    , param_format_(param_format)
    , state_topic_format_(state_topic_format)
  {}

  ControllerType getType() const override { return ControllerType::CUSTOM; }
  std::string getTypeName() const override { return type_name_; }

  std::string getParamPrefix(const std::string& controller_name,
                            const std::string& joint_name) const override
  {
    std::string result = param_format_;
    // 替换占位符
    size_t pos;
    while ((pos = result.find("${controller}")) != std::string::npos)
    {
      result.replace(pos, 12, controller_name);
    }
    while ((pos = result.find("${joint}")) != std::string::npos)
    {
      result.replace(pos, 8, joint_name);
    }
    return result;
  }

  std::string getPosStateTopic(const std::string& controller_name) const override
  {
    std::string result = state_topic_format_;
    size_t pos;
    while ((pos = result.find("${controller}")) != std::string::npos)
    {
      result.replace(pos, 12, controller_name);
    }
    return result;
  }

  std::string getErrorTopic(const std::string& controller_name) const override
  {
    return "/" + controller_name + "/error";
  }

  std::vector<std::string> getSupportedJoints() const override
  {
    return supported_joints_;
  }

  void setSupportedJoints(const std::vector<std::string>& joints)
  {
    supported_joints_ = joints;
  }

  std::unique_ptr<IControllerConfig> clone() const override
  {
    return std::make_unique<CustomControllerConfig>(*this);
  }

private:
  std::string type_name_;
  std::string param_format_;
  std::string state_topic_format_;
  std::vector<std::string> supported_joints_;
};

/**
 * @brief 控制器配置工厂
 *
 * 用于创建和注册不同类型的控制器配置
 */
class ControllerConfigFactory
{
public:
  using CreatorFunc = std::function<std::unique_ptr<IControllerConfig>()>;

  static ControllerConfigFactory& instance()
  {
    static ControllerConfigFactory factory;
    return factory;
  }

  /**
   * @brief 注册控制器类型
   */
  bool registerController(const std::string& type_name, CreatorFunc creator)
  {
    if (creators_.find(type_name) != creators_.end())
    {
      return false;  // 已存在
    }
    creators_[type_name] = creator;
    return true;
  }

  /**
   * @brief 创建控制器配置
   */
  std::unique_ptr<IControllerConfig> create(const std::string& type_name)
  {
    auto it = creators_.find(type_name);
    if (it != creators_.end())
    {
      return it->second();
    }
    return nullptr;
  }

  /**
   * @brief 根据控制器名称自动检测类型
   */
  std::unique_ptr<IControllerConfig> detectFromName(const std::string& controller_name)
  {
    // 自动检测
    if (controller_name.find("gimbal") != std::string::npos)
    {
      return std::make_unique<GimbalControllerConfig>();
    }
    else if (controller_name.find("chassis") != std::string::npos)
    {
      return std::make_unique<ChassisControllerConfig>();
    }
    else if (controller_name.find("shooter") != std::string::npos)
    {
      return std::make_unique<ShooterControllerConfig>();
    }

    // 未识别的类型
    return nullptr;
  }

  /**
   * @brief 获取所有已注册的类型
   */
  std::vector<std::string> getRegisteredTypes() const
  {
    std::vector<std::string> types;
    for (const auto& pair : creators_)
    {
      types.push_back(pair.first);
    }
    return types;
  }

private:
  ControllerConfigFactory()
  {
    // 注册默认类型
    registerController("gimbal", []() {
      return std::make_unique<GimbalControllerConfig>();
    });
    registerController("chassis", []() {
      return std::make_unique<ChassisControllerConfig>();
    });
    registerController("shooter", []() {
      return std::make_unique<ShooterControllerConfig>();
    });
  }

  std::map<std::string, CreatorFunc> creators_;
};

}  // namespace rm_pid_tuner
