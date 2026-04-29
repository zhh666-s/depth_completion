#ifndef PARAMETER_HPP
#define PARAMETER_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>


class Parameter
{
private:
    std::string m_channel;
    std::vector<double> m_distortion;
    std::vector<int> m_image_size;
    std::vector<double> m_intrinsic;
    std::string m_modality;
    std::vector<double> m_rotation;
    std::string m_target;
    std::vector<double> m_translation;
    std::vector<double> m_undistort_distortion;
    std::vector<double> m_undistort_intrinsic;

public:
    /**
     * @brief 构造函数，从 JSON 文件路径解析参数
     * @param parameter_json_path JSON 文件路径
     * @throws std::runtime_error 如果文件打开失败、JSON 解析错误或缺少必要的键值
     */
    Parameter(const std::string& parameter_json_path);

    /**
     * @brief 从 JSON 对象解析参数
     * @param j JSON 对象
     * @throws std::runtime_error 如果缺少必要的键值或数据格式不正确
     */
    void from_json(const nlohmann::json& j);

    // Getter 方法
    const std::string& getChannel() const { return m_channel; }
    const std::vector<double>& getDistortion() const { return m_distortion; }
    const std::vector<int>& getImageSize() const { return m_image_size; }
    const std::vector<double>& getIntrinsic() const { return m_intrinsic; }
    const std::string& getModality() const { return m_modality; }
    const std::vector<double>& getRotation() const { return m_rotation; }
    const std::string& getTarget() const { return m_target; }
    const std::vector<double>& getTranslation() const { return m_translation; }
    const std::vector<double>& getUndistortDistortion() const { return m_undistort_distortion; }
    const std::vector<double>& getUndistortIntrinsic() const { return m_undistort_intrinsic; }

    // 打印参数
    void printParameters() const;

};


#endif // PARAMETER_HPP
