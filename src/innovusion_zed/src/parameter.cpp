#include "innovusion_zed/parameter.hpp"

Parameter::Parameter(const std::string& parameter_json_path) {
    std::ifstream parameter_file(parameter_json_path);
    if (!parameter_file.is_open()) {
        throw std::runtime_error("Failed to open parameter file: " + parameter_json_path);
    }

    nlohmann::json parameter_json;
    try {
        parameter_file >> parameter_json;
        from_json(parameter_json);
    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        throw std::runtime_error("Failed to parse JSON: " + std::string(e.what()));
    } catch (const nlohmann::json::out_of_range& e) {
        std::cerr << "Error: JSON key not found: " << e.what() << std::endl;
        throw std::runtime_error("JSON key not found: " + std::string(e.what()));
    } catch (const std::runtime_error& e) {
        std::cerr << "Error processing JSON data: " << e.what() << std::endl;
        throw;
    }
}


void Parameter::from_json(const nlohmann::json& j) {
    try {
        if (j.contains("channel")) {
            j.at("channel").get_to(m_channel);
        } else {
            throw std::runtime_error("Key 'channel' not found in JSON.");
        }

        if (j.contains("distortion")) {
            j.at("distortion").get_to(m_distortion);
        } else {
            throw std::runtime_error("Key 'distortion' not found in JSON.");
        }
        
        if (j.contains("image_size")) {
            j.at("image_size").get_to(m_image_size);
        } else {
            throw std::runtime_error("Key 'image_size' not found in JSON.");
        }

        if (j.contains("intrinsic")) {
            j.at("intrinsic").get_to(m_intrinsic);
        } else {
            throw std::runtime_error("Key 'intrinsic' not found in JSON.");
        }

        if (j.contains("modality")) {
            j.at("modality").get_to(m_modality);
        } else {
            throw std::runtime_error("Key 'modality' not found in JSON.");
        }

        if (j.contains("rotation")) {
            j.at("rotation").get_to(m_rotation);
        } else {
            throw std::runtime_error("Key 'rotation' not found in JSON.");
        }

        if (j.contains("target")) {
            j.at("target").get_to(m_target);
        } else {
            throw std::runtime_error("Key 'target' not found in JSON.");
        }

        if (j.contains("translation")) {
            j.at("translation").get_to(m_translation);
        } else {
            throw std::runtime_error("Key 'translation' not found in JSON.");
        }

        if (j.contains("undistort_distortion")) {
            j.at("undistort_distortion").get_to(m_undistort_distortion);
        } else {
            throw std::runtime_error("Key 'undistort_distortion' not found in JSON.");
        }

        if (j.contains("undistort_intrinsic")) {
            j.at("undistort_intrinsic").get_to(m_undistort_intrinsic);
        } else {
            throw std::runtime_error("Key 'undistort_intrinsic' not found in JSON.");
        }
    } catch (const nlohmann::json::type_error& e) {
        std::cerr << "Error: JSON type error: " << e.what() << std::endl;
        throw std::runtime_error("JSON type error: " + std::string(e.what()));
    }
}

void Parameter::printParameters() const {
    std::cout << "Channel: " << m_channel << std::endl;
    std::cout << "Distortion: ";
    for (double d : m_distortion) std::cout << d << " ";
    std::cout << std::endl;
    std::cout << "Image Size: ";
    for (int s : m_image_size) std::cout << s << " ";
    std::cout << "Intrinsic: ";
    for (double i : m_intrinsic) std::cout << i << " ";
    std::cout << std::endl;
    std::cout << "Modality: " << m_modality << std::endl;
    std::cout << "Rotation: ";
    for (double r : m_rotation) std::cout << r << " ";
    std::cout << std::endl;
    std::cout << "Target: " << m_target << std::endl;
    std::cout << "Translation: ";
    for (double t : m_translation) std::cout << t << " ";
    std::cout << std::endl;
    std::cout << "Undistort Distortion: ";
    for (double ud : m_undistort_distortion) std::cout << ud << " ";
    std::cout << std::endl;
    std::cout << "Undistort Intrinsic: ";
    for (double ui : m_undistort_intrinsic) std::cout << ui << " ";
    std::cout << std::endl;
}